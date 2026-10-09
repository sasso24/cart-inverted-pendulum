/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "network.h"
#include "network_data.h"
#include "stima.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIRCOSTEP 32
#define STEP_CHANNEL TIM_CHANNEL_1
#define MIN_FREQUENCY (MIRCOSTEP * 100.0f)
#define MAX_FREQUENCY (MIRCOSTEP * 1000.0f)
#define STEPPER_ACCELERATION_HZ_PER_S 1600000.0f

// tutte i punti di rif sono da dietro
#define DIR_FORWARD  GPIO_PIN_SET   //SInistra (da dietro)
#define DIR_BACKWARD GPIO_PIN_RESET

#define HOMING_VELOCITY_HZ  MIN_FREQUENCY*2
#define HOMING_TIMEOUT_MS  120000U
#define BUTTON_RELEASE_MS  50U
#define THETA_STABILE_MS    1000U
#define THETA_TIMEOUT_MS   30000U

/* Contratto di cart_pendolo_velocita_rl/export_stm32/LEGGIMI.md. */
#define CONTROL_PERIOD_MS       20U
#define MOTOR_PERIOD_MS          2U
#define CONTROL_TIMEOUT_MS      40U
#define METRI_PER_STEP           (0.002f * 48.0f / (200.0f * MIRCOSTEP))
#define POSIZIONE_SCALA_M        0.462f
#define VELOCITA_SCALA_M_S       0.48f
#define OMEGA_SCALA_RAD_S        10.0f

/* Hardware-in-the-loop: 1 = la scheda NON muove il motore e non legge
 * l'encoder; riceve dal PC (hil.py, USART2 dell'ST-LINK) le misure simulate,
 * esegue stimatore e rete e restituisce il comando. 0 = funzionamento normale.
 * Rimettere a 0 prima di collegare il motore.
 */
#ifndef HIL_MODE
#define HIL_MODE 0
#endif

#if AI_NETWORK_IN_1_SIZE != 6 || AI_NETWORK_OUT_1_SIZE != 1
#error "La rete deve avere sei ingressi e una uscita"
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Var encoder
#define ENCODER_CPR  2400
#define HALF_CPR     1200
#define PI_F         3.14159265358979323846f

// Metti -1 se vuoi invertire il verso positivo dell'angolo.
#define ENCODER_SIGN  -1

static uint32_t conteggio_precedente = 0;
static int32_t posizione = HALF_CPR;

volatile float theta_deg = 180.0f;
volatile float theta_rad = PI_F;
/* Le inizializzazioni statiche sopra non sono ancora un riferimento valido. */
volatile uint8_t theta_inizializzato = 0;

// Var stepper
/* Soglia di partenza in Hz, modificabile prima di ogni movimento. */
float max_starting_frequency = 1000.0f;
static float stepper_target_frequency = MAX_FREQUENCY;
static float stepper_counter_clock;
static float stepper_start_frequency;
static uint32_t stepper_ramp_start_ms;
static uint32_t stepper_ramp_last_ms;
static volatile uint8_t stepper_ramp_active = 0;
volatile uint32_t step_count = 0;
volatile uint32_t step_target = 0;
volatile uint8_t stepper_running = 0;
static volatile GPIO_PinState stepper_direction = DIR_FORWARD;
static volatile uint8_t arresto_finecorsa = 0;

/* Riferimento del carrello: zero al finecorsa sinistro, positivo a destra.
 * Stima in impulsi STEP comandati, valida solo dopo homing riuscito.
 */
volatile int32_t posizione_carrello_steps = 0;
volatile int32_t posizione_carrello_min_steps = 0;
volatile int32_t posizione_carrello_max_steps = 0;
volatile uint32_t corsa_totale_steps = 0;
static volatile uint8_t riferimento_carrello_valido = 0;
volatile uint8_t carrello_inizializzato = 0;
volatile HAL_StatusTypeDef inizializzazione_esito = HAL_ERROR;

/* Il main e' l'unico contesto che puo' abilitare il controllo.
 * L'ISR puo' solo richiedere un avvio oppure fermare immediatamente.
 */
volatile uint8_t controllo_abilitato = 0;
static volatile uint8_t richiesta_avvio = 0;
static volatile uint8_t pulsante_pronto = 0;
static volatile uint8_t pulsante_inizializzato = 0;
static volatile uint32_t pulsante_rilascio_ms = 0;

/* Diagnostica osservabile dal debugger: 0=OK, 1=init rete, 2=inferenza,
 * 3=NaN/Inf, 4=finecorsa/limite, 5=deadline, 6=avvio PWM, 7=calibrazione theta. */
volatile uint32_t controllo_errore = 0;
volatile uint8_t rete_pronta = 0;
volatile float rete_osservazione[6];
volatile float rete_azione = 0.0f;
volatile float frequenza_richiesta_hz = 0.0f;
volatile float frequenza_applicata_hz = 0.0f;
volatile float velocita_carrello_m_s = 0.0f;
volatile float velocita_angolare_rad_s = 0.0f;
volatile uint32_t rete_inferenze = 0;
volatile uint32_t rete_durata_ms = 0;
static ai_handle rete = AI_HANDLE_NULL;
AI_ALIGNED(4) static ai_u8 rete_activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
static ai_buffer *rete_input;
static ai_buffer *rete_output;
volatile ai_error rete_ultimo_errore;
static volatile float frequenza_rampa_hz = 0.0f;
static volatile uint8_t stop_al_fronte_step = 0;
static volatile uint32_t ultimo_comando_ms = 0;
static uint32_t ultimo_motore_ms = 0;
static uint32_t ultimo_campione_ms = 0;
static int32_t ultimo_carrello_steps = 0;
static float ultimo_theta_rad = PI_F;

/* Stimatore dello stato (stima.c), lo stesso usato in addestramento.
 * Predizione in SysTick ogni 2 ms, correzione in Rete_Process ogni 20 ms.
 * La rete riceve x, theta, v, omega STIMATI, non derivati dalle letture.
 */
Stima stima;                         /* osservabile dal debugger */
static volatile uint8_t stima_valida = 0;
static uint32_t ultimo_stima_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
static void Controllo_Fault(uint32_t errore);
static void Rete_Process(void);
void Controllo_StimaTick(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Prima dell'abilitazione di B1, l'homing iniziale e' indipendente
 * dal comando del controllo. Dopo, ogni avvio richiede l'abilitazione.
 */
static uint8_t Stepper_MovimentoConsentito(void)
{
    return !pulsante_inizializzato || controllo_abilitato;
}

/* Attesa interrompibile: le vecchie sequenze devono uscire prima che
 * il main possa accettare una nuova richiesta di avvio.
 */
static uint8_t Controllo_Attendi(uint32_t durata_ms)
{
    uint32_t inizio = HAL_GetTick();
    while (Stepper_MovimentoConsentito()
           && (uint32_t)(HAL_GetTick() - inizio) < durata_ms)
        HAL_Delay(1);
    return Stepper_MovimentoConsentito();
}

/* Chiamata da SysTick durante i movimenti bloccanti nel main.
 * Nessun ritardo sul primo fronte di pressione; un'altra pressione viene
 * accettata solo dopo almeno 50 ms di rilascio stabile (PC13 alto).
 */
void Controllo_ButtonTick(void)
{
    if (!pulsante_inizializzato)
        return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
        pulsante_rilascio_ms = 0;
    else if (pulsante_rilascio_ms < BUTTON_RELEASE_MS)
        ++pulsante_rilascio_ms;
    else
        pulsante_pronto = 1;
    __set_PRIMASK(primask);
}

void Leggi_theta(){
	uint32_t conteggio = __HAL_TIM_GET_COUNTER(&htim2);

	int64_t delta = (int64_t)conteggio
				- (int64_t)conteggio_precedente;

	conteggio_precedente = conteggio;

	// Gestione del passaggio attraverso lo zero del timer a 32 bit.
	if (delta > 2147483647LL)
	  delta -= 4294967296LL;
	else if (delta < -2147483648LL)
	  delta += 4294967296LL;

	// Aggiorna la posizione e la riporta entro un giro.
	int64_t nuova_posizione =
	  ((int64_t)posizione + ENCODER_SIGN * delta) % ENCODER_CPR;

	// Intervallo conteggi: (-1200, +1200].
	if (nuova_posizione > HALF_CPR)
	  nuova_posizione -= ENCODER_CPR;
	else if (nuova_posizione <= -HALF_CPR)
	  nuova_posizione += ENCODER_CPR;

	posizione = (int32_t)nuova_posizione;

	// Le due uscite richieste.
	theta_deg = (float)posizione * (360.0f / ENCODER_CPR);
	theta_rad = (float)posizione * (2.0f * PI_F / ENCODER_CPR);
}

/* Pull-down esterno: 1 quando il contatto porta PC1 a livello alto,
 * 0 quando l'ingresso e' basso. Lettura istantanea senza antirimbalzo.
 */
uint8_t FinecorsaSinistro_Premuto(void)
{
    return HAL_GPIO_ReadPin(FC_sx_GPIO_Port, FC_sx_Pin) == GPIO_PIN_SET;
}

/* Come il sinistro: pull-down esterno, contatto premuto a livello alto. */
uint8_t FinecorsaDestro_Premuto(void)
{
    return HAL_GPIO_ReadPin(Fc_dx_GPIO_Port, Fc_dx_Pin) == GPIO_PIN_SET;
}

static uint8_t FinecorsaDirezione_Premuto(GPIO_PinState direction)
{
    return (direction == DIR_FORWARD) ? FinecorsaSinistro_Premuto()
                                      : FinecorsaDestro_Premuto();
}

void Stepper_SetDirection(GPIO_PinState dir)
{
    stepper_direction = dir;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, dir);
}

void Stepper_Stop(void)
{
    HAL_TIM_PWM_Stop_IT(&htim3, STEP_CHANNEL);
    stepper_running = 0;
    stepper_ramp_active = 0;
    frequenza_applicata_hz = 0.0f;
    stop_al_fronte_step = 0;
}

/* Prima dell'homing, a motore fermo: lasciare il pendolo libero verso
 * il basso. L'encoder incrementale verifica la quiete, non la verticalita'.
 * Ogni variazione del contatore fa ripartire l'attesa di un secondo.
 */
static HAL_StatusTypeDef Inizializza_Theta(void)
{
    Stepper_Stop();
    theta_inizializzato = 0;
    uint32_t inizio = HAL_GetTick();
    uint32_t stabile_da = inizio;
    uint32_t riferimento = __HAL_TIM_GET_COUNTER(&htim2);

    while ((uint32_t)(HAL_GetTick() - inizio) < THETA_TIMEOUT_MS)
    {
        uint32_t conteggio = __HAL_TIM_GET_COUNTER(&htim2);
        uint32_t now = HAL_GetTick();
        if (conteggio != riferimento)
        {
            riferimento = conteggio;
            stabile_da = now;
        }
        else if ((uint32_t)(now - stabile_da) >= THETA_STABILE_MS)
        {
            /* Usa il conteggio acquisito come origine senza resettare TIM2:
             * gli impulsi successivi, anche durante l'homing, restano validi. */
            conteggio_precedente = riferimento;
            posizione = HALF_CPR;
            theta_deg = 180.0f;
            theta_rad = PI_F;
            ultimo_theta_rad = theta_rad;
            velocita_angolare_rad_s = 0.0f;
            theta_inizializzato = 1;
            return HAL_OK;
        }
        HAL_Delay(1);
    }
    return HAL_TIMEOUT;
}

static void Controllo_Disabilita(void)
{
    controllo_abilitato = 0;
    frequenza_richiesta_hz = 0.0f;
    frequenza_rampa_hz = 0.0f;
    rete_azione = 0.0f;
    Stepper_Stop();
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

/* Chiamare solo dal main, dopo l'uscita da ogni sequenza precedente. */
static void Controllo_AccettaAvvio(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (richiesta_avvio)
    {
        richiesta_avvio = 0;
        if (rete_pronta && theta_inizializzato
            && carrello_inizializzato && riferimento_carrello_valido
            && !FinecorsaSinistro_Premuto() && !FinecorsaDestro_Premuto()
            && posizione_carrello_steps > posizione_carrello_min_steps
            && posizione_carrello_steps < posizione_carrello_max_steps)
        {
            controllo_errore = 0;
            arresto_finecorsa = 0;
            frequenza_richiesta_hz = 0.0f;
            frequenza_rampa_hz = 0.0f;
            ultimo_comando_ms = HAL_GetTick();
            ultimo_motore_ms = ultimo_comando_ms;
            controllo_abilitato = 1;
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        }
    }
    __set_PRIMASK(primask);
}

/* Check e avvio devono essere indivisibili rispetto all'ISR di stop.
 * Sezione breve, senza attese: HAL_TIM_PWM_Start_IT scrive solo registri.
 * Nel main gli avvii richiedono B1; l'homing iniziale resta autonomo.
 */
static void Stepper_StartIfEnabled(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (Stepper_MovimentoConsentito())
    {
        stepper_running = 1;
        if (HAL_TIM_PWM_Start_IT(&htim3, STEP_CHANNEL) != HAL_OK)
            Stepper_Stop();
    }
    else
        Stepper_Stop();
    __set_PRIMASK(primask);
}

static float Stepper_ClampFrequency(float frequency)
{
    if (!isfinite(frequency) || frequency < MIN_FREQUENCY)
        return MIN_FREQUENCY;
    if (frequency > MAX_FREQUENCY)
        return MAX_FREQUENCY;
    return frequency;
}

/* ARR e CCR vengono applicati insieme al prossimo periodo PWM.
 * Non generare UG durante il moto: troncherebbe il periodo corrente.
 */
static void Stepper_ApplyFrequency(float frequency)
{
    float period = ceilf(stepper_counter_clock / frequency);
    if (period > 65536.0f)
        period = 65536.0f;
    else if (period < 2.0f)
        period = 2.0f;
    uint32_t ticks = (uint32_t)period;

    /* Protegge anche il read/modify/write di CR1 da uno stop concorrente. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    SET_BIT(htim3.Instance->CR1, TIM_CR1_UDIS);
    __HAL_TIM_SET_AUTORELOAD(&htim3, ticks - 1U);
    __HAL_TIM_SET_COMPARE(&htim3, STEP_CHANNEL, ticks / 2U);
    CLEAR_BIT(htim3.Instance->CR1, TIM_CR1_UDIS);
    __set_PRIMASK(primask);
}

/* Chiamata a timer fermo per OGNI nuovo movimento, anche dopo uno stop
 * anticipato. La frequenza obiettivo resta separata da quella della rampa.
 */
static void Stepper_PrepareRamp(void)
{
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
        timer_clock *= 2U;
    stepper_counter_clock = (float)timer_clock / (htim3.Instance->PSC + 1U);

    stepper_target_frequency = Stepper_ClampFrequency(stepper_target_frequency);
    float starting_limit = Stepper_ClampFrequency(max_starting_frequency);
    stepper_start_frequency = fminf(stepper_target_frequency, starting_limit);
    stepper_ramp_active = stepper_target_frequency > stepper_start_frequency;
    Stepper_ApplyFrequency(stepper_start_frequency);
    htim3.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE | TIM_FLAG_CC1);
    stepper_ramp_start_ms = HAL_GetTick();
    stepper_ramp_last_ms = stepper_ramp_start_ms;
}

/* Aggiornamento non bloccante dal callback degli impulsi STEP.
 * La rampa e' lineare nel tempo e termina esattamente all'obiettivo.
 */
static void Stepper_UpdateRamp(void)
{
    if (!stepper_ramp_active)
        return;
    uint32_t now = HAL_GetTick();
    if (now == stepper_ramp_last_ms)
        return;
    stepper_ramp_last_ms = now;
    float frequency = stepper_start_frequency
        + STEPPER_ACCELERATION_HZ_PER_S * 0.001f
        * (uint32_t)(now - stepper_ramp_start_ms);
    if (frequency >= stepper_target_frequency)
    {
        frequency = stepper_target_frequency;
        stepper_ramp_active = 0;
    }
    Stepper_ApplyFrequency(frequency);
}

void Stepper_MoveSteps(uint32_t steps)
{
    Stepper_Stop();
    step_count = 0;
    step_target = steps;
    arresto_finecorsa = 0;

    if (!Stepper_MovimentoConsentito())
        return;
    if (FinecorsaDirezione_Premuto(stepper_direction))
    {
        arresto_finecorsa = 1;
        return;
    }
    if (steps == 0U)
        return;

    Stepper_PrepareRamp();
    Stepper_StartIfEnabled();
}

void Stepper_Move(GPIO_PinState direction, uint32_t steps)
{
    Stepper_Stop();
    Stepper_SetDirection(direction);
    if (!Controllo_Attendi(1))
        return;
    Stepper_MoveSteps(steps);

    while (stepper_running)
    {
        if (FinecorsaDirezione_Premuto(direction))
        {
            Stepper_Stop();
            arresto_finecorsa = 1;
        }
    }
}

/* velocity: impulsi STEP al secondo, limitati a MIN/MAX_FREQUENCY.
 * Valori non positivi/non finiti fermano il motore. Ogni chiamata riparte
 * al massimo da max_starting_frequency e accelera fino a velocity.
 */
void Stepper_set_velocity(GPIO_PinState direction, float velocity)
{
    Stepper_Stop();
    step_count = 0;
    step_target = 0; /* Zero identifica il movimento continuo. */
    arresto_finecorsa = 0;

    if (!Stepper_MovimentoConsentito() || !isfinite(velocity) || velocity <= 0.0f)
        return;

    stepper_target_frequency = Stepper_ClampFrequency(velocity);
    Stepper_SetDirection(direction);
    if (!Controllo_Attendi(1))
        return;
    if (FinecorsaDirezione_Premuto(direction))
    {
        arresto_finecorsa = 1;
        return;
    }

    Stepper_PrepareRamp();
    Stepper_StartIfEnabled();
}

/* Da qualsiasi posizione cerca lentamente il finecorsa destro e azzera
 * il conteggio; misura poi la corsa fino a sinistra e torna al centro a 8 kHz.
 * Il riferimento finale resta zero a sinistra, positivo a destra.
 * Chiamare dal main dopo MX_TIM3_Init(), mai da un interrupt.
 */
void Inizializza(void)
{
    Stepper_Stop();
    carrello_inizializzato = 0;
    riferimento_carrello_valido = 0;
    inizializzazione_esito = HAL_BUSY;
    arresto_finecorsa = 0;
    corsa_totale_steps = 0;
    posizione_carrello_min_steps = 0;
    posizione_carrello_max_steps = 0;
    posizione_carrello_steps = 0;

    /* Il controllo direzionale consente di partire anche da un finecorsa.
     * Se siamo gia' a destra, il motore non emette alcun impulso.
     */
    Stepper_set_velocity(DIR_BACKWARD, HOMING_VELOCITY_HZ);
    uint32_t inizio = HAL_GetTick();
    while (stepper_running)
    {
        /* L'ISR arresta il moto al termine della fase alta dello STEP. */
        if ((uint32_t)(HAL_GetTick() - inizio) >= HOMING_TIMEOUT_MS)
        {
            Stepper_Stop();
            inizializzazione_esito = HAL_TIMEOUT;
            return;
        }
        HAL_Delay(1);
    }

    Stepper_Stop();
    HAL_Delay(20);
    if (!arresto_finecorsa || !FinecorsaDestro_Premuto()
        || FinecorsaSinistro_Premuto())
    {
        inizializzazione_esito = HAL_ERROR;
        return;
    }

    /* Zero della misura della corsa al contatto destro confermato. */
    step_count = 0;
    posizione_carrello_steps = 0;
    Stepper_set_velocity(DIR_FORWARD, HOMING_VELOCITY_HZ);
    inizio = HAL_GetTick();
    while (stepper_running)
    {
        /* L'ISR conta l'impulso e controlla il finecorsa al termine
         * della fase alta: non troncare qui un impulso da contare.
         */
        if ((uint32_t)(HAL_GetTick() - inizio) >= HOMING_TIMEOUT_MS)
        {
            Stepper_Stop();
            inizializzazione_esito = HAL_TIMEOUT;
            return;
        }
        HAL_Delay(1);
    }

    Stepper_Stop();
    HAL_Delay(20); /* Conferma del contatto a motore fermo. */
    if (!arresto_finecorsa || !FinecorsaSinistro_Premuto()
        || step_count < 2U || step_count > INT32_MAX)
    {
        inizializzazione_esito = HAL_ERROR;
        return;
    }

    /* Salvare la misura prima che il prossimo movimento azzeri step_count. */
    corsa_totale_steps = step_count;
    posizione_carrello_min_steps = 0;
    posizione_carrello_max_steps = (int32_t)corsa_totale_steps;
    posizione_carrello_steps = posizione_carrello_min_steps;
    riferimento_carrello_valido = 1;

    /* Con corsa dispari, il centro e' arrotondato verso sinistra. */
    uint32_t passi_centro = corsa_totale_steps / 2U;
    Stepper_SetDirection(DIR_BACKWARD);
    HAL_Delay(1);
    stepper_target_frequency = MAX_FREQUENCY;
    Stepper_MoveSteps(passi_centro);
    inizio = HAL_GetTick();
    while (stepper_running)
    {
        if ((uint32_t)(HAL_GetTick() - inizio) >= HOMING_TIMEOUT_MS)
        {
            Stepper_Stop();
            riferimento_carrello_valido = 0;
            inizializzazione_esito = HAL_TIMEOUT;
            return;
        }
        HAL_Delay(1);
    }

    if (arresto_finecorsa || step_count != passi_centro)
    {
        riferimento_carrello_valido = 0;
        inizializzazione_esito = HAL_ERROR;
        return;
    }
    carrello_inizializzato = 1;
    inizializzazione_esito = HAL_OK;
}

static void Controllo_Fault(uint32_t errore)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    controllo_errore = errore;
    richiesta_avvio = 0;
    Controllo_Disabilita();
    __set_PRIMASK(primask);
}

static uint8_t Rete_Init(void)
{
    const ai_handle activations[] = { AI_HANDLE_PTR(rete_activations) };
    rete_ultimo_errore = ai_network_create_and_init(&rete, activations, NULL);
    if (rete_ultimo_errore.type != AI_ERROR_NONE)
        return 0;
    rete_input = ai_network_inputs_get(rete, NULL);
    rete_output = ai_network_outputs_get(rete, NULL);
    if (rete_input == NULL || rete_output == NULL
        || rete_input[0].data == AI_HANDLE_NULL
        || rete_output[0].data == AI_HANDLE_NULL)
        return 0;
    rete_pronta = 1;
    return 1;
}

/* Eseguita da SysTick: rampa indipendente dal tempo di inferenza.
 * Non chiama HAL_Delay e non esegue la rete in interrupt.
 * L'inversione attende il fronte di discesa STEP e lascia DIR stabile
 * almeno un intervallo di rampa prima di riavviare il timer.
 */
void Controllo_MotorTick(void)
{
    if (!pulsante_inizializzato || !controllo_abilitato)
        return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t now = HAL_GetTick();
    /* Ricontrollo dopo la mascheratura: B1 puo' aver interrotto SysTick. */
    if (!controllo_abilitato)
        goto done;
    if (FinecorsaSinistro_Premuto() || FinecorsaDestro_Premuto()
        || posizione_carrello_steps <= posizione_carrello_min_steps
        || posizione_carrello_steps >= posizione_carrello_max_steps)
    {
        Controllo_Fault(4);
        goto done;
    }
    if ((uint32_t)(now - ultimo_comando_ms) >= CONTROL_TIMEOUT_MS)
    {
        Controllo_Fault(5);
        goto done;
    }
    if ((uint32_t)(now - ultimo_motore_ms) < MOTOR_PERIOD_MS)
        goto done;
    ultimo_motore_ms = now;

    /* Un solo passo da 2 ms; nessun salto di accelerazione dopo un ritardo. */
    float delta_max = STEPPER_ACCELERATION_HZ_PER_S * MOTOR_PERIOD_MS / 1000.0f;
    float delta = frequenza_richiesta_hz - frequenza_rampa_hz;
    frequenza_rampa_hz += fmaxf(-delta_max, fminf(delta, delta_max));
    float hz = fabsf(frequenza_rampa_hz);
    GPIO_PinState dir = frequenza_rampa_hz > 0.0f ? DIR_BACKWARD : DIR_FORWARD;

    /* TIM3 a 16 bit: sotto la frequenza rappresentabile non emettere STEP. */
    if (hz < stepper_counter_clock / 65536.0f
        || (stepper_running && dir != stepper_direction))
    {
        if (stepper_running)
            stop_al_fronte_step = 1;
        goto done;
    }
    if (stop_al_fronte_step)
        goto done;
    if (!stepper_running && dir != stepper_direction)
    {
        Stepper_SetDirection(dir);
        goto done;
    }
    Stepper_ApplyFrequency(hz);
    if (!stepper_running)
    {
        step_count = 0;
        step_target = 0;
        stepper_ramp_active = 0; /* La rampa RL sostituisce quella di homing. */
        htim3.Instance->EGR = TIM_EGR_UG;
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE | TIM_FLAG_CC1);
        Stepper_StartIfEnabled();
        if (!stepper_running)
        {
            Controllo_Fault(6);
            goto done;
        }
    }
    /* Frequenza quantizzata realmente programmata, non l'uscita della rete. */
    frequenza_applicata_hz = (dir == DIR_BACKWARD ? 1.0f : -1.0f)
        * stepper_counter_clock / (float)(htim3.Instance->ARR + 1U);
done:
    __set_PRIMASK(primask);
}

/* I sei ingressi della rete dalla stima: stesse scale dell'addestramento. */
static void Rete_Osservazione(const Stima *st, float applied_hz, volatile float obs[6])
{
    obs[0] = st->x / POSIZIONE_SCALA_M;
    obs[1] = sinf(st->theta);
    obs[2] = cosf(st->theta);
    obs[3] = st->velocita / VELOCITA_SCALA_M_S;
    obs[4] = st->omega / OMEGA_SCALA_RAD_S;
    obs[5] = applied_hz / MAX_FREQUENCY;
}

/* Uscita della rete [-1,1] -> frequenza STEP firmata, come stepper.py. */
static float Rete_AzioneInHz(float action)
{
    return action == 0.0f ? 0.0f
        : copysignf(MIN_FREQUENCY + fabsf(action) * (MAX_FREQUENCY - MIN_FREQUENCY), action);
}

/* Eseguita da SysTick dopo Controllo_MotorTick: predizione dello stimatore
 * ogni 2 ms con la velocita' STEP effettivamente programmata (zero se il
 * timer e' fermo, anche durante la pausa per il cambio di DIR).
 * Fuori dal controllo il carrello e' considerato fermo.
 */
void Controllo_StimaTick(void)
{
    if (!stima_valida)
        return;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - ultimo_stima_ms) < MOTOR_PERIOD_MS)
        return;
    ultimo_stima_ms = now;
    float v = (controllo_abilitato && stepper_running)
        ? frequenza_applicata_hz * METRI_PER_STEP : 0.0f;
    Stima_Predici(&stima, v, MOTOR_PERIOD_MS * 0.001f);
}

/* Campionamento anche a controllo fermo: lo stimatore resta aggiornato.
 * La posizione del carrello viene dagli STEP emessi (nessun encoder lineare);
 * le velocita' vengono dallo stimatore, non da differenze fra letture.
 */
static void Rete_Process(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = (uint32_t)(now - ultimo_campione_ms);
    if (elapsed < CONTROL_PERIOD_MS)
        return;
    ultimo_campione_ms = now;
    Leggi_theta();
    int32_t cart_steps;
    float applied_hz;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    cart_steps = posizione_carrello_steps;
    applied_hz = frequenza_applicata_hz;
    __set_PRIMASK(primask);
    float x_misurata = (cart_steps - 0.5f * (float)corsa_totale_steps) * METRI_PER_STEP;
    ultimo_theta_rad = theta_rad;
    ultimo_carrello_steps = cart_steps;

    /* Correzione dello stimatore; SysTick non deve predire a meta' aggiornamento. */
    Stima st;
    primask = __get_PRIMASK();
    __disable_irq();
    if (!stima_valida)
    {
        /* Prima lettura dopo l'homing: pendolo fermo, carrello fermo. */
        Stima_Reset(&stima, x_misurata, theta_rad, 0.0f);
        ultimo_stima_ms = HAL_GetTick();
        stima_valida = 1;
    }
    else
        Stima_Correggi(&stima, x_misurata, theta_rad);
    /* A controllo fermo il conteggio STEP e' esatto: niente deriva della stima. */
    if (!controllo_abilitato)
        stima.x = x_misurata;
    st = stima;
    __set_PRIMASK(primask);
    velocita_carrello_m_s = st.velocita;
    velocita_angolare_rad_s = st.omega;

    Rete_Osservazione(&st, applied_hz, rete_osservazione);
    if (!controllo_abilitato || !rete_pronta)
        return;
    ai_float *input = (ai_float *)rete_input[0].data;
    for (uint32_t i = 0; i < 6U; ++i)
    {
        if (!isfinite(rete_osservazione[i]))
        {
            Controllo_Fault(3);
            return;
        }
        input[i] = rete_osservazione[i];
    }
    uint32_t start = HAL_GetTick();
    ai_i32 batches = ai_network_run(rete, rete_input, rete_output);
    rete_durata_ms = (uint32_t)(HAL_GetTick() - start);
    if (batches != 1)
    {
        rete_ultimo_errore = ai_network_get_error(rete);
        rete_pronta = 0;
        Controllo_Fault(2);
        return;
    }
    float action = ((ai_float *)rete_output[0].data)[0];
    if (!isfinite(action))
    {
        Controllo_Fault(3);
        return;
    }
    action = fmaxf(-1.0f, fminf(action, 1.0f));
    float requested = Rete_AzioneInHz(action);
    primask = __get_PRIMASK();
    __disable_irq();
    /* Uno stop durante ai_network_run non puo' essere annullato dal risultato. */
    if (controllo_abilitato)
    {
        if ((uint32_t)(HAL_GetTick() - now) >= CONTROL_PERIOD_MS)
            Controllo_Fault(5);
        else
        {
            rete_azione = action;
            frequenza_richiesta_hz = requested;
            ultimo_comando_ms = HAL_GetTick();
            ++rete_inferenze;
        }
    }
    __set_PRIMASK(primask);
}

#if HIL_MODE
/* ------------------------------------------------------------------------
 * Hardware-in-the-loop (vedi HIL.md). Protocollo su USART2, little-endian:
 *   PC -> scheda: A5 5A | HilRichiesta (58 byte) | checksum
 *   scheda -> PC: A5 5A | HilRisposta  (60 byte) | checksum
 * checksum = complemento a due della somma dei byte del corpo.
 * comando 0 = inizio episodio (Stima_Reset), 1 = passo da 20 ms
 * (n predizioni da 2 ms con le velocita' v[], poi correzione), 2 = ping.
 * Una richiesta con lo stesso seq della precedente non viene rieseguita:
 * si rispedisce la risposta precedente (ritrasmissione dopo un timeout).
 * ------------------------------------------------------------------------ */
#define HIL_SYNC0 0xA5U
#define HIL_SYNC1 0x5AU
#define HIL_MAX_PREDIZIONI 10U

typedef struct __attribute__((packed))
{
    uint8_t comando;
    uint8_t n;
    uint32_t seq;
    float v[HIL_MAX_PREDIZIONI];  /* m/s dopo la rampa, uno ogni 2 ms */
    float x;                      /* m, misura simulata rispetto al centro */
    float theta;                  /* rad, misura simulata (0 in alto) */
    float applied_hz;             /* frequenza STEP applicata, firmata */
} HilRichiesta;

typedef struct __attribute__((packed))
{
    uint32_t seq;
    uint32_t errore;              /* stessi codici di controllo_errore; 8 = richiesta non valida */
    float azione;
    float richiesta_hz;
    float osservazione[6];
    float stima_x, stima_theta, stima_omega;
    uint32_t rete_us;             /* durata di ai_network_run */
    uint32_t totale_us;           /* stimatore + rete */
} HilRisposta;

_Static_assert(sizeof(HilRichiesta) == 58, "HilRichiesta");
_Static_assert(sizeof(HilRisposta) == 60, "HilRisposta");

static Stima hil_stima;
static HilRisposta hil_risposta;
volatile uint32_t hil_pacchetti = 0;
volatile uint32_t hil_scartati = 0;

static uint8_t Hil_Checksum(const uint8_t *dati, uint32_t n)
{
    uint8_t somma = 0;
    for (uint32_t i = 0; i < n; ++i)
        somma = (uint8_t)(somma + dati[i]);
    return (uint8_t)(0U - somma);
}

static uint8_t Hil_Ricevi(HilRichiesta *richiesta)
{
    uint8_t b = 0;
    do
    {
        if (HAL_UART_Receive(&huart2, &b, 1, HAL_MAX_DELAY) != HAL_OK)
            return 0;
        if (b == HIL_SYNC0
            && HAL_UART_Receive(&huart2, &b, 1, 20) == HAL_OK && b == HIL_SYNC1)
            break;
    } while (1);
    uint8_t corpo[sizeof(HilRichiesta) + 1U];
    if (HAL_UART_Receive(&huart2, corpo, sizeof corpo, 100) != HAL_OK)
        return 0;
    if (Hil_Checksum(corpo, sizeof(HilRichiesta)) != corpo[sizeof(HilRichiesta)])
        return 0;
    memcpy(richiesta, corpo, sizeof *richiesta);
    return 1;
}

static void Hil_Invia(const HilRisposta *risposta)
{
    uint8_t pacchetto[2U + sizeof(HilRisposta) + 1U];
    pacchetto[0] = HIL_SYNC0;
    pacchetto[1] = HIL_SYNC1;
    memcpy(&pacchetto[2], risposta, sizeof *risposta);
    pacchetto[sizeof pacchetto - 1U] = Hil_Checksum(&pacchetto[2], sizeof(HilRisposta));
    HAL_UART_Transmit(&huart2, pacchetto, sizeof pacchetto, 100);
}

static uint32_t Hil_Microsecondi(uint32_t cicli)
{
    return cicli / (SystemCoreClock / 1000000U);
}

/* Elabora una richiesta valida: stesso stimatore e stessa rete del controllo. */
static void Hil_Elabora(const HilRichiesta *r, HilRisposta *out)
{
    memset(out, 0, sizeof *out);
    out->seq = r->seq;
    if (r->comando == 2U)
        return;
    uint8_t valida = (r->comando <= 1U) && r->n <= HIL_MAX_PREDIZIONI
        && isfinite(r->x) && isfinite(r->theta) && isfinite(r->applied_hz);
    for (uint32_t i = 0; valida && i < r->n; ++i)
        valida = isfinite(r->v[i]);
    if (!valida || !rete_pronta)
    {
        out->errore = !rete_pronta ? 2U : 8U;
        return;
    }
    uint32_t inizio = DWT->CYCCNT;
    if (r->comando == 0U)
        Stima_Reset(&hil_stima, r->x, r->theta, 0.0f);
    else
    {
        for (uint32_t i = 0; i < r->n; ++i)
            Stima_Predici(&hil_stima, r->v[i], MOTOR_PERIOD_MS * 0.001f);
        Stima_Correggi(&hil_stima, r->x, r->theta);
    }
    Rete_Osservazione(&hil_stima, r->applied_hz, rete_osservazione);
    ai_float *input = (ai_float *)rete_input[0].data;
    for (uint32_t i = 0; i < 6U; ++i)
    {
        out->osservazione[i] = rete_osservazione[i];
        input[i] = rete_osservazione[i];
    }
    out->stima_x = hil_stima.x;
    out->stima_theta = hil_stima.theta;
    out->stima_omega = hil_stima.omega;
    uint32_t inizio_rete = DWT->CYCCNT;
    ai_i32 batches = ai_network_run(rete, rete_input, rete_output);
    uint32_t fine = DWT->CYCCNT;
    out->rete_us = Hil_Microsecondi(fine - inizio_rete);
    out->totale_us = Hil_Microsecondi(fine - inizio);
    if (batches != 1)
    {
        rete_ultimo_errore = ai_network_get_error(rete);
        out->errore = 2U;
        return;
    }
    float action = ((ai_float *)rete_output[0].data)[0];
    if (!isfinite(action))
    {
        out->errore = 3U;
        return;
    }
    action = fmaxf(-1.0f, fminf(action, 1.0f));
    out->azione = action;
    out->richiesta_hz = Rete_AzioneInHz(action);
    rete_azione = action;
    frequenza_richiesta_hz = out->richiesta_hz;
    ++rete_inferenze;
}

/* Non ritorna. Il motore non viene mai comandato in questa modalita'. */
static void Hil_Esegui(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    Stepper_Stop();
    uint8_t prima = 1;
    HilRichiesta richiesta;
    while (1)
    {
        if (!Hil_Ricevi(&richiesta))
        {
            ++hil_scartati;
            continue;
        }
        if (prima || richiesta.seq != hil_risposta.seq || richiesta.comando == 2U)
            Hil_Elabora(&richiesta, &hil_risposta);
        prima = 0;
        Hil_Invia(&hil_risposta);
        ++hil_pacchetti;
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
}
#endif /* HIL_MODE */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  conteggio_precedente = 0U;

  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
      Error_Handler();
  }

  if (!Rete_Init())
  {
      controllo_errore = 1;
      Error_Handler();
  }

#if HIL_MODE
  /* Nessun homing e nessun movimento: misure e comandi passano dal PC. */
  Hil_Esegui();
#endif

  /* Acquisire il riferimento del pendolo PRIMA di muovere il carrello. */
  if (Inizializza_Theta() != HAL_OK)
  {
      controllo_errore = 7;
      Error_Handler();
  }

  /* Homing automatico come prima, indipendente dal pulsante. */
  Inizializza();
  if (inizializzazione_esito != HAL_OK)
  {
      Error_Handler();
  }

  /* Da qui B1 abilita/disabilita solo il controllo nel ciclo principale.
   * Le pressioni durante l'homing non vengono accodate.
   */
  Stepper_Stop();
  __HAL_GPIO_EXTI_CLEAR_IT(B1_Pin);
  HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
  /* Aggiornamento dal riferimento pre-homing, NON una nuova calibrazione. */
  Leggi_theta();
  ultimo_theta_rad = theta_rad;
  ultimo_carrello_steps = posizione_carrello_steps;
  ultimo_campione_ms = HAL_GetTick();
  pulsante_inizializzato = 1;
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if (!controllo_abilitato)
          Controllo_AccettaAvvio();
      Rete_Process();
      HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 124;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 6;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_Pin */
  GPIO_InitStruct.Pin = DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FC_sx_Pin Fc_dx_Pin */
  GPIO_InitStruct.Pin = FC_sx_Pin|Fc_dx_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Abilitazione rimandata al main, dopo Inizializza(). */
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != B1_Pin || !pulsante_inizializzato)
        return;
    /* Entrambi i fronti azzerano il tempo di rilascio: anche un breve
     * rimbalzo fra due SysTick non puo' riarmare il pulsante. */
    pulsante_rilascio_ms = 0;
    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != GPIO_PIN_RESET
        || !pulsante_pronto)
        return;
    pulsante_pronto = 0;
    if (controllo_abilitato || richiesta_avvio)
    {
        richiesta_avvio = 0;
        Controllo_Disabilita(); /* Stop PWM qui, senza attendere il main. */
    }
    else
        richiesta_avvio = 1; /* Avvio del controllo nel ciclo principale. */
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1
        || !stepper_running || !Stepper_MovimentoConsentito())
        return;

    step_count++;
    if (riferimento_carrello_valido)
        posizione_carrello_steps += (stepper_direction == DIR_FORWARD) ? -1 : 1;

    if (FinecorsaDirezione_Premuto(stepper_direction))
    {
        Stepper_Stop();
        arresto_finecorsa = 1;
        if (pulsante_inizializzato)
            Controllo_Fault(4);
        return;
    }

    if (pulsante_inizializzato
        && (posizione_carrello_steps <= posizione_carrello_min_steps
            || posizione_carrello_steps >= posizione_carrello_max_steps))
    {
        Controllo_Fault(4);
        return;
    }
    if (stop_al_fronte_step)
    {
        Stepper_Stop();
        return;
    }

    /* Il movimento continuo non ha un numero di passi limite. */
    if (step_target != 0U && step_count >= step_target)
    {
        Stepper_Stop();
        return;
    }
    Stepper_UpdateRamp();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  /* Fermare l'hardware PRIMA del blocco: il PWM continua senza CPU. */
  if (htim3.Instance == TIM3)
      Controllo_Disabilita();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
