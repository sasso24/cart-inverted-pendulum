/* HAL minimale: il codice sotto test viene estratto da main.c, non duplicato. */
#include <assert.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t CR1, PSC, EGR, ARR, CCR1, CNT; } Timer;
typedef struct { Timer *Instance; uint32_t Channel; } TIM_HandleTypeDef;
static Timer timer2, timer3;
static struct { uint32_t CFGR; } rcc;
#define RCC (&rcc)
#define RCC_CFGR_PPRE1 1U
#define TIM3 (&timer3)
#define TIM_CHANNEL_1 0U
#define TIM_HAL_DUMMY 1U
#define HAL_TIM_ACTIVE_CHANNEL_1 1U
#define TIM_CR1_UDIS 2U
#define TIM_EGR_UG 1U
#define TIM_FLAG_UPDATE 1U
#define TIM_FLAG_CC1 2U
#define SET_BIT(reg, bit) ((reg) |= (bit))
#define CLEAR_BIT(reg, bit) ((reg) &= ~(bit))
#define __HAL_TIM_GET_COUNTER(h) ((h)->Instance->CNT)
#define __HAL_TIM_SET_COUNTER(h, v) ((h)->Instance->CNT = (v))
#define __HAL_TIM_SET_AUTORELOAD(h, v) ((h)->Instance->ARR = (v))
#define __HAL_TIM_SET_COMPARE(h, c, v) ((void)(c), (h)->Instance->CCR1 = (v))
#define __HAL_TIM_CLEAR_FLAG(h, f) ((void)(h), (void)(f))
#define B1_GPIO_Port 0
#define B1_Pin 13
#define FC_sx_GPIO_Port 0
#define FC_sx_Pin 1
#define Fc_dx_GPIO_Port 0
#define Fc_dx_Pin 2
#define LD2_GPIO_Port 0
#define LD2_Pin 5
#define DIR_GPIO_Port 0
#define DIR_Pin 0
static TIM_HandleTypeDef htim2 = { &timer2, 0 };
static TIM_HandleTypeDef htim3 = { &timer3, HAL_TIM_ACTIVE_CHANNEL_1 };
static GPIO_PinState pins[16];
static uint32_t tick, primask;
static int pwm, starts, interrupt_during_start, interrupt_pending;
static int stop_on_prepare, stop_on_delay, homing_mode, stop_phase;
static void press(void);
static void release(void);
void Controllo_ButtonTick(void);
void HAL_GPIO_EXTI_Callback(uint16_t pin);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
static uint32_t __get_PRIMASK(void) { return primask; }
static void __disable_irq(void) { primask = 1; }
static void __set_PRIMASK(uint32_t value)
{
    primask = value;
    if (!primask && interrupt_pending) {
        interrupt_pending = 0;
        HAL_GPIO_EXTI_Callback(B1_Pin);
    }
}
static GPIO_PinState HAL_GPIO_ReadPin(int port, uint16_t pin)
{ (void)port; return pins[pin]; }
static void HAL_GPIO_WritePin(int port, uint16_t pin, GPIO_PinState state)
{ (void)port; pins[pin] = state; }
static uint32_t HAL_GetTick(void) { return tick; }
static uint32_t HAL_RCC_GetPCLK1Freq(void)
{
    if (stop_on_prepare) { stop_on_prepare = 0; press(); }
    return 42000000;
}
static HAL_StatusTypeDef HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *h, uint32_t c)
{
    (void)c;
    assert(primask); /* Il check e l'avvio non possono essere separati dall'ISR. */
    if (interrupt_during_start) {
        interrupt_during_start = 0;
        pins[B1_Pin] = GPIO_PIN_RESET;
        interrupt_pending = 1;
    }
    pwm = 1;
    h->Instance->CR1 |= 1U;
    ++starts;
    return HAL_OK;
}
static HAL_StatusTypeDef HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *h, uint32_t c)
{ (void)c; pwm = 0; h->Instance->CR1 &= ~1U; return HAL_OK; }
static void HAL_Delay(uint32_t ms);
#include "application.inc"

static void press(void)
{
    pins[B1_Pin] = GPIO_PIN_RESET;
    HAL_GPIO_EXTI_Callback(B1_Pin);
}
static void release(void)
{
    pins[B1_Pin] = GPIO_PIN_SET;
    HAL_GPIO_EXTI_Callback(B1_Pin);
    for (unsigned i = 0; i <= BUTTON_RELEASE_MS; ++i) {
        ++tick;
        Controllo_ButtonTick();
    }
}
static void HAL_Delay(uint32_t ms)
{
    assert(!primask); /* Nessuna attesa nelle sezioni critiche. */
    tick += ms;
    Controllo_ButtonTick();
    if (stop_on_delay) { stop_on_delay = 0; press(); }
    if (homing_mode && pwm) {
        if (starts == stop_phase) {
            press();
            release();
            press();
            /* Prima del main B1 non arresta e non accoda richieste. */
            assert(pwm && !controllo_abilitato && !richiesta_avvio);
        }
        {
            if (starts == 1) {
                pins[Fc_dx_Pin] = GPIO_PIN_SET;
            } else if (starts == 2) {
                pins[Fc_dx_Pin] = GPIO_PIN_RESET;
                pins[FC_sx_Pin] = GPIO_PIN_SET;
                step_count = 99;
            } else {
                pins[FC_sx_Pin] = GPIO_PIN_RESET;
                step_count = step_target - 1;
            }
            HAL_TIM_PWM_PulseFinishedCallback(&htim3);
        }
    }
}
static void reset(void)
{
    memset(pins, 0, sizeof pins);
    memset(&timer3, 0, sizeof timer3);
    timer3.PSC = 83;
    tick = primask = 0;
    pwm = starts = interrupt_during_start = interrupt_pending = 0;
    stop_on_prepare = stop_on_delay = homing_mode = stop_phase = 0;
    controllo_abilitato = richiesta_avvio = pulsante_pronto = 0;
    pulsante_inizializzato = 1;
    pulsante_rilascio_ms = 0;
    riferimento_carrello_valido = carrello_inizializzato = 0;
    Stepper_Stop();
}
static void enable(void)
{
    release();
    press();
    assert(richiesta_avvio && !controllo_abilitato && !pwm);
    Controllo_AccettaAvvio();
    assert(controllo_abilitato && pins[LD2_Pin]);
    release();
}
static void assert_stopped(void)
{
    assert(!pwm && !stepper_running && !controllo_abilitato);
    assert(!pins[LD2_Pin]);
}
int main(void)
{
    reset();
    /* Entrata nel ciclo principale con tasto premuto: controllo fermo. */
    for (int i = 0; i < 100; ++i) { press(); Controllo_ButtonTick(); }
    assert(!richiesta_avvio && !controllo_abilitato);
    Stepper_MoveSteps(10);
    Stepper_set_velocity(DIR_FORWARD, 5000);
    assert(!starts);

    enable();
    Stepper_MoveSteps(100);
    assert(pwm);
    riferimento_carrello_valido = carrello_inizializzato = 1;
    press();
    assert_stopped();
    assert(riferimento_carrello_valido && carrello_inizializzato);
    for (int i = 0; i < 20; ++i) {
        pins[B1_Pin] = GPIO_PIN_SET; HAL_GPIO_EXTI_Callback(B1_Pin);
        Controllo_ButtonTick(); press();
    }
    assert(!richiesta_avvio);
    Stepper_MoveSteps(100); Stepper_set_velocity(DIR_BACKWARD, 5000);
    assert(starts == 1);
    release(); press();
    assert(richiesta_avvio && !controllo_abilitato);
    Stepper_MoveSteps(100); /* Il vecchio contesto non puo' ripartire. */
    assert(starts == 1);
    release(); press(); /* Si puo' annullare anche un avvio in coda. */
    assert(!richiesta_avvio);

    reset(); enable(); stop_on_prepare = 1;
    Stepper_MoveSteps(100); /* Stop fra preparazione e avvio. */
    assert_stopped(); assert(!starts);

    reset(); enable(); interrupt_during_start = 1;
    Stepper_MoveSteps(100); /* IRQ pendente durante la sezione critica. */
    assert_stopped(); assert(starts == 1);

    reset(); enable(); stop_on_delay = 1;
    Stepper_set_velocity(DIR_FORWARD, 5000);
    assert_stopped(); assert(!starts);

    reset(); enable(); tick = UINT32_MAX - 5;
    assert(Controllo_Attendi(10)); /* Wrap del contatore millisecondi. */
    stop_on_delay = 1;
    assert(!Controllo_Attendi(2000));
    assert_stopped();

    /* L'homing parte senza B1 e ignora le pressioni in ciascuna fase. */
    for (int phase = 1; phase <= 3; ++phase) {
        reset(); pulsante_inizializzato = 0;
        homing_mode = 1; stop_phase = phase;
        Inizializza();
        assert(carrello_inizializzato && inizializzazione_esito == HAL_OK);
        assert(starts == 3 && !pwm && corsa_totale_steps == 100);
        assert(!controllo_abilitato && !richiesta_avvio);
        pulsante_inizializzato = 1;
        homing_mode = 0;
        enable();
        assert(starts == 3); /* Abilitare il main non ripete l'homing. */
        Stepper_MoveSteps(100);
        assert(starts == 4 && pwm);
        press();
        assert_stopped();
        enable();
        assert(starts == 4 && carrello_inizializzato);
    }
    puts("OK: homing autonomo, B1 solo nel main, toggle senza nuovo homing, debounce, stop, avvii concorrenti e attese");
    return 0;
}
