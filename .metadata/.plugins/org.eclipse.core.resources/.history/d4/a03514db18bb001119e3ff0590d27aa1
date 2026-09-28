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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEP_CHANNEL TIM_CHANNEL_1
#define MIN_FREQUENCY 500.0f
#define MAX_FREQUENCY 8000.0f
#define STEPPER_ACCELERATION_HZ_PER_S 10000.0f

// tutte i punti di rif sono da dietro
#define DIR_FORWARD  GPIO_PIN_SET   //SInistra (da dietro)
#define DIR_BACKWARD GPIO_PIN_RESET

#define HOMING_VELOCITY_HZ  1000.0f
#define HOMING_TIMEOUT_MS  120000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

    SET_BIT(htim3.Instance->CR1, TIM_CR1_UDIS);
    __HAL_TIM_SET_AUTORELOAD(&htim3, ticks - 1U);
    __HAL_TIM_SET_COMPARE(&htim3, STEP_CHANNEL, ticks / 2U);
    CLEAR_BIT(htim3.Instance->CR1, TIM_CR1_UDIS);
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

    if (stepper_direction == DIR_FORWARD && FinecorsaSinistro_Premuto())
    {
        arresto_finecorsa = 1;
        return;
    }
    if (steps == 0U)
        return;

    Stepper_PrepareRamp();
    stepper_running = 1;
    if (HAL_TIM_PWM_Start_IT(&htim3, STEP_CHANNEL) != HAL_OK)
        Stepper_Stop();
}

void Stepper_Move(GPIO_PinState direction, uint32_t steps)
{
    Stepper_Stop();
    Stepper_SetDirection(direction);
    HAL_Delay(1);
    Stepper_MoveSteps(steps);

    while (stepper_running)
    {
        if (direction == DIR_FORWARD && FinecorsaSinistro_Premuto())
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

    if (!isfinite(velocity) || velocity <= 0.0f)
        return;

    stepper_target_frequency = Stepper_ClampFrequency(velocity);
    Stepper_SetDirection(direction);
    HAL_Delay(1);
    if (direction == DIR_FORWARD && FinecorsaSinistro_Premuto())
    {
        arresto_finecorsa = 1;
        return;
    }

    Stepper_PrepareRamp();
    stepper_running = 1;
    if (HAL_TIM_PWM_Start_IT(&htim3, STEP_CHANNEL) != HAL_OK)
        Stepper_Stop();
}

/* Partire dall'estremo destro, posizionato manualmente: non ha un sensore.
 * Misura la corsa lentamente fino a sinistra e torna al centro a 8 kHz.
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

    /* Se si parte gia' a sinistra non si puo' misurare la corsa. */
    if (FinecorsaSinistro_Premuto())
    {
        inizializzazione_esito = HAL_ERROR;
        return;
    }

    Stepper_set_velocity(DIR_FORWARD, HOMING_VELOCITY_HZ);
    uint32_t inizio = HAL_GetTick();
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

    if (step_count != passi_centro)
    {
        riferimento_carrello_valido = 0;
        inizializzazione_esito = HAL_ERROR;
        return;
    }
    carrello_inizializzato = 1;
    inizializzazione_esito = HAL_OK;
}

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
  /* USER CODE BEGIN 2 */

  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  conteggio_precedente = 0U;

  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
      Error_Handler();
  }

  Inizializza();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Leggi_theta();



	 /*

	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	  Stepper_Move(DIR_FORWARD, 2000);
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	  HAL_Delay(1000);



	  Stepper_Move(DIR_BACKWARD, 2000);


	  HAL_Delay(2000);*/
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_Pin */
  GPIO_InitStruct.Pin = DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FC_sx_Pin */
  GPIO_InitStruct.Pin = FC_sx_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FC_sx_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1
        || !stepper_running)
        return;

    step_count++;
    if (riferimento_carrello_valido)
        posizione_carrello_steps += (stepper_direction == DIR_FORWARD) ? -1 : 1;

    if (stepper_direction == DIR_FORWARD && FinecorsaSinistro_Premuto())
    {
        Stepper_Stop();
        arresto_finecorsa = 1;
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
