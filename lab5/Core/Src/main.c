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
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "keypad.h"
#include "motor.h"
#include "display.h"
#include "buzzer.h"
#include "encoder.h"
#include "adc.h"
#include "safety.h"
#include "control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PIR_GPIO_Port      GPIOB
#define PIR_Pin            GPIO_PIN_9

#define ENCODER_A_GPIO_Port GPIOB
#define ENCODER_A_Pin       GPIO_PIN_0
#define ENCODER_B_GPIO_Port GPIOB
#define ENCODER_B_Pin       GPIO_PIN_1

#define MOTOR_TASK_PERIOD_MS          10U
#define ADC_TASK_PERIOD_MS            50U
#define BUTTON_DEBOUNCE_MS            150U
#define TOUCH_IRQ_DEBOUNCE_MS         20U
#define BUZZER_BEEP_MS                50U
#define CONTROL_EVENT_QUEUE_LENGTH    16U
#define SAFETY_TASK_PERIOD_MS         5U
#define ENCODER_TASK_PERIOD_MS        20U
#define BUZZER_ALARM_PERIOD_MS        500U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim1;

typedef enum {
    CONTROL_EVT_BUTTON_PRESS = 0,
    CONTROL_EVT_TOUCH_IRQ,
    CONTROL_EVT_ADC_DUTY,
    CONTROL_EVT_PIR_TRIGGER,
    CONTROL_EVT_ENCODER_UPDATE,
    CONTROL_EVT_REARM_REQUEST
} ControlEventType;

typedef struct {
    ControlEventType type;
    uint16_t value;
} ControlEvent;


/* Definitions for SafetyTask */
osThreadId_t SafetyTaskHandle;
const osThreadAttr_t SafetyTask_attributes = {
  .name = "SafetyTask",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 128 * 4
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for BuzzerTask */
osThreadId_t BuzzerTaskHandle;
const osThreadAttr_t BuzzerTask_attributes = {
  .name = "BuzzerTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for AdcTask */
osThreadId_t AdcTaskHandle;
const osThreadAttr_t AdcTask_attributes = {
  .name = "AdcTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for motorCmdQueue */
osMessageQueueId_t motorCmdQueueHandle;
const osMessageQueueAttr_t motorCmdQueue_attributes = {
  .name = "motorCmdQueue"
};
/* Definitions for buzzerQueue */
osMessageQueueId_t buzzerQueueHandle;
const osMessageQueueAttr_t buzzerQueue_attributes = {
  .name = "buzzerQueue"
};
/* USER CODE BEGIN PV */

QueueHandle_t motorCmdQueue; 
QueueHandle_t controlEventQueue;
QueueHandle_t motorAppliedQueue;
QueueHandle_t buzzerQueue;
volatile uint32_t touchIrqCount = 0;
volatile int32_t encoderCount = 0;
volatile uint32_t encoderIrqCount = 0;
volatile uint32_t pirIrqCount = 0;
bool emergencyActive = false;
uint16_t measuredRPM = 0;
float kp = 1.0f;  
int16_t error_integral = 0;
static GPIO_PinState buttonPressedState = GPIO_PIN_SET;
volatile uint32_t buttonIrqCount = 0;
volatile uint32_t buttonQueuedEventCount = 0;
volatile uint8_t buttonLastLevel = 0;
volatile uint32_t buttonPollInjectedCount = 0;
static bool lcdReady = false;


uint32_t countTask1 = 0;
uint32_t countTask2 = 0;
uint32_t countTask3 = 0;
uint32_t countInWhile = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
void StartSafetyTask(void *argument);
void StartMotorTask(void *argument);
void StartControlTask(void *argument);
void StartEncoderTask(void *argument);
void StartBuzzerTask(void *argument);
void StartAdcTask(void *argument);

/* USER CODE BEGIN PFP */
static void ControlEvtSendFromISR(ControlEventType type, uint16_t value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
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
  MX_ICACHE_Init();
  MX_LPUART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  printf("From main: Welcome to STM32 world !\r\n");
  printf("[BOOT] Lab3 init start\r\n");
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  MotorCommand initCmd = { .state = STOP, .duty_percent = 0U };
  Motor_ApplyCommand(&initCmd);
  /* Detect blue-button polarity from idle level so press works on either board wiring. */
  buttonPressedState = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  printf("[BUTTON] PC13 pressed_state=%s\r\n", buttonPressedState == GPIO_PIN_SET ? "HIGH" : "LOW");
  Keypad_Init();
  Display_Init();
  Buzzer_Init();
  Encoder_Init();
  Safety_Init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  buzzerTimer = xTimerCreate("buz",
                             pdMS_TO_TICKS(BUZZER_BEEP_MS),
                             pdFALSE,
                             NULL,
                             BuzzerTimerCallback);
  if (buzzerTimer == NULL) {
    Error_Handler();
  }
  /* USER CODE END RTOS_TIMERS */
                             BuzzerTimerCallback);
  if (buzzerTimer == NULL) {
    Error_Handler();
  }
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of motorCmdQueue */
  motorCmdQueueHandle = osMessageQueueNew (1, sizeof(MotorCommand), &motorCmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  motorCmdQueue = xQueueCreate(1, sizeof(MotorCommand));
  controlEventQueue = xQueueCreate(CONTROL_EVENT_QUEUE_LENGTH, sizeof(ControlEvent));
  motorAppliedQueue = xQueueCreate(1, sizeof(MotorCommand));
  buzzerQueue = xQueueCreate(5, sizeof(BuzzerCommand));
  if (motorCmdQueue == NULL || controlEventQueue == NULL || motorAppliedQueue == NULL || buzzerQueue == NULL) {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SafetyTask */
  SafetyTaskHandle = osThreadNew(StartSafetyTask, NULL, &SafetyTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(StartEncoderTask, NULL, &EncoderTask_attributes);

  /* creation of BuzzerTask */
  BuzzerTaskHandle = osThreadNew(StartBuzzerTask, NULL, &BuzzerTask_attributes);

  /* creation of AdcTask */
  AdcTaskHandle = osThreadNew(StartAdcTask, NULL, &AdcTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 55;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10C0ECFF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache (default 2-ways set associative cache)
  */
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TOUCH_EN_GPIO_Port, TOUCH_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 (Blue Button EXTI) */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 (LD1 alternate, depending on solder bridge) */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 (Red LED) */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 (Blue LED) */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 (Touch Enable) */
  GPIO_InitStruct.Pin = TOUCH_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TOUCH_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 (Touch Interrupt EXTI) */
  GPIO_InitStruct.Pin = TOUCH_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TOUCH_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 (PIR Input EXTI) */
  GPIO_InitStruct.Pin = PIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 (Encoder A/B EXTI) */
  GPIO_InitStruct.Pin = ENCODER_A_Pin | ENCODER_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENCODER_A_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI13_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI13_IRQn);
  HAL_NVIC_SetPriority(EXTI6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI6_IRQn);
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13) {
    GPIO_PinState level = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
    buttonIrqCount++;
    buttonLastLevel = (uint8_t)((level == GPIO_PIN_SET) ? 1U : 0U);
    buttonQueuedEventCount++;
    ControlEvtSendFromISR(CONTROL_EVT_BUTTON_PRESS, (uint16_t)buttonLastLevel);
  } else if (GPIO_Pin == TOUCH_INT_Pin) {
    touchIrqCount++;
    ControlEvtSendFromISR(CONTROL_EVT_TOUCH_IRQ, 0U);
  } else if (GPIO_Pin == PIR_Pin) {
    pirIrqCount++;
    ControlEvtSendFromISR(CONTROL_EVT_PIR_TRIGGER, 0U);
  } else if (GPIO_Pin == ENCODER_A_Pin || GPIO_Pin == ENCODER_B_Pin) {
    encoderIrqCount++;
    // Simple encoder: assume CW increases count
    GPIO_PinState a = HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port, ENCODER_A_Pin);
    GPIO_PinState b = HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin);
    if (GPIO_Pin == ENCODER_A_Pin) {
      if (a == b) {
        encoderCount++;
      } else {
        encoderCount--;
      }
    } else {
      if (a != b) {
        encoderCount++;
      } else {
        encoderCount--;
      }
    }
    ControlEvtSendFromISR(CONTROL_EVT_ENCODER_UPDATE, 0U);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSafetyTask */
/**
  * @brief  Function implementing the SafetyTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSafetyTask */
void StartSafetyTask(void *argument) {
    (void)argument;
    ControlEvent evt;
    printf("[SafetyTask] started\r\n");

    while (1) {
        if (xQueueReceive(controlEventQueue, &evt, portMAX_DELAY) == pdPASS) {
            Safety_ProcessEvent(&evt);
        }
    }
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
  * @brief  Function implementing the MotorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument) {
    (void)argument;
    MotorCommand active = { .state = STOP, .duty_percent = 0U };
    MotorCommand pending;
    TickType_t lastWake = xTaskGetTickCount();
    uint16_t currentDuty = 0;

    printf("[MotorTask] started\r\n");
    Motor_ApplyCommand(&active);
    (void)xQueueOverwrite(motorAppliedQueue, &active);

    while (1) {
        while (xQueueReceive(motorCmdQueue, &pending, 0) == pdPASS) {
            active = pending;
        }

        if (active.state == CW || active.state == CCW) {
            uint16_t targetRPM = active.duty_percent * 3;  // 0-100 -> 0-300 RPM
            int16_t error = (int16_t)targetRPM - (int16_t)measuredRPM;
            error_integral += error;
            int16_t control = (int16_t)(kp * error + 0.1f * error_integral);
            currentDuty = (uint16_t)control;
            if (currentDuty > 100) currentDuty = 100;
            if (currentDuty < 0) currentDuty = 0;
            active.duty_percent = currentDuty;
        } else {
            currentDuty = 0;
            active.duty_percent = 0;
            error_integral = 0;
        }

        Motor_ApplyCommand(&active);
        (void)xQueueOverwrite(motorAppliedQueue, &active);
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTOR_TASK_PERIOD_MS));
    }
}


/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument) {
    (void)argument;
    ControlEvent evt;
    MotorCommand cmd = { .state = STOP, .duty_percent = 0U };
    MotorCommand applied;
    MotorState state = STOP;
    uint16_t duty = ADC_ReadDutyPercent();
    TickType_t lastButtonTick = 0;
    TickType_t lastTouchTick = 0;
    TickType_t lastDiag = 0;
    TickType_t lastTouchPoll = 0;
    TickType_t lastButtonPoll = 0;
    bool lastTouchPollValid = false;
    MotorState lastTouchPollState = STOP;
    GPIO_PinState lastButtonSample = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

    printf("[ControlTask] started\r\n");
    (void)xQueueOverwrite(motorCmdQueue, &cmd);
    Display_UpdateDisplay(STOP, 0, 0);

    while (1) {
        if (xQueueReceive(controlEventQueue, &evt, pdMS_TO_TICKS(20)) != pdPASS) {
            TickType_t now = xTaskGetTickCount();
            if ((now - lastButtonPoll) >= pdMS_TO_TICKS(20)) {
                GPIO_PinState sample = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
                if (sample != lastButtonSample) {
                    printf("[BUTTON] poll edge level=%s (pressed=%s) irqCount=%lu\r\n",
                           (sample == GPIO_PIN_SET) ? "HIGH" : "LOW",
                           (buttonPressedState == GPIO_PIN_SET) ? "HIGH" : "LOW",
                           (unsigned long)buttonIrqCount);
                    /* Fallback only if EXTI path is dead on this board/runtime. */
                    if (buttonIrqCount == 0U && sample == buttonPressedState) {
                        ControlEvent bevt = { .type = CONTROL_EVT_BUTTON_PRESS,
                                              .value = (sample == GPIO_PIN_SET) ? 1U : 0U };
                        if (xQueueSend(controlEventQueue, &bevt, 0) == pdPASS) {
                            buttonPollInjectedCount++;
                            printf("[BUTTON] poll fallback injected press event\r\n");
                        }
                    }
                    lastButtonSample = sample;
                }
                lastButtonPoll = now;
            }
            if (true &&
                (now - lastTouchPoll) >= pdMS_TO_TICKS(80)) {
                MotorState polledState;
                bool valid = Keypad_ReadCommandState(&polledState);
                if (valid && (!lastTouchPollValid || polledState != lastTouchPollState)) {
                    state = polledState;
                    cmd.state = state;
                    cmd.duty_percent = (state == STOP) ? 0U : duty;
                    (void)xQueueOverwrite(motorCmdQueue, &cmd);
                    uint16_t targetRPM = cmd.duty_percent * 3;
                    Display_UpdateDisplay(state, targetRPM, measuredRPM);
                    Buzzer_Beep();
                    printf("[TOUCH] Poll command -> %s\r\n", MotorStateStr(state));
                }
                lastTouchPollValid = valid;
                if (valid) {
                    lastTouchPollState = polledState;
                }
                lastTouchPoll = now;
            }
            
            continue;
        }

        TickType_t now = xTaskGetTickCount();
        bool publish = false;
        bool stateChanged = false;
        const char *line2 = NULL;

        if (evt.type == CONTROL_EVT_BUTTON_PRESS) {
            GPIO_PinState evtLevel = (evt.value != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            printf("[BUTTON] IRQ edge level=%s (pressed=%s)\r\n",
                   (evtLevel == GPIO_PIN_SET) ? "HIGH" : "LOW",
                   (buttonPressedState == GPIO_PIN_SET) ? "HIGH" : "LOW");
            if (evtLevel != buttonPressedState) {
                printf("[BUTTON] ignored release/non-press edge\r\n");
                continue;
            }
            if ((now - lastButtonTick) >= pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
                lastButtonTick = now;
                state = EMERGENCY;
                stateChanged = true;
                publish = true;
                line2 = "Button";
                printf("[ControlTask] Button -> %s\r\n", MotorStateStr(state));
            } else {
                printf("[BUTTON] ignored by debounce\r\n");
            }
        } else if (evt.type == CONTROL_EVT_TOUCH_IRQ) {
            MotorState touchState;
            if ((now - lastTouchTick) >= pdMS_TO_TICKS(TOUCH_IRQ_DEBOUNCE_MS)) {
                lastTouchTick = now;
                printf("[TOUCH] IRQ received (count=%lu)\r\n", (unsigned long)touchIrqCount);
                if (Keypad_ReadCommandState(&touchState)) {
                    if (emergencyActive) {
                        if (touchState == EMERGENCY) {  // Re-arm
                            ControlEvent rearmEvt = { .type = CONTROL_EVT_REARM_REQUEST, .value = 0 };
                            (void)xQueueSend(controlEventQueue, &rearmEvt, 0);
                        } else {
                            printf("[TOUCH] Command ignored in EMERGENCY\r\n");
                        }
                    } else {
                        state = touchState;
                        stateChanged = true;
                        publish = true;
                        line2 = "Touch";
                        printf("[TOUCH] Command -> %s\r\n", MotorStateStr(state));
                    }
                } else {
                    printf("[TOUCH] IRQ but no valid key / read failed\r\n");
                }
            }
        } else if (evt.type == CONTROL_EVT_ADC_DUTY) {
            if (evt.value <= 100U && evt.value != duty) {
                duty = evt.value;
                if (state != STOP && !emergencyActive) {
                    publish = true;
                }
            }
        } else if (evt.type == CONTROL_EVT_REARM_REQUEST) {
            if (emergencyActive) {
                emergencyActive = false;
                state = STOP;
                stateChanged = true;
                publish = true;
                line2 = "Re-arm";
                printf("[ControlTask] Re-arm -> %s\r\n", MotorStateStr(state));
            }
        }

        if (publish && !emergencyActive) {
            cmd.state = state;
            cmd.duty_percent = (state == STOP) ? 0U : duty;
            (void)xQueueOverwrite(motorCmdQueue, &cmd);
        }

        if (stateChanged) {
            bool matchedApplied = false;
            TickType_t waitUntil = xTaskGetTickCount() + pdMS_TO_TICKS(MOTOR_TASK_PERIOD_MS * 4U);

            while (xTaskGetTickCount() < waitUntil) {
                TickType_t nowWait = xTaskGetTickCount();
                TickType_t waitTicks = (waitUntil > nowWait) ? (waitUntil - nowWait) : 0;
                if (xQueueReceive(motorAppliedQueue, &applied, waitTicks) != pdPASS) {
                    break;
                }
                if (applied.state == state) {
                    matchedApplied = true;
                    break;
                }
                printf("[ControlTask] stale applied=%s expected=%s\r\n",
                       MotorStateStr(applied.state), MotorStateStr(state));
            }

            MotorState displayState = matchedApplied ? applied.state : state;
            uint16_t targetRPM = cmd.duty_percent * 3; // 0-100 -> 0-300
            Display_UpdateDisplay(displayState, targetRPM, measuredRPM);
            Buzzer_Beep();
        }
    }
}


/* USER CODE BEGIN Header_StartEncoderTask */
/**
* @brief Function implementing the EncoderTask thread.
* @param argument: 
* @retval None
*/
/* USER CODE END Header_StartEncoderTask */
void StartEncoderTask(void *argument) {
    (void)argument;
    TickType_t lastWake = xTaskGetTickCount();
    int32_t lastCount = 0;
    printf("[EncoderTask] started\r\n");

    while (1) {
        int32_t currentCount = encoderCount;
        int32_t delta = currentCount - lastCount;
        // Simple RPM calculation: pulses per period * (1000 / period_ms) / pulses_per_rev
        uint16_t rpm = (uint16_t)((abs(delta) * 1000U / ENCODER_TASK_PERIOD_MS) / ENCODER_PULSES_PER_REV);
        measuredRPM = rpm;
        lastCount = currentCount;
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(ENCODER_TASK_PERIOD_MS));
    }
}

/* USER CODE BEGIN Header_StartBuzzerTask */
/**
* @brief Function implementing the BuzzerTask thread.
* @param argument: 
* @retval None
*/
/* USER CODE END Header_StartBuzzerTask */
void StartBuzzerTask(void *argument) {
    (void)argument;
    BuzzerCommand cmd;
    printf("[BuzzerTask] started\r\n");

    while (1) {
        if (xQueueReceive(buzzerQueue, &cmd, portMAX_DELAY) == pdPASS) {
            Buzzer_ProcessCommand(cmd);
        }
    }
}

/* USER CODE BEGIN Header_StartAdcTask */
/**
* @brief Function implementing the AdcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAdcTask */
void StartAdcTask(void *argument) {
    (void)argument;
    TickType_t lastWake = xTaskGetTickCount();
    uint16_t lastDuty = 0xFFFFU;
    printf("[AdcTask] started\r\n");
    while (1) {
        uint16_t duty = ADC_ReadDutyPercent();
        if (lastDuty == 0xFFFFU || duty != lastDuty) {
            ControlEvent evt = { .type = CONTROL_EVT_ADC_DUTY, .value = duty };
            (void)xQueueSend(controlEventQueue, &evt, 0);
            lastDuty = duty;
        }
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(ADC_TASK_PERIOD_MS));
    }
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
