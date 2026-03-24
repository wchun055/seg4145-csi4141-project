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
#include "lcd.h"
#include "encoder.h"
#include "motor.h"
#include "keypad.h"
#include "buzzer.h"
#include "controller.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TOUCH_EN_GPIO_Port GPIOB
#define TOUCH_EN_Pin       GPIO_PIN_5
#define TOUCH_INT_GPIO_Port GPIOB
#define TOUCH_INT_Pin      GPIO_PIN_6

#define TOUCH_AT42QT1070_ADDR         (0x1BU << 1)
#define TOUCH_AT42QT1070_CHIP_ID_REG  0x00U
#define TOUCH_AT42QT1070_KEY_REG      0x03U
#define TOUCH_AT42QT1070_CHIP_ID      0x2EU
#define TOUCH_MPR121_TOUCH_STATUS_REG 0x00U
#define TOUCH_MPR121_SOFTRESET_REG    0x80U
#define TOUCH_MPR121_ECR_REG          0x5EU
#define TOUCH_MPR121_DEBOUNCE_REG     0x5BU
#define TOUCH_MPR121_CONFIG1_REG      0x5CU
#define TOUCH_MPR121_CONFIG2_REG      0x5DU
#define TOUCH_MPR121_THRESHOLD_BASE   0x41U
#define TOUCH_MPR121_SOFTRESET_CMD    0x63U
#define TOUCH_MPR121_ECR_RUN_12ELE    0x8CU

#define TOUCH_MPR121_ADDR_0           (0x5AU << 1)
#define TOUCH_MPR121_ADDR_1           (0x5BU << 1)
#define TOUCH_MPR121_ADDR_2           (0x5CU << 1)
#define TOUCH_MPR121_ADDR_3           (0x5DU << 1)

#define MOTOR_TASK_PERIOD_MS          10U
#define ADC_TASK_PERIOD_MS            50U
#define BUTTON_DEBOUNCE_MS            150U
#define TOUCH_IRQ_DEBOUNCE_MS         20U
#define BUZZER_BEEP_MS                50U
#define CONTROL_EVENT_QUEUE_LENGTH    16U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim1;

/* Speed preset definitions for Lab 4 */
#define SPEED_PRESET_STOP       0U
#define SPEED_PRESET_LOW        3U      /* Low speed in RPM */
#define SPEED_PRESET_MEDIUM     6U      /* Medium speed in RPM */
#define SPEED_PRESET_HIGH       8U      /* High speed in RPM */

typedef struct {
    uint16_t targetSpeedRPM;
} SpeedCommand;  // Struct to hold speed control information (RPM-based)

typedef enum {
    CONTROL_EVT_BUTTON_PRESS = 0,
    CONTROL_EVT_TOUCH_IRQ,
    CONTROL_EVT_ADC_DUTY
} ControlEventType;

typedef struct {
    ControlEventType type;
    uint16_t value;
} ControlEvent;

typedef enum {
    TOUCH_PROTO_NONE = 0,
    TOUCH_PROTO_AT42QT1070,
    TOUCH_PROTO_MPR121
} TouchProtocol;


/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "MotorTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
  .name = "ControlTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for Task3 */
osThreadId_t Task3Handle;
const osThreadAttr_t Task3_attributes = {
  .name = "AdcTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for motorCmdQueue */
osMessageQueueId_t motorCmdQueueHandle;
const osMessageQueueAttr_t motorCmdQueue_attributes = {
  .name = "motorCmdQueue"
};
/* USER CODE BEGIN PV */

QueueHandle_t motorCmdQueue; // Queue to communicate motor speed commands
SpeedCommand speedCmd; // Struct for speed commands (target RPM)
QueueHandle_t controlEventQueue;
QueueHandle_t motorAppliedQueue;
volatile uint32_t touchIrqCount = 0;
static TouchProtocol touchProtocol = TOUCH_PROTO_NONE;
static uint16_t touchI2cAddr = 0U;
static bool touchEnableActiveHigh = true;
static uint16_t touchLastMask = 0xFFFFU;
static GPIO_PinState buttonPressedState = GPIO_PIN_SET;
volatile uint32_t buttonIrqCount = 0;
volatile uint32_t buttonQueuedEventCount = 0;
volatile uint8_t buttonLastLevel = 0;
volatile uint32_t buttonPollInjectedCount = 0;
static TimerHandle_t buzzerTimer = NULL;
static bool lcdReady = false;

/* Speed control state shared between tasks */
volatile uint16_t currentTargetRPM = 0U;
volatile uint16_t currentMeasuredRPM = 0U;

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
void StartTask1(void *argument);
void StartTask2(void *argument);
void StartTask3(void *argument);

/* USER CODE BEGIN PFP */
static void Touch_Enable(bool enable);
static void Touch_LogI2CScan(void);
static bool Touch_Mpr121Init(uint16_t addr);
static bool Touch_TryAutodetect(void);
static uint16_t Touch_ReadSpeedPreset(void);
static void Motor_ApplyCommand(const SpeedCommand *cmd);
static void ControlEvtSendFromISR(ControlEventType type, uint16_t value);
static const char* SpeedPresetToString(uint16_t preset);
static void LCD_DisplaySpeedInfo(uint16_t presetRPM, uint16_t measuredRPM);
static void BuzzerTimerCallback(TimerHandle_t timer);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}

static void LD1_Set(GPIO_PinState state)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, state); // LD1 (default)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, state); // LD1 alternate (solder bridge)
}

static void Touch_Enable(bool enable)
{
  GPIO_PinState asserted = touchEnableActiveHigh ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState deasserted = touchEnableActiveHigh ? GPIO_PIN_RESET : GPIO_PIN_SET;
  HAL_GPIO_WritePin(TOUCH_EN_GPIO_Port, TOUCH_EN_Pin, enable ? asserted : deasserted);
}

static bool Touch_Mpr121Init(uint16_t addr)
{
  /* MPR121 often powers up in STOP mode; initialize and enable electrodes. */
  if (HAL_I2C_Mem_Write(&hi2c2, addr, TOUCH_MPR121_SOFTRESET_REG, I2C_MEMADD_SIZE_8BIT,
                        (uint8_t[]){TOUCH_MPR121_SOFTRESET_CMD}, 1, 20) != HAL_OK) {
    return false;
  }

  if (HAL_I2C_Mem_Write(&hi2c2, addr, TOUCH_MPR121_ECR_REG, I2C_MEMADD_SIZE_8BIT,
                        (uint8_t[]){0x00U}, 1, 20) != HAL_OK) {
    return false;
  }

  for (uint8_t electrode = 0; electrode < 12; electrode++) {
    uint8_t regs[2] = { 12U, 6U }; /* touch threshold, release threshold */
    uint8_t reg = (uint8_t)(TOUCH_MPR121_THRESHOLD_BASE + electrode * 2U);
    if (HAL_I2C_Mem_Write(&hi2c2, addr, reg, I2C_MEMADD_SIZE_8BIT, regs, 2, 20) != HAL_OK) {
      return false;
    }
  }

  if (HAL_I2C_Mem_Write(&hi2c2, addr, TOUCH_MPR121_DEBOUNCE_REG, I2C_MEMADD_SIZE_8BIT,
                        (uint8_t[]){0x11U}, 1, 20) != HAL_OK) {
    return false;
  }
  if (HAL_I2C_Mem_Write(&hi2c2, addr, TOUCH_MPR121_CONFIG1_REG, I2C_MEMADD_SIZE_8BIT,
                        (uint8_t[]){0x10U}, 1, 20) != HAL_OK) {
    return false;
  }
  if (HAL_I2C_Mem_Write(&hi2c2, addr, TOUCH_MPR121_CONFIG2_REG, I2C_MEMADD_SIZE_8BIT,
                        (uint8_t[]){0x20U}, 1, 20) != HAL_OK) {
    return false;
  }
  if (HAL_I2C_Mem_Write(&hi2c2, addr, TOUCH_MPR121_ECR_REG, I2C_MEMADD_SIZE_8BIT,
                        (uint8_t[]){TOUCH_MPR121_ECR_RUN_12ELE}, 1, 20) != HAL_OK) {
    return false;
  }

  return true;
}

static void Touch_LogI2CScan(void)
{
  bool any = false;
  printf("[TOUCH] I2C2 scan start\r\n");
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(addr << 1), 2, 5) == HAL_OK) {
      printf("[TOUCH] found device @ 0x%02X\r\n", addr);
      any = true;
    }
  }
  if (!any) {
    printf("[TOUCH] no I2C devices found\r\n");
  }
}

static bool Touch_TryAutodetect(void)
{
  static const uint16_t mprAddrs[] = {
      TOUCH_MPR121_ADDR_0, TOUCH_MPR121_ADDR_1, TOUCH_MPR121_ADDR_2, TOUCH_MPR121_ADDR_3
  };
  uint8_t data[2] = {0};

  if (HAL_I2C_Mem_Read(&hi2c2, TOUCH_AT42QT1070_ADDR, TOUCH_AT42QT1070_CHIP_ID_REG,
                       I2C_MEMADD_SIZE_8BIT, data, 1, 20) == HAL_OK &&
      data[0] == TOUCH_AT42QT1070_CHIP_ID) {
    touchProtocol = TOUCH_PROTO_AT42QT1070;
    touchI2cAddr = TOUCH_AT42QT1070_ADDR;
    touchLastMask = 0xFFFFU;
    printf("[TOUCH] AT42QT1070 detected @ 0x%02X\r\n", TOUCH_AT42QT1070_ADDR >> 1);
    return true;
  }

  for (uint32_t i = 0; i < (sizeof(mprAddrs) / sizeof(mprAddrs[0])); i++) {
    uint16_t addr = mprAddrs[i];
    if (HAL_I2C_Mem_Read(&hi2c2, addr, TOUCH_MPR121_TOUCH_STATUS_REG,
                         I2C_MEMADD_SIZE_8BIT, data, 2, 20) == HAL_OK) {
      touchProtocol = TOUCH_PROTO_MPR121;
      touchI2cAddr = addr;
      touchLastMask = 0xFFFFU;
      printf("[TOUCH] MPR121-like controller detected @ 0x%02X\r\n", addr >> 1);
      if (Touch_Mpr121Init(addr)) {
        printf("[TOUCH] MPR121 initialized\r\n");
      } else {
        printf("[TOUCH] MPR121 init failed (using raw reads only)\r\n");
      }
      return true;
    }
  }

  touchProtocol = TOUCH_PROTO_NONE;
  touchI2cAddr = 0U;
  touchLastMask = 0xFFFFU;
  printf("[TOUCH] controller not detected (tried AT42 0x%02X, MPR121 0x5A-0x5D)\r\n",
/**
 * @brief Read speed preset from touch sensor
 * Key mapping (Lab 4):
 *   Key 0 -> Bit 0 -> STOP
 *   Key 1 -> Bit 1 -> LOW speed preset
 *   Key 2 -> Bit 2 -> MEDIUM speed preset
 *   Key 3 -> Bit 3 -> HIGH speed preset
 */
static uint16_t Touch_ReadSpeedPreset(void)
{
  uint8_t data[2] = {0};
  uint16_t mask = 0;

  if (touchProtocol == TOUCH_PROTO_NONE && !Touch_TryAutodetect()) {
    return SPEED_PRESET_STOP;  /* Default to stop if no touch controller */
  }

  if (touchProtocol == TOUCH_PROTO_AT42QT1070) {
    if (HAL_I2C_Mem_Read(&hi2c2, touchI2cAddr, TOUCH_AT42QT1070_KEY_REG,
                         I2C_MEMADD_SIZE_8BIT, data, 1, 20) != HAL_OK) {
      return SPEED_PRESET_STOP;
    }
    mask = data[0];
  } else if (touchProtocol == TOUCH_PROTO_MPR121) {
    if (HAL_I2C_Mem_Read(&hi2c2, touchI2cAddr, TOUCH_MPR121_TOUCH_STATUS_REG,
                         I2C_MEMADD_SIZE_8BIT, data, 2, 20) != HAL_OK) {
      return SPEED_PRESET_STOP;
    }
    mask = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
  } else {
    return SPEED_PRESET_STOP;
  }

  if (mask != touchLastMask) {
    printf("[TOUCH] raw mask=0x%04X proto=%d addr=0x%02X\r\n",
           mask, (int)touchProtocol, (unsigned int)(touchI2cAddr >> 1));
    touchLastMask = mask;
  }

  /* Deterministic priority: STOP(key0) > LOW(key1) > MEDIUM(key2) > HIGH(key3) */
  if (mask & (1U << 0)) {
    printf("[PRESET] STOP selected\r\n");
    return SPEED_PRESET_STOP;
  }
  if (mask & (1U << 1)) {
    printf("[PRESET] LOW speed selected (3 RPM)\r\n");
    return SPEED_PRESET_LOW;
  }
  if (mask & (1U << 2)) {
    printf("[PRESET] MEDIUM speed selected (6 RPM)\r\n");
    return SPEED_PRESET_MEDIUM;
  }
  if (mask & (1U << 3)) {
    printf("[PRESET] HIGH speed selected (8 RPM)\r\n");
    return SPEED_PRESET_HIGH;
  }

  return SPEED_PRESET_STOP;
}

static uint16_t ReadDutyPercent(void)
{
  uint32_t adcValue = 0;
  if (HAL_ADC_Start(&hadc1) == HAL_OK) {
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
      adcValue = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
  }
  return (uint16_t)((adcValue * 100U) / 4095U);
}

/**
 * @brief Convert speed preset to string
 */
static const char* SpeedPresetToString(uint16_t preset)
{
  switch(preset) {
    case SPEED_PRESET_STOP:    return "STOP";
    case SPEED_PRESET_LOW:     return "LOW";
    case SPEED_PRESET_MEDIUM:  return "MEDIUM";
    case SPEED_PRESET_HIGH:    return "HIGH";
    default:                   return "UNKNOWN";
  }
}

/**
 * @brief Display preset and measured RPM on LCD
 * Line 1: "Preset: SPEED"
 * Line 2: "RPM: RPM_NUMBER"
 */
static void LCD_DisplaySpeedInfo(uint16_t presetRPM, uint16_t measuredRPM)
{
  if (!lcdReady) return;
  static char line1[17];
  static char line2[17];
  
  /* Line 1: Show preset name */
  const char *presetName = SpeedPresetToString(presetRPM);
  snprintf(line1, sizeof(line1), "Preset: %-8s", presetName);
  (void)LCD_DisplayText(line1, FIRST_LINE);
  
  /* Line 2: Show measured RPM */
  snprintf(line2, sizeof(line2), "RPM: %d", measuredRPM);
  (void)LCD_DisplayText(line2, SECOND_LINE);
}

static void BuzzerTimerCallback(TimerHandle_t timer)
{
  (void)timer;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

/**
 * @brief Apply speed command using modular motor control
 * Uses Motor_SetCommand from motor.h for PWM-based speed control
 */
static void Motor_ApplyCommand(const SpeedCommand *cmd)
{
  if (cmd == NULL) return;

  if (cmd->targetSpeedRPM == SPEED_PRESET_STOP) {
    /* Stop the motor */
    Motor_SetCommand(MOTOR_STOP, 0);
    LD1_Set(GPIO_PIN_RESET);
    printf("[MOTOR] Speed: 0 RPM (STOP)\r\n");
  } else {
    /* Motor control loop will handle PWM adjustment via closed-loop controller
       For now, set initial direction (CW) - controller will adjust PWM */
    Motor_SetCommand(MOTOR_CW, 30);  /* Start at 30% PWM, controller will adjust */
    LD1_Set(GPIO_PIN_SET);
    printf("[MOTOR] Target speed: %u RPM\r\n", cmd->targetSpeedRPM);
  } else {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    LD1_Set(GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  }
}

static void ControlEvtSendFromISR(ControlEventType type, uint16_t value)
{
  BaseType_t woken = pdFALSE;
  ControlEvent evt = { .type = type, .value = value };
  if (controlEventQueue != NULL) {
    (void)xQueueSendFromISR(controlEventQueue, &evt, &woken);
  }
  portYIELD_FROM_ISR(woken);
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
  printf("[BOOT] Lab 4: DC Motor Speed Control Init Start\r\n");
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* Initialize motor to STOP */
  SpeedCommand initCmd = { .targetSpeedRPM = SPEED_PRESET_STOP };
  Motor_ApplyCommand(&initCmd);
  printf("[BOOT] Motor initialized to STOP\r\n");
  /* Detect blue-button polarity from idle level so press works on either board wiring. */
  buttonPressedState = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  printf("[BUTTON] PC13 pressed_state=%s\r\n", buttonPressedState == GPIO_PIN_SET ? "HIGH" : "LOW");
  /* Enable and initialize touch sensor */
  touchEnableActiveHigh = true;
  Touch_Enable(true);
  HAL_Delay(30);
  Touch_LogI2CScan();
  if (!Touch_TryAutodetect()) {
    printf("[TOUCH] retrying with active-low enable on PB5\r\n");
    Touch_Enable(false);
    touchEnableActiveHigh = false;
    Touch_Enable(true);
    HAL_Delay(30);
    Touch_LogI2CScan();
    (void)Touch_TryAutodetect();
  }
  printf("[TOUCH] enable polarity: %s\r\n", touchEnableActiveHigh ? "active-high" : "active-low");
  /* Initialize LCD display */
  LCD_Init();
  lcdReady = true;
  setLCD_RGB(32, 32, 32);
  LCD_DisplaySpeedInfo(SPEED_PRESET_STOP, 0);
  printf("[BOOT] LCD initialized\r\n");
  /* Initialize all modular components */
  Encoder_Init();
  Motor_Init();
  Controller_Init();
  Keypad_Init();
  Buzzer_Init();
  printf("[BOOT] Lab 4: All modules initialized\r\n");
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

  /* Create the queue(s) */
  /* creation of motorCmdQueue - updated for speed commands */
  motorCmdQueueHandle = osMessageQueueNew (1, sizeof(SpeedCommand), &motorCmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  motorCmdQueue = xQueueCreate(1, sizeof(SpeedCommand));
  controlEventQueue = xQueueCreate(CONTROL_EVENT_QUEUE_LENGTH, sizeof(ControlEvent));
  motorAppliedQueue = xQueueCreate(1, sizeof(SpeedCommand));
  if (motorCmdQueue == NULL || controlEventQueue == NULL || motorAppliedQueue == NULL) {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task1 */
  Task1Handle = osThreadNew(StartTask1, NULL, &Task1_attributes);

  /* creation of Task2 */
  Task2Handle = osThreadNew(StartTask2, NULL, &Task2_attributes);

  /* creation of Task3 */
  Task3Handle = osThreadNew(StartTask3, NULL, &Task3_attributes);

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

  /*Configure GPIO pins : PF0 PF1 (I2C2 SDA/SCL) */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI13_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI13_IRQn);
  HAL_NVIC_SetPriority(EXTI6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI6_IRQn);

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
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask1 */
/**
  * @brief  Function implementing the Task1 thread (Motor Control).
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask1 */
void StartTask1(void *argument) {
    (void)argument;
    uint16_t targetRPM = 0U;
    uint16_t measuredRPM = 0U;
    uint16_t computedPWM = 0U;
    TickType_t lastWake = xTaskGetTickCount();

    printf("[MotorControlTask] started (10ms period)\r\n");

    while (1) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTOR_CONTROL_TASK_PERIOD_MS));
        
        /* Get current target speed and measured speed */
        taskENTER_CRITICAL();
        targetRPM = currentTargetRPM;
        measuredRPM = currentMeasuredRPM;
        taskEXIT_CRITICAL();
        
        /* Apply motor control */
        if (targetRPM == 0U) {
            Motor_EmergencyStop();
            Controller_SetTarget(0U);
        } else {
            Controller_SetTarget(targetRPM);
            computedPWM = Controller_Update(measuredRPM);
            Motor_SetCommand(MOTOR_CW, computedPWM);
        }
    }
}


/* USER CODE BEGIN Header_StartTask2 */
/**
* @brief Function implementing the Task2 thread (Control/UI).
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask2 */
void StartTask2(void *argument) {
    (void)argument;
    KeypadPreset lastPreset = KEYPAD_PRESET_NONE;
    KeypadPreset currentPreset;
    uint16_t targetRPM = 0U;
    uint8_t beepCount = 0U;
    TickType_t lastWake = xTaskGetTickCount();

    printf("[UITask] started (50ms period)\r\n");

    while (1) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(UI_TASK_PERIOD_MS));
        
        /* Process keypad input */
        Keypad_Update();
        
        /* Check for new preset selection */
        if (Keypad_GetNewPreset(&currentPreset) && currentPreset != lastPreset) {
            targetRPM = Keypad_GetTargetRPM(currentPreset);
            beepCount = Keypad_GetBeepCount(currentPreset);
            
            /* Update target speed */
            taskENTER_CRITICAL();
            currentTargetRPM = targetRPM;
            taskEXIT_CRITICAL();
            
            /* Trigger buzzer pattern */
            if (beepCount > 0U) {
                Buzzer_BeepPattern(beepCount);
            }
            
            printf("[UI] Preset changed: %u RPM (beeps: %u)\r\n", targetRPM, beepCount);
            lastPreset = currentPreset;
        }
        
        /* Update buzzer state machine */
        Buzzer_Update();
        
        /* Update Encoder measurement */
        Encoder_Update();
        taskENTER_CRITICAL();
        currentMeasuredRPM = Encoder_GetSpeedRPM();
        taskEXIT_CRITICAL();
        
        /* Update LCD display */
        LCD_DisplaySpeedInfo(currentTargetRPM, currentMeasuredRPM);
    }
}


/* USER CODE BEGIN Header_StartTask3 */
/**
* @brief Function implementing the Task3 thread (Monitoring).
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask3 */
void StartTask3(void *argument) {
    (void)argument;
    TickType_t lastDiag = 0U;
    printf("[MonitorTask] started\r\n");
    while (1) {
        TickType_t now = xTaskGetTickCount();
        if ((now - lastDiag) >= pdMS_TO_TICKS(5000)) {
            printf("[MONITOR] Target: %u RPM, Measured: %u RPM\r\n", 
                   currentTargetRPM, currentMeasuredRPM);
            lastDiag = now;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
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
