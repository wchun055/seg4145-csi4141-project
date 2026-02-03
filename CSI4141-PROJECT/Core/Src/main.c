/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Lab 2 – FreeRTOS Motor Control (Open Loop)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "cmsis_os.h"

/* ===================== TYPES ===================== */

typedef enum {
  MOTOR_STOP = 0,
  MOTOR_CW,
  MOTOR_CCW
} MotorState_t;

/* ===================== GLOBALS ===================== */

volatile MotorState_t motorState = MOTOR_STOP;
volatile uint32_t motorDuty = 0;   // 0–100 %

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;  // PWM
TIM_HandleTypeDef htim2;  // Buzzer (optional)

/* ===================== RTOS ===================== */

osThreadId_t MotorTaskHandle;
osThreadId_t UITaskHandle;

const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .priority = osPriorityAboveNormal,
  .stack_size = 256
};

const osThreadAttr_t UITask_attributes = {
  .name = "UITask",
  .priority = osPriorityNormal,
  .stack_size = 256
};

/* ===================== PROTOTYPES ===================== */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);

void StartMotorTask(void *argument);
void StartUITask(void *argument);

/* ===================== MAIN ===================== */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();

  /* ===== INITIAL STATE (MANDATORY) ===== */
  motorState = MOTOR_STOP;
  motorDuty  = 0;

  BSP_LED_Init(LED_RED);
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);

  BSP_LED_On(LED_RED);
  BSP_LED_Off(LED_GREEN);
  BSP_LED_Off(LED_BLUE);

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET); // STB LOW

  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  osKernelInitialize();

  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);
  UITaskHandle    = osThreadNew(StartUITask, NULL, &UITask_attributes);

  osKernelStart();

  while (1) { }
}

/* ===================== MOTOR TASK ===================== */

void StartMotorTask(void *argument)
{
  for (;;)
  {
    switch (motorState)
    {
      case MOTOR_STOP:
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

        BSP_LED_On(LED_RED);
        BSP_LED_Off(LED_GREEN);
        BSP_LED_Off(LED_BLUE);
        break;

      case MOTOR_CW:
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                              (motorDuty * htim1.Init.Period) / 100);

        BSP_LED_On(LED_GREEN);
        BSP_LED_Off(LED_RED);
        BSP_LED_Off(LED_BLUE);
        break;

      case MOTOR_CCW:
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                              (motorDuty * htim1.Init.Period) / 100);

        BSP_LED_On(LED_BLUE);
        BSP_LED_Off(LED_RED);
        BSP_LED_Off(LED_GREEN);
        break;
    }

    osDelay(10);   // Motor control loop
  }
}

/* ===================== UI TASK ===================== */

void StartUITask(void *argument)
{
  uint8_t lastButton = 0;

  for (;;)
  {
    /* ---- Button handling ---- */
    uint8_t button = BSP_PB_GetState(BUTTON_USER);

    if (button && !lastButton)
    {
      if (motorState == MOTOR_STOP)
        motorState = MOTOR_CW;
      else if (motorState == MOTOR_CW)
        motorState = MOTOR_CCW;
      else
        motorState = MOTOR_STOP;
    }

    lastButton = button;

    /* ---- ADC → PWM ---- */
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 5);
    uint32_t adcVal = HAL_ADC_GetValue(&hadc1);

    motorDuty = (adcVal * 100) / 4095;

    if (motorState == MOTOR_STOP)
      motorDuty = 0;

    osDelay(25);   // UI rate
  }
}

/* ===================== GPIO ===================== */

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}


