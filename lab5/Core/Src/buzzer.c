/**
  * @file buzzer.c
  * @brief Buzzer module implementation
  */

#include "buzzer.h"
#include "main.h"

#define BUZZER_BEEP_MS 50U
#define BUZZER_ALARM_PERIOD_MS 500U

static TimerHandle_t buzzerTimer = NULL;

static void BuzzerTimerCallback(TimerHandle_t timer)
{
  (void)timer;
  // Toggle buzzer for alarm
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
}

void Buzzer_Init(void)
{
  buzzerTimer = xTimerCreate("buz",
                             pdMS_TO_TICKS(BUZZER_BEEP_MS),
                             pdFALSE,
                             NULL,
                             BuzzerTimerCallback);
  if (buzzerTimer == NULL) {
    Error_Handler();
  }
}

static void Buzzer_Beep(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  if (buzzerTimer != NULL) {
    (void)xTimerReset(buzzerTimer, 0);
  }
}

void Buzzer_ProcessCommand(BuzzerCommand cmd)
{
  if (cmd == BUZZER_CMD_BEEP) {
    Buzzer_Beep();
  } else if (cmd == BUZZER_CMD_ALARM_START) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    if (buzzerTimer != NULL) {
      xTimerChangePeriod(buzzerTimer, pdMS_TO_TICKS(BUZZER_ALARM_PERIOD_MS), 0);
      xTimerStart(buzzerTimer, 0);
    }
  } else if (cmd == BUZZER_CMD_ALARM_STOP) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    if (buzzerTimer != NULL) {
      xTimerStop(buzzerTimer, 0);
    }
  }
}