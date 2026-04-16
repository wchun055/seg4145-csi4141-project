/**
  * @file safety.c
  * @brief Safety module implementation
  */

#include "safety.h"
#include "motor.h"
#include "buzzer.h"
#include "display.h"
#include "main.h"

extern QueueHandle_t controlEventQueue;
extern QueueHandle_t buzzerQueue;
extern QueueHandle_t motorCmdQueue;
extern bool emergencyActive;

void Safety_Init(void)
{
  emergencyActive = false;
}

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

void Safety_Init(void)
{
}

void Safety_ProcessEvent(void)
{
  ControlEvent evt;
  if (xQueueReceive(controlEventQueue, &evt, portMAX_DELAY) == pdPASS) {
    if (evt.type == CONTROL_EVT_PIR_TRIGGER) {
      printf("[Safety] PIR triggered, entering EMERGENCY\r\n");
      emergencyActive = true;
      MotorCommand emergCmd = { .state = EMERGENCY, .duty_percent = 0U };
      (void)xQueueOverwrite(motorCmdQueue, &emergCmd);
      BuzzerCommand buzCmd = BUZZER_CMD_ALARM_START;
      (void)xQueueSend(buzzerQueue, &buzCmd, 0);
      Display_Update(EMERGENCY, 0, 0);
    } else if (evt.type == CONTROL_EVT_REARM_REQUEST) {
      if (emergencyActive) {
        printf("[Safety] Re-arming requested\r\n");
        emergencyActive = false;
        BuzzerCommand buzCmd = BUZZER_CMD_ALARM_STOP;
        (void)xQueueSend(buzzerQueue, &buzCmd, 0);
        BuzzerCommand beepCmd = BUZZER_CMD_BEEP;
        (void)xQueueSend(buzzerQueue, &beepCmd, 0);
        MotorCommand stopCmd = { .state = STOP, .duty_percent = 0U };
        (void)xQueueOverwrite(motorCmdQueue, &stopCmd);
        Display_Update(STOP, 0, 0);
      }
    }
  }
}