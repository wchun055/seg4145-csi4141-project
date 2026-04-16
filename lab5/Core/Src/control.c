/**
  * @file control.c
  * @brief Control module implementation
  */

#include "control.h"
#include "keypad.h"
#include "adc.h"
#include "encoder.h"
#include "motor.h"
#include "buzzer.h"
#include "display.h"
#include "main.h"

#define BUTTON_DEBOUNCE_MS 150U
#define TOUCH_IRQ_DEBOUNCE_MS 20U

extern QueueHandle_t controlEventQueue;
extern QueueHandle_t motorCmdQueue;
extern QueueHandle_t motorAppliedQueue;
extern QueueHandle_t buzzerQueue;
extern bool emergencyActive;

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

static MotorState state = STOP;
static uint16_t duty = 0;

static MotorState CycleState(MotorState s)
{
  if (s == STOP) return CW;
  if (s == CW) return CCW;
  return STOP;
}

void Control_Init(void)
{
  MotorCommand cmd = { .state = STOP, .duty_percent = 0U };
  (void)xQueueOverwrite(motorCmdQueue, &cmd);
  Display_Update(STOP, 0, 0);
}

void Control_ProcessEvent(void)
{
  ControlEvent evt;
  if (xQueueReceive(controlEventQueue, &evt, pdMS_TO_TICKS(20)) != pdPASS) {
    return;
  }

  TickType_t now = xTaskGetTickCount();
  bool publish = false;
  bool stateChanged = false;
  const char *line2 = NULL;

  if (evt.type == CONTROL_EVT_BUTTON_PRESS) {
    if (!emergencyActive && (now - 0) >= pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) { 
      state = CycleState(state);
      stateChanged = true;
      publish = true;
      printf("[Control] Button -> %s\r\n", state == CW ? "CW" : state == CCW ? "CCW" : "STOP");
    }
  } else if (evt.type == CONTROL_EVT_TOUCH_IRQ) {
    if ((now - 0) >= pdMS_TO_TICKS(TOUCH_IRQ_DEBOUNCE_MS)) {
      MotorState touchState;
      if (Keypad_ReadCommandState(&touchState)) {
        if (emergencyActive) {
          if (touchState == EMERGENCY) {
            ControlEvent rearmEvt = { .type = CONTROL_EVT_REARM_REQUEST, .value = 0 };
            (void)xQueueSend(controlEventQueue, &rearmEvt, 0);
          }
        } else {
          state = touchState;
          stateChanged = true;
          publish = true;
          printf("[Control] Touch -> %s\r\n", state == CW ? "CW" : state == CCW ? "CCW" : "STOP");
        }
      }
    }
  } else if (evt.type == CONTROL_EVT_ADC_DUTY) {
    if (evt.value <= 100U && evt.value != duty) {
      duty = evt.value;
      if (state != STOP && !emergencyActive) {
        publish = true;
      }
    }
  }

  if (publish && !emergencyActive) {
    MotorCommand cmd = { .state = state, .duty_percent = duty };
    (void)xQueueOverwrite(motorCmdQueue, &cmd);
  }

  if (stateChanged) {
    MotorCommand applied;
    TickType_t waitUntil = xTaskGetTickCount() + pdMS_TO_TICKS(100); 

    if (xQueueReceive(motorAppliedQueue, &applied, waitUntil) == pdPASS && applied.state == state) {
      uint16_t targetRPM = duty * 3;
      uint16_t measuredRPM = Encoder_GetRPM();
      Display_Update(state, targetRPM, measuredRPM);
    }
    BuzzerCommand beepCmd = BUZZER_CMD_BEEP;
    (void)xQueueSend(buzzerQueue, &beepCmd, 0);
  }
}