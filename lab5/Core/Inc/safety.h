/**
  * @file safety.h
  * @brief Header for safety module
  */

#ifndef __SAFETY_H
#define __SAFETY_H

#include "main.h"

typedef enum {STOP, CW, CCW, EMERGENCY} MotorState;

typedef struct {
    MotorState state;
    uint16_t duty_percent;
} MotorCommand;

typedef enum {
    BUZZER_CMD_BEEP = 0,
    BUZZER_CMD_ALARM_START,
    BUZZER_CMD_ALARM_STOP
} BuzzerCommand;

void Safety_Init(void);
void Safety_ProcessEvent(ControlEvent *evt);

#endif /* __SAFETY_H */