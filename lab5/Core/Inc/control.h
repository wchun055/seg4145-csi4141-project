/**
  * @file control.h
  * @brief Header for control module
  */

#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"

typedef enum {STOP, CW, CCW, EMERGENCY} MotorState;

typedef struct {
    MotorState state;
    uint16_t duty_percent;
} MotorCommand;

void Control_Init(void);
void Control_ProcessEvent(ControlEvent *evt);

#endif /* __CONTROL_H */