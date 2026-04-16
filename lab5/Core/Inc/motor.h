/**
  * @file motor.h
  * @brief Header for motor module
  */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

typedef enum {STOP, CW, CCW, EMERGENCY} MotorState;

typedef struct {
    MotorState state;
    uint16_t duty_percent;
} MotorCommand;

void Motor_Init(void);
void Motor_ApplyCommand(const MotorCommand *cmd);

#endif /* __MOTOR_H */