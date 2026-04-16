/**
  * @file keypad.h
  * @brief Header for keypad module
  */

#ifndef __KEYPAD_H
#define __KEYPAD_H

#include "main.h"

typedef enum {STOP, CW, CCW, EMERGENCY} MotorState;

bool Keypad_Init(void);
bool Keypad_ReadCommandState(MotorState *stateOut);
void Keypad_Enable(bool enable);

#endif /* __KEYPAD_H */