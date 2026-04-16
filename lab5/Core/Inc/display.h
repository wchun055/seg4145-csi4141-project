/**
  * @file display.h
  * @brief Header for display module
  */

#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "main.h"

typedef enum {STOP, CW, CCW, EMERGENCY} MotorState;

void Display_Init(void);
void Display_UpdateDisplay(MotorState s, uint16_t targetRPM, uint16_t measuredRPM);

#endif /* __DISPLAY_H */