/**
  * @file buzzer.h
  * @brief Header for buzzer module
  */

#ifndef __BUZZER_H
#define __BUZZER_H

#include "main.h"

typedef enum {
    BUZZER_CMD_BEEP = 0,
    BUZZER_CMD_ALARM_START,
    BUZZER_CMD_ALARM_STOP
} BuzzerCommand;

void Buzzer_Init(void);
void Buzzer_ProcessCommand(BuzzerCommand cmd);

#endif /* __BUZZER_H */