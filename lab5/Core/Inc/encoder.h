/**
  * @file encoder.h
  * @brief Header for encoder module
  */

#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

void Encoder_Init(void);
uint16_t Encoder_GetRPM(void);

#endif /* __ENCODER_H */