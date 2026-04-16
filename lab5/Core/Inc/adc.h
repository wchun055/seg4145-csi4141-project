/**
  * @file adc.h
  * @brief Header for ADC module
  */

#ifndef __ADC_H
#define __ADC_H

#include "main.h"

void ADC_Init(void);
uint16_t ADC_ReadDutyPercent(void);

#endif /* __ADC_H */