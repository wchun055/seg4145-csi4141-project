/**
  * @file adc.c
  * @brief ADC module implementation
  */

#include "adc.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;

void ADC_Init(void)
{}

uint16_t ADC_ReadDutyPercent(void)
{
  uint32_t adcValue = 0;
  if (HAL_ADC_Start(&hadc1) == HAL_OK) {
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
      adcValue = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
  }
  return (uint16_t)((adcValue * 100U) / 4095U);
}