/**
  * @file encoder.c
  * @brief Encoder module implementation
  */

#include "encoder.h"
#include "main.h"

#define ENCODER_PULSES_PER_REV 1000U
#define ENCODER_TASK_PERIOD_MS 20U

volatile int32_t encoderCount = 0;

void Encoder_Init(void)
{}

uint16_t Encoder_GetRPM(void)
{
  static int32_t lastCount = 0;
  static TickType_t lastTime = 0;
  TickType_t now = xTaskGetTickCount();
  int32_t delta = encoderCount - lastCount;
  uint16_t rpm = 0;

  if (now > lastTime) {
    uint32_t timeDiff = now - lastTime;
    rpm = (uint16_t)((abs(delta) * 1000U / timeDiff) / ENCODER_PULSES_PER_REV * 60);
  }

  lastCount = encoderCount;
  lastTime = now;

  return rpm;
}