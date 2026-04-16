/**
  * @file display.c
  * @brief Display module implementation
  */

#include "display.h"
#include "lcd.h"
#include <stdio.h>

static bool displayReady = false;

void Display_Init(void)
{
  LCD_Init();
  displayReady = true;
  setLCD_RGB(32, 32, 32);
  printf("[DISPLAY] LCD initialized\r\n");
}

static const char *MotorStateStr(MotorState s)
{
  if (s == CW) return "CW";
  if (s == CCW) return "CCW";
  if (s == EMERGENCY) return "EMERGENCY";
  return "STOP";
}

void Display_UpdateDisplay(MotorState s, uint16_t targetRPM, uint16_t measuredRPM)
{
  if (!displayReady) return;
  char line1[17];
  char line2[17];

  if (s == STOP) {
    snprintf(line1, sizeof(line1), "State: STOP");
    snprintf(line2, sizeof(line2), "Actual: 0 RPM");
  } else if (s == CW) {
    snprintf(line1, sizeof(line1), "CW Tgt: %3dRPM", targetRPM);
    snprintf(line2, sizeof(line2), "Act: %3d RPM", measuredRPM);
  } else if (s == CCW) {
    snprintf(line1, sizeof(line1), "CCW Tgt: %3dRPM", targetRPM);
    snprintf(line2, sizeof(line2), "Act: %3d RPM", measuredRPM);
  } else if (s == EMERGENCY) {
    snprintf(line1, sizeof(line1), "!! EMERGENCY !!");
    snprintf(line2, sizeof(line2), "Locked          0RPM");
  } else {
    snprintf(line1, sizeof(line1), "State: %s", MotorStateStr(s));
    snprintf(line2, sizeof(line2), "Actual: %3d RPM", measuredRPM);
  }

  LCD_DisplayText(line1, FIRST_LINE);
  LCD_DisplayText(line2, SECOND_LINE);
}