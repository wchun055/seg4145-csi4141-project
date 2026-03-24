/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "motor.h"
#include "keypad.h"
#include "buzzer.h"
#include "controller.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* ========== Hardware Pin Definitions (Lab 4) ========== */

/* Motor Control Pins */
#define MOTOR_PWM_Pin           GPIO_PIN_9          /* TIM1_CH1 on PE9 */
#define MOTOR_PWM_GPIO_Port     GPIOE
#define MOTOR_AI1_Pin           GPIO_PIN_10         /* Direction A on PB10 */
#define MOTOR_AI1_GPIO_Port     GPIOB
#define MOTOR_AI2_Pin           GPIO_PIN_11         /* Direction B on PB11 */
#define MOTOR_AI2_GPIO_Port     GPIOB
#define MOTOR_STB_Pin           GPIO_PIN_15         /* Standby on PE15 */
#define MOTOR_STB_GPIO_Port     GPIOE

/* Encoder Input Pins */
#define ENCODER_CHA_Pin         GPIO_PIN_0          /* Channel A on PB0 (EXTI) */
#define ENCODER_CHA_GPIO_Port   GPIOB
#define ENCODER_CHB_Pin         GPIO_PIN_1          /* Channel B on PB1 (EXTI) */
#define ENCODER_CHB_GPIO_Port   GPIOB

/* Buzzer Pin */
#define BUZZER_Pin              GPIO_PIN_0          /* Buzzer on PA0 */
#define BUZZER_GPIO_Port        GPIOA

/* LCD I2C Pins (already defined in existing code) */
#define LCD_I2C_SDA_Pin         GPIO_PIN_0          /* I2C2_SDA on PF0 */
#define LCD_I2C_SDA_GPIO_Port   GPIOF
#define LCD_I2C_SCL_Pin         GPIO_PIN_1          /* I2C2_SCL on PF1 */
#define LCD_I2C_SCL_GPIO_Port   GPIOF

/* Keypad/Touch Pins */
#define TOUCH_EN_Pin            GPIO_PIN_5          /* Touch enable on PB5 */
#define TOUCH_EN_GPIO_Port      GPIOB
#define TOUCH_INT_Pin           GPIO_PIN_6          /* Touch interrupt on PB6 */
#define TOUCH_INT_GPIO_Port     GPIOB

/* ========== Task Timing ========== */
#define MOTOR_CONTROL_TASK_PERIOD_MS    10U    /* 10 ms = 100 Hz control loop */
#define ENCODER_UPDATE_PERIOD_MS        10U    /* 10 ms measurement window */
#define UI_TASK_PERIOD_MS               50U    /* 50 ms for LCD/keypad updates */
#define BUZZER_UPDATE_PERIOD_MS         10U    /* 10 ms beep timing */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
