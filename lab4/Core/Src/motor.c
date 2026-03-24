/**
 * @file motor.c
 * @brief DC motor control implementation
 * 
 * Handles PWM-based motor speed control and direction selection.
 */

#include "motor.h"
#include "main.h"
#include <string.h>

/* Motor state variables */
static MotorState currentState = MOTOR_STOP;
static uint16_t currentDutyCycle = 0;

/**
 * @brief Motor_Init
 * 
 * Initialize motor control: PWM timer, direction GPIO, and standby pin.
 * This is called during system startup.
 * Timer and GPIO must be pre-configured in STM32CubeMX:
 * - TIM1_CH1 (PE9) for PWM
 * - PB10 (AI1) for direction
 * - PB11 (AI2) for direction
 * - PE15 (STB) for standby
 */
void Motor_Init(void)
{
    /* Initialize motor state */
    currentState = MOTOR_STOP;
    currentDutyCycle = 0;
    
    /* Ensure motor standby is enabled initially (active high)
     * Note: GPIO initialization is done by STM32CubeMX
     */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);  /* STB active (standby disabled) */
    
    /* Direction pins to STOP state (both low or high - implementation dependent) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  /* AI1 = 0 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);  /* AI2 = 0 */
    
    /* Start PWM at 0% duty cycle */
    if (htim1.Instance != NULL) {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        TIM1->CCR1 = 0;  /* 0% duty cycle */
    }
}

/**
 * @brief Motor_SetCommand
 * 
 * Set motor state and PWM duty cycle.
 */
void Motor_SetCommand(MotorState state, uint16_t dutyCycle)
{
    /* Clamp duty cycle to valid range */
    if (dutyCycle > MOTOR_PWM_MAX_DUTY) {
        dutyCycle = MOTOR_PWM_MAX_DUTY;
    }
    
    /* If stopping, force duty cycle to 0 */
    if (state == MOTOR_STOP) {
        dutyCycle = 0;
    }
    
    currentState = state;
    currentDutyCycle = dutyCycle;
    
    /* Apply motor command */
    switch (state) {
        case MOTOR_STOP:
            /* Disable motor: set both AI pins to 0 and PWM to 0 */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  /* AI1 = 0 */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);  /* AI2 = 0 */
            if (htim1.Instance != NULL) {
                TIM1->CCR1 = 0;
            }
            break;
            
        case MOTOR_CW:
            /* Clockwise: AI1=1, AI2=0 */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);    /* AI1 = 1 */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);  /* AI2 = 0 */
            if (htim1.Instance != NULL) {
                /* Set PWM duty cycle: compute ARR-based value */
                uint32_t ccr_value = (dutyCycle * TIM1->ARR) / 100U;
                TIM1->CCR1 = ccr_value;
            }
            break;
            
        case MOTOR_CCW:
            /* Counter-clockwise: AI1=0, AI2=1 */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  /* AI1 = 0 */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);    /* AI2 = 1 */
            if (htim1.Instance != NULL) {
                /* Set PWM duty cycle */
                uint32_t ccr_value = (dutyCycle * TIM1->ARR) / 100U;
                TIM1->CCR1 = ccr_value;
            }
            break;
    }
}

/**
 * @brief Motor_GetState
 */
MotorState Motor_GetState(void)
{
    return currentState;
}

/**
 * @brief Motor_GetDutyCycle
 */
uint16_t Motor_GetDutyCycle(void)
{
    return currentDutyCycle;
}

/**
 * @brief Motor_EmergencyStop
 */
void Motor_EmergencyStop(void)
{
    Motor_SetCommand(MOTOR_STOP, 0);
}

/**
 * @brief Motor_SetDutyCycle
 * 
 * Adjust only the PWM duty cycle, preserving current direction.
 */
void Motor_SetDutyCycle(uint16_t dutyCycle)
{
    Motor_SetCommand(currentState, dutyCycle);
}

/**
 * @brief Motor_StateToString
 */
const char* Motor_StateToString(MotorState state)
{
    switch (state) {
        case MOTOR_STOP:  return "STOP";
        case MOTOR_CW:    return "CW";
        case MOTOR_CCW:   return "CCW";
        default:          return "UNKNOWN";
    }
}
