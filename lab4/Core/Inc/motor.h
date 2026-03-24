/**
 * @file motor.h
 * @brief DC motor control module
 * 
 * Provides PWM-based motor speed and direction control.
 * Motor uses TIM1_CH1 (PE9) for PWM, with AI1/AI2 for direction control.
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>
#include <stdbool.h>

/* Motor state definitions */
typedef enum {
    MOTOR_STOP = 0,     /* Motor disabled, standby active */
    MOTOR_CW = 1,       /* Clockwise rotation */
    MOTOR_CCW = 2       /* Counter-clockwise rotation */
} MotorState;

/* Motor control limits */
#define MOTOR_PWM_MIN_DUTY      0U      /* Minimum PWM duty cycle (0%) */
#define MOTOR_PWM_MAX_DUTY      100U    /* Maximum PWM duty cycle (100%) */
#define MOTOR_PWM_DEFAULT_DUTY  0U      /* Default startup duty cycle */

/* Hardware GPIO pins (from Lab 4 specification)
 * Standby (STB): PE15
 * PWM (TIM1_CH1): PE9
 * AI1: PB10
 * AI2: PB11
 */

/**
 * @brief Initialize motor control system
 * 
 * Configures PWM timer (TIM1), GPIO pins for direction control (AI1, AI2),
 * and standby pin (STB). Motor starts in STOP state.
 */
void Motor_Init(void);

/**
 * @brief Set motor state (direction) and PWM duty cycle
 * 
 * @param state Motor state (MOTOR_STOP, MOTOR_CW, or MOTOR_CCW)
 * @param dutyCycle PWM duty cycle as percentage (0-100)
 * 
 * If state is MOTOR_STOP, dutyCycle is ignored and PWM is set to 0.
 * Duty cycle values outside 0-100 are clamped.
 */
void Motor_SetCommand(MotorState state, uint16_t dutyCycle);

/**
 * @brief Get current motor state
 * 
 * @return Current motor state (MOTOR_STOP, MOTOR_CW, or MOTOR_CCW)
 */
MotorState Motor_GetState(void);

/**
 * @brief Get current PWM duty cycle
 * 
 * @return PWM duty cycle as percentage (0-100)
 */
uint16_t Motor_GetDutyCycle(void);

/**
 * @brief Emergency stop - immediately halt motor
 * 
 * Sets motor to MOTOR_STOP state with 0% duty cycle.
 * Can be called from ISR or task context.
 */
void Motor_EmergencyStop(void);

/**
 * @brief Set PWM duty cycle while maintaining current direction
 * 
 * @param dutyCycle PWM duty cycle as percentage (0-100)
 * 
 * Useful for closed-loop control where only PWM needs adjustment.
 */
void Motor_SetDutyCycle(uint16_t dutyCycle);

/**
 * @brief Convert motor state to string (for debugging)
 * 
 * @param state Motor state
 * @return String representation ("STOP", "CW", or "CCW")
 */
const char* Motor_StateToString(MotorState state);

#endif /* INC_MOTOR_H_ */
