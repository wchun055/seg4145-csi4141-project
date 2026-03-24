/**
 * @file controller.h
 * @brief Closed-loop speed controller for DC motor
 * 
 * Implements a proportional (P) or proportional-integral (PI) controller
 * for automatic PWM adjustment based on speed error.
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Controller state
 */
typedef enum {
    CONTROL_IDLE,       /* Controller inactive (no target set) */
    CONTROL_RUNNING,    /* Closed-loop control active */
    CONTROL_ERROR       /* Error state (e.g., excessive speed overshoot) */
} ControllerState;

/**
 * @brief Controller configuration
 */
typedef struct {
    float proportional_gain;     /* Kp: proportional gain */
    float integral_gain;         /* Ki: integral gain (for PI controller) */
    uint16_t errorThreshold;     /* Maximum allowed speed error before error state */
    uint16_t maxPWMDuty;         /* Maximum PWM duty cycle (100 typical) */
    uint16_t minPWMDuty;         /* Minimum PWM duty cycle for motor to turn */
} ControllerConfig;

/**
 * @brief Initialize closed-loop speed controller
 * 
 * Sets default PID gains and error thresholds.
 */
void Controller_Init(void);

/**
 * @brief Set target speed reference
 * 
 * @param targetRPM Target motor speed in RPM (0 for STOP)
 */
void Controller_SetTarget(uint16_t targetRPM);

/**
 * @brief Get current target speed
 * 
 * @return Target speed in RPM
 */
uint16_t Controller_GetTarget(void);

/**
 * @brief Update controller (periodic call from task)
 * 
 * @param measuredRPM Current measured speed in RPM
 * @return Computed PWM duty cycle (0-100)
 * 
 * Compares measured speed with target, applies control law,
 * returns recommended PWM duty cycle.
 */
uint16_t Controller_Update(uint16_t measuredRPM);

/**
 * @brief Get current controller state
 * 
 * @return CONTROL_IDLE, CONTROL_RUNNING, or CONTROL_ERROR
 */
ControllerState Controller_GetState(void);

/**
 * @brief Reset controller integrator
 * 
 * Clears integral term (called on step input or preset change).
 */
void Controller_ResetIntegral(void);

/**
 * @brief Set controller gains
 * 
 * @param config Controller configuration structure
 */
void Controller_SetConfig(const ControllerConfig *config);

/**
 * @brief Check if controller has converged to target
 * 
 * @return true if error is within specified band, false otherwise
 */
bool Controller_HasConverged(void);

#endif /* INC_CONTROLLER_H_ */
