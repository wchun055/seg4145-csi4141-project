/**
 * @file controller.c
 * @brief Closed-loop speed controller implementation
 * 
 * PI controller for motor speed regulation with integral windup protection.
 */

#include "controller.h"
#include <string.h>
#include <math.h>

/* Controller state machine and variables */
static struct {
    ControllerState state;
    uint16_t targetRPM;
    float integralTerm;
    uint16_t lastPWM;
} controlCtx = {
    .state = CONTROL_IDLE,
    .targetRPM = 0,
    .integralTerm = 0.0f,
    .lastPWM = 0
};

/* Controller configuration with reasonable defaults */
static ControllerConfig controlConfig = {
    .proportional_gain = 0.5f,   /* Kp */
    .integral_gain = 0.05f,       /* Ki */
    .errorThreshold = 50,         /* 50 RPM error threshold */
    .maxPWMDuty = 100,
    .minPWMDuty = 20             /* Minimum PWM for motor to overcome friction */
};

/**
 * @brief Controller_Init
 */
void Controller_Init(void)
{
    controlCtx.state = CONTROL_IDLE;
    controlCtx.targetRPM = 0;
    controlCtx.integralTerm = 0.0f;
    controlCtx.lastPWM = 0;
}

/**
 * @brief Controller_SetTarget
 */
void Controller_SetTarget(uint16_t targetRPM)
{
    if (targetRPM == 0) {
        /* Stopping */
        controlCtx.state = CONTROL_IDLE;
        controlCtx.targetRPM = 0;
        controlCtx.integralTerm = 0.0f;
    } else {
        /* Starting control loop */
        controlCtx.state = CONTROL_RUNNING;
        controlCtx.targetRPM = targetRPM;
        controlCtx.integralTerm = 0.0f;  /* Reset integral on new target */
    }
}

/**
 * @brief Controller_GetTarget
 */
uint16_t Controller_GetTarget(void)
{
    return controlCtx.targetRPM;
}

/**
 * @brief Controller_Update
 * 
 * PI controller law:
 * PWM = Kp * error + Ki * integral(error)
 * With anti-windup protection and output saturation.
 */
uint16_t Controller_Update(uint16_t measuredRPM)
{
    if (controlCtx.state == CONTROL_IDLE) {
        return 0;  /* No control active */
    }
    
    /* Compute speed error */
    int32_t error = (int32_t)controlCtx.targetRPM - (int32_t)measuredRPM;
    
    /* Proportional term */
    float pTerm = controlConfig.proportional_gain * (float)error;
    
    /* Integral term with anti-windup */
    controlCtx.integralTerm += controlConfig.integral_gain * (float)error;
    
    /* Anti-windup: clamp integral term to prevent excessive accumulation */
    if (controlCtx.integralTerm > 50.0f) {
        controlCtx.integralTerm = 50.0f;
    } else if (controlCtx.integralTerm < -50.0f) {
        controlCtx.integralTerm = -50.0f;
    }
    
    /* Total control output */
    float controlOutput = pTerm + controlCtx.integralTerm;
    
    /* Convert to PWM duty cycle (0-100) */
    uint16_t pwmDuty = 0;
    
    if (controlOutput > 0) {
        /* Accelerate: ensure minimum threshold to overcome motor friction */
        pwmDuty = (uint16_t)controlOutput;
        if (pwmDuty > 0 && pwmDuty < controlConfig.minPWMDuty) {
            pwmDuty = controlConfig.minPWMDuty;
        }
    } else if (controlOutput < 0) {
        /* Should not happen in this lab (single direction), but handle gracefully */
        pwmDuty = 0;
    }
    
    /* Saturate to maximum duty cycle */
    if (pwmDuty > controlConfig.maxPWMDuty) {
        pwmDuty = controlConfig.maxPWMDuty;
    }
    
    /* Check for excessive error */
    if (error > (int32_t)controlConfig.errorThreshold * 2) {
        controlCtx.state = CONTROL_ERROR;  /* Could trigger alert or recovery */
    }
    
    controlCtx.lastPWM = pwmDuty;
    return pwmDuty;
}

/**
 * @brief Controller_GetState
 */
ControllerState Controller_GetState(void)
{
    return controlCtx.state;
}

/**
 * @brief Controller_ResetIntegral
 */
void Controller_ResetIntegral(void)
{
    controlCtx.integralTerm = 0.0f;
}

/**
 * @brief Controller_SetConfig
 */
void Controller_SetConfig(const ControllerConfig *config)
{
    if (config != NULL) {
        memcpy(&controlConfig, config, sizeof(ControllerConfig));
    }
}

/**
 * @brief Controller_HasConverged
 */
bool Controller_HasConverged(void)
{
    /* Not implemented in basic version; could track steady-state error */
    return false;
}
