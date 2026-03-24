/**
 * @file encoder.c
 * @brief Encoder-based speed measurement implementation
 * 
 * Handles quadrature encoder signals from motor encoder inputs.
 * Computes motor speed in RPM from pulse counts over a fixed measurement window.
 */

#include "encoder.h"
#include "main.h"
#include <stdint.h>

/* Internal state */
static volatile uint32_t pulseCount = 0;        /* Total pulses captured */
static volatile uint32_t lastPulseCount = 0;    /* Pulses at last measurement */
static uint16_t measuredSpeedRPM = 0;           /* Current speed in RPM */

/**
 * @brief Encoder_Init
 * 
 * Initialize GPIO interrupts for encoder inputs (PB0, PB1).
 * This function is called during system initialization.
 * GPIO interrupt configuration should be set up in STM32CubeMX or HAL init.
 */
void Encoder_Init(void)
{
    /* GPIO and EXTI configuration is handled by STM32CubeMX
     * This function is reserved for any additional encoder initialization
     * such as timer setup if using input capture mode.
     */
    pulseCount = 0;
    lastPulseCount = 0;
    measuredSpeedRPM = 0;
}

/**
 * @brief Encoder_HandleChannelA_IRQ
 * 
 * Called on PB0 (Encoder Channel A) interrupt.
 * Increments pulse counter for each edge detection.
 */
void Encoder_HandleChannelA_IRQ(void)
{
    /* Simple edge counting - increment on every edge (both rising and falling)
     * For quadrature decoding with direction, additional logic would be needed.
     */
    pulseCount++;
}

/**
 * @brief Encoder_HandleChannelB_IRQ
 * 
 * Called on PB1 (Encoder Channel B) interrupt.
 * Can be used for direction detection in quadrature encoding.
 */
void Encoder_HandleChannelB_IRQ(void)
{
    /* For simple counting, also increment on channel B edges.
     * For quadrature direction detection, examine phase difference between A and B.
     */
    pulseCount++;
}

/**
 * @brief Encoder_GetSpeedRPM
 * 
 * Returns the most recently computed speed in RPM.
 */
uint16_t Encoder_GetSpeedRPM(void)
{
    return measuredSpeedRPM;
}

/**
 * @brief Encoder_GetPulseCount
 * 
 * Returns the raw pulse counter value for debugging/analysis.
 */
uint32_t Encoder_GetPulseCount(void)
{
    return pulseCount;
}

/**
 * @brief Encoder_ResetCounter
 * 
 * Resets the pulse counter to zero.
 */
void Encoder_ResetCounter(void)
{
    pulseCount = 0;
    lastPulseCount = 0;
}

/**
 * @brief Encoder_Update
 * 
 * Computes RPM from pulse count change over the measurement period.
 * Call this periodically (e.g., every ENCODER_MEASUREMENT_PERIOD_MS = 10 ms).
 * 
 * Formula:
 *   RPM = (pulses_per_period / ENCODER_PULSES_PER_OUTPUT_REV) 
 *         * (1000 / ENCODER_MEASUREMENT_PERIOD_MS) * 60 / 1000
 *   RPM = (pulse_delta / ENCODER_PULSES_PER_OUTPUT_REV) * (60000 / ENCODER_MEASUREMENT_PERIOD_MS)
 *   RPM = (pulse_delta / 374) * 6000  [for 10 ms period]
 */
void Encoder_Update(void)
{
    /* Read current pulse count (atomically) */
    uint32_t current = pulseCount;
    
    /* Calculate pulses detected in this period */
    uint32_t deltaPulses = current - lastPulseCount;
    lastPulseCount = current;
    
    /* Convert to RPM
     * Each revolution = ENCODER_PULSES_PER_OUTPUT_REV pulses
     * Period = ENCODER_MEASUREMENT_PERIOD_MS milliseconds
     * RPM = (pulses_per_period / pulses_per_rev) * (60000 / period_ms)
     */
    uint32_t rpmValue = (deltaPulses * 60000UL) 
                      / (ENCODER_PULSES_PER_OUTPUT_REV * ENCODER_MEASUREMENT_PERIOD_MS);
    
    /* Saturate at 16-bit maximum */
    if (rpmValue > 65535UL) {
        measuredSpeedRPM = 65535;
    } else {
        measuredSpeedRPM = (uint16_t)rpmValue;
    }
}
