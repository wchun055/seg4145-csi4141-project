/**
 * @file encoder.h
 * @brief Encoder-based speed measurement module for DC motor
 * 
 * This module handles quadrature encoder signal processing and RPM calculation.
 * Encoder configuration: 11 pulses per motor shaft revolution, 34:1 gear ratio
 * Result: 374 pulses per geared output shaft revolution per channel
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>
#include <stdbool.h>

/* Encoder configuration */
#define ENCODER_PULSES_PER_SHAFT_REV    11U     /* Pulses at motor shaft */
#define ENCODER_GEAR_RATIO              34U     /* Gear reduction ratio */
#define ENCODER_PULSES_PER_OUTPUT_REV   (ENCODER_PULSES_PER_SHAFT_REV * ENCODER_GEAR_RATIO) /* 374 pulses/revolution */

/* Measurement period in milliseconds */
#define ENCODER_MEASUREMENT_PERIOD_MS   10U

/**
 * @brief Initialize encoder measurement system
 * 
 * Sets up GPIO interrupts for encoder channels A (PB0) and B (PB1),
 * initializes counters and configures periodic measurement.
 */
void Encoder_Init(void);

/**
 * @brief Handle encoder channel A interrupt (rising/falling edge)
 * 
 * Called from EXTI interrupt handler for PB0.
 * Increments encoder pulse counter with direction tracking.
 */
void Encoder_HandleChannelA_IRQ(void);

/**
 * @brief Handle encoder channel B interrupt (rising/falling edge)
 * 
 * Called from EXTI interrupt handler for PB1.
 * Used for direction detection in quadrature encoding.
 */
void Encoder_HandleChannelB_IRQ(void);

/**
 * @brief Get the latest measured speed in RPM
 * 
 * @return Motor speed in revolutions per minute (RPM)
 * 
 * This value is updated periodically (every ENCODER_MEASUREMENT_PERIOD_MS).
 * Speed is computed from pulse counts over the measurement window.
 */
uint16_t Encoder_GetSpeedRPM(void);

/**
 * @brief Get the raw pulse counter value
 * 
 * @return Current encoder pulse count
 * 
 * Useful for debugging and timing analysis.
 */
uint32_t Encoder_GetPulseCount(void);

/**
 * @brief Reset the encoder pulse counter
 * 
 * Clears the internal pulse counter (typically called once at startup).
 */
void Encoder_ResetCounter(void);

/**
 * @brief Process encoder measurement (called periodically from a task)
 * 
 * Computes RPM from pulse count over the measurement period.
 * Call this function periodically (e.g., every 10 ms) from the control task.
 */
void Encoder_Update(void);

#endif /* INC_ENCODER_H_ */
