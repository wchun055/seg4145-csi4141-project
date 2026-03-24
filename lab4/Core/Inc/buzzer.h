/**
 * @file buzzer.h
 * @brief Non-blocking buzzer control module
 * 
 * Provides audible feedback for user interactions (keypad presets).
 * Buzzer output on PA0 (GPIO).
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include <stdint.h>
#include <stdbool.h>

/* Buzzer configuration */
#define BUZZER_BEEP_DURATION_MS     50U  /* Duration of a single beep */

/**
 * @brief Initialize buzzer system
 * 
 * Configures GPIO PA0 as output and sets up timer for non-blocking beep patterns.
 * Must be called during system initialization.
 */
void Buzzer_Init(void);

/**
 * @brief Generate a non-blocking beep pattern
 * 
 * @param beepCount Number of beeps (1, 2, 3, etc.)
 * 
 * Starts a timer-based beep pattern. Does not block.
 * Safe to call from task or ISR context.
 * Examples:
 *   Buzzer_BeepPattern(1);  // Low speed preset
 *   Buzzer_BeepPattern(2);  // Medium speed preset
 *   Buzzer_BeepPattern(3);  // High speed preset
 */
void Buzzer_BeepPattern(uint8_t beepCount);

/**
 * @brief Check if buzzer is currently active
 * 
 * @return true if beeping pattern is in progress, false otherwise
 */
bool Buzzer_IsActive(void);

/**
 * @brief Stop buzzer immediately
 * 
 * Cancels any active beep pattern and turns off buzzer.
 */
void Buzzer_Stop(void);

/**
 * @brief Periodic update function for buzzer timing
 * 
 * Must be called periodically (e.g., 10 ms) to maintain beep timing.
 * Typically called from a FreeRTOS task or timer callback.
 */
void Buzzer_Update(void);

#endif /* INC_BUZZER_H_ */
