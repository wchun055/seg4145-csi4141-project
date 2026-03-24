/**
 * @file keypad.h
 * @brief Keypad/Touch interface for speed preset selection
 * 
 * Handles user input from keypad/touch sensors for motor speed control.
 * Supports multiple touch protocols (AT42QT1070, MPR121).
 */

#ifndef INC_KEYPAD_H_
#define INC_KEYPAD_H_

#include <stdint.h>
#include <stdbool.h>

/* Keypad preset mappings (Lab 4 requirements) */
typedef enum {
    KEYPAD_PRESET_STOP,      /* Key "0": Stop motor */
    KEYPAD_PRESET_LOW,       /* Key "1": Low speed (1 beep) */
    KEYPAD_PRESET_MEDIUM,    /* Key "2": Medium speed (2 beeps) */
    KEYPAD_PRESET_HIGH,      /* Key "3": High speed (3 beeps) */
    KEYPAD_PRESET_NONE       /* No valid preset detected */
} KeypadPreset;

/**
 * @brief Speed preset configuration
 */
typedef struct {
    uint16_t lowSpeedRPM;       /* Low speed target (RPM) */
    uint16_t mediumSpeedRPM;    /* Medium speed target (RPM) */
    uint16_t highSpeedRPM;      /* High speed target (RPM) */
} KeypadSettings;

/**
 * @brief Initialize keypad system
 * 
 * Configures touch sensor interface (I2C, GPIO), detects touch protocol (AT42QT1070 or MPR121).
 * Must be called during system initialization.
 */
void Keypad_Init(void);

/**
 * @brief Read current keypad preset selection
 * 
 * @return Current pressed preset (KEYPAD_PRESET_STOP, LOW, MEDIUM, HIGH, or NONE)
 * 
 * Returns KEYPAD_PRESET_NONE if no key is currently pressed.
 */
KeypadPreset Keypad_ReadPreset(void);

/**
 * @brief Check if a new preset was just selected (edge detection)
 * 
 * @param preset Pointer to store detected preset
 * @return true if a new preset was detected since last call, false otherwise
 * 
 * Useful for debouncing and ensuring single action per key press.
 */
bool Keypad_GetNewPreset(KeypadPreset *preset);

/**
 * @brief Get the number of beeps for a preset
 * 
 * @param preset Preset type
 * @return Number of beeps (0, 1, 2, or 3)
 */
uint8_t Keypad_GetBeepCount(KeypadPreset preset);

/**
 * @brief Get the target RPM for a preset
 * 
 * @param preset Preset type
 * @return Target speed in RPM (0 for STOP, or configured speed for 1/2/3)
 */
uint16_t Keypad_GetTargetRPM(KeypadPreset preset);

/**
 * @brief Set custom preset speeds
 * 
 * @param settings Structure with low/medium/high speed settings
 */
void Keypad_SetPresetSpeeds(const KeypadSettings *settings);

/**
 * @brief Periodic update for keypad processing
 * 
 * Should be called from a task (not ISR) to handle I2C reads and debouncing.
 */
void Keypad_Update(void);

#endif /* INC_KEYPAD_H_ */
