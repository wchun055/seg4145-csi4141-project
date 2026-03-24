/**
 * @file keypad.c
 * @brief Keypad/Touch interface implementation
 * 
 * Provides preset selection from touch sensor with multiple protocol support.
 */

#include "keypad.h"
#include "main.h"
#include <string.h>

/* Keypad state */
static KeypadSettings presetSpeeds = {
    .lowSpeedRPM = 3,       /* Example: 3 RPM */
    .mediumSpeedRPM = 6,    /* Example: 6 RPM */
    .highSpeedRPM = 8       /* Example: 8 RPM */
};

static KeypadPreset lastPreset = KEYPAD_PRESET_NONE;
static KeypadPreset currentPreset = KEYPAD_PRESET_NONE;
static bool newPresetDetected = false;

/* Touch protocol detection */
extern TouchProtocol touchProtocol;  /* From main.c */

/**
 * @brief Keypad_Init
 */
void Keypad_Init(void)
{
    /* Touch sensor initialization is done in main.c at startup.
     * This function is reserved for any keypad-specific setup.
     */
    lastPreset = KEYPAD_PRESET_NONE;
    currentPreset = KEYPAD_PRESET_NONE;
    newPresetDetected = false;
}

/**
 * @brief Keypad_ReadPreset
 * 
 * Maps touch key bits to preset enum.
 * Key mapping:
 *   Key 0 -> Bit 0 -> STOP
 *   Key 1 -> Bit 1 -> LOW speed
 *   Key 2 -> Bit 2 -> MEDIUM speed
 *   Key 3 -> Bit 3 -> HIGH speed
 */
KeypadPreset Keypad_ReadPreset(void)
{
    uint8_t data[2] = {0};
    uint16_t mask = 0;
    
    /* Extern from main.c - read touch status from I2C */
    extern I2C_HandleTypeDef hi2c2;
    extern TouchProtocol touchProtocol;
    extern uint16_t touchI2cAddr;
    
    if (touchProtocol == TOUCH_PROTO_NONE) {
        return KEYPAD_PRESET_NONE;
    }
    
    if (touchProtocol == TOUCH_PROTO_AT42QT1070) {
        if (HAL_I2C_Mem_Read(&hi2c2, touchI2cAddr, 0x03, I2C_MEMADD_SIZE_8BIT,
                             data, 1, 20) != HAL_OK) {
            return KEYPAD_PRESET_NONE;
        }
        mask = data[0];
    } else if (touchProtocol == TOUCH_PROTO_MPR121) {
        if (HAL_I2C_Mem_Read(&hi2c2, touchI2cAddr, 0x00, I2C_MEMADD_SIZE_8BIT,
                             data, 2, 20) != HAL_OK) {
            return KEYPAD_PRESET_NONE;
        }
        mask = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    }
    
    /* Priority: STOP(key 0) > LOW(key 1) > MEDIUM(key 2) > HIGH(key 3) */
    if (mask & (1U << 0)) {
        return KEYPAD_PRESET_STOP;
    }
    if (mask & (1U << 1)) {
        return KEYPAD_PRESET_LOW;
    }
    if (mask & (1U << 2)) {
        return KEYPAD_PRESET_MEDIUM;
    }
    if (mask & (1U << 3)) {
        return KEYPAD_PRESET_HIGH;
    }
    
    return KEYPAD_PRESET_NONE;
}

/**
 * @brief Keypad_GetNewPreset
 * 
 * Edge detection: returns true only on transition to new preset.
 */
bool Keypad_GetNewPreset(KeypadPreset *preset)
{
    if (preset == NULL) {
        return false;
    }
    
    if (newPresetDetected) {
        *preset = currentPreset;
        newPresetDetected = false;
        return true;
    }
    
    return false;
}

/**
 * @brief Keypad_GetBeepCount
 */
uint8_t Keypad_GetBeepCount(KeypadPreset preset)
{
    switch (preset) {
        case KEYPAD_PRESET_STOP:    return 0;
        case KEYPAD_PRESET_LOW:     return 1;
        case KEYPAD_PRESET_MEDIUM:  return 2;
        case KEYPAD_PRESET_HIGH:    return 3;
        default:                    return 0;
    }
}

/**
 * @brief Keypad_GetTargetRPM
 */
uint16_t Keypad_GetTargetRPM(KeypadPreset preset)
{
    switch (preset) {
        case KEYPAD_PRESET_STOP:    return 0;
        case KEYPAD_PRESET_LOW:     return presetSpeeds.lowSpeedRPM;
        case KEYPAD_PRESET_MEDIUM:  return presetSpeeds.mediumSpeedRPM;
        case KEYPAD_PRESET_HIGH:    return presetSpeeds.highSpeedRPM;
        default:                    return 0;
    }
}

/**
 * @brief Keypad_SetPresetSpeeds
 */
void Keypad_SetPresetSpeeds(const KeypadSettings *settings)
{
    if (settings != NULL) {
        memcpy(&presetSpeeds, settings, sizeof(KeypadSettings));
    }
}

/**
 * @brief Keypad_Update
 * 
 * Periodic task: reads keypad, detects new presets
 */
void Keypad_Update(void)
{
    KeypadPreset current = Keypad_ReadPreset();
    
    /* Edge detection: new preset if different from last */
    if (current != lastPreset && current != KEYPAD_PRESET_NONE) {
        currentPreset = current;
        newPresetDetected = true;
        lastPreset = current;
    } else if (current == KEYPAD_PRESET_NONE) {
        lastPreset = KEYPAD_PRESET_NONE;
    }
}
