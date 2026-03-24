/**
 * @file buzzer.c
 * @brief Non-blocking buzzer implementation
 * 
 * State machine-based buzzer driver for generating beep patterns without blocking.
 */

#include "buzzer.h"
#include "main.h"

/* Buzzer state machine */
typedef enum {
    BUZZER_IDLE,        /* Not active */
    BUZZER_BEEPING,     /* Beeper on */
    BUZZER_SILENT       /* Between beeps */
} BuzzerState;

/* Buzzer context */
static struct {
    BuzzerState state;
    uint8_t beepsRemaining;
    uint16_t timeoutCounter;
    uint16_t beepDuration;
    uint16_t silenceDuration;
} buzzerCtx = {
    .state = BUZZER_IDLE,
    .beepsRemaining = 0,
    .timeoutCounter = 0,
    .beepDuration = BUZZER_BEEP_DURATION_MS / 10,  /* Assuming 10ms update period */
    .silenceDuration = 80 / 10                      /* 80ms silence between beeps */
};

/**
 * @brief Buzzer_Init
 */
void Buzzer_Init(void)
{
    /* PA0 is configured as output by STM32CubeMX */
    /* Initially turn off buzzer */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    
    /* Reset state machine */
    buzzerCtx.state = BUZZER_IDLE;
    buzzerCtx.beepsRemaining = 0;
    buzzerCtx.timeoutCounter = 0;
}

/**
 * @brief Buzzer_BeepPattern
 */
void Buzzer_BeepPattern(uint8_t beepCount)
{
    /* Start new beep pattern */
    if (beepCount == 0) return;
    
    buzzerCtx.beepsRemaining = beepCount;
    buzzerCtx.state = BUZZER_BEEPING;
    buzzerCtx.timeoutCounter = buzzerCtx.beepDuration;
    
    /* Turn on buzzer immediately */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
}

/**
 * @brief Buzzer_IsActive
 */
bool Buzzer_IsActive(void)
{
    return (buzzerCtx.state != BUZZER_IDLE);
}

/**
 * @brief Buzzer_Stop
 */
void Buzzer_Stop(void)
{
    buzzerCtx.state = BUZZER_IDLE;
    buzzerCtx.beepsRemaining = 0;
    buzzerCtx.timeoutCounter = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

/**
 * @brief Buzzer_Update
 * 
 * State machine for non-blocking beep pattern generation.
 * Call periodically (e.g., every 10 ms).
 */
void Buzzer_Update(void)
{
    if (buzzerCtx.state == BUZZER_IDLE) {
        return;  /* Nothing to do */
    }
    
    /* Decrement timeout counter */
    if (buzzerCtx.timeoutCounter > 0) {
        buzzerCtx.timeoutCounter--;
        return;  /* Still waiting */
    }
    
    /* Timeout expired, transition to next state */
    if (buzzerCtx.state == BUZZER_BEEPING) {
        /* Finish beep, go to silence */
        if (buzzerCtx.beepsRemaining > 1) {
            /* More beeps to come */
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);  /* Turn off */
            buzzerCtx.state = BUZZER_SILENT;
            buzzerCtx.timeoutCounter = buzzerCtx.silenceDuration;
            buzzerCtx.beepsRemaining--;
        } else {
            /* Last beep finished */
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);  /* Turn off */
            buzzerCtx.state = BUZZER_IDLE;
            buzzerCtx.beepsRemaining = 0;
        }
    } else if (buzzerCtx.state == BUZZER_SILENT) {
        /* Silence done, start next beep */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);  /* Turn on */
        buzzerCtx.state = BUZZER_BEEPING;
        buzzerCtx.timeoutCounter = buzzerCtx.beepDuration;
    }
}
