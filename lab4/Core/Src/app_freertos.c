/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "timers.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "motor.h"
#include "keypad.h"
#include "buzzer.h"
#include "controller.h"
#include "lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    uint16_t targetRPM;     /* Current target speed */
    uint16_t measuredRPM;   /* Measured motor speed */
} ControlLoop;

/* Task priorities (higher number = higher priority) */
#define MOTOR_CONTROL_TASK_PRIORITY     (configMAX_PRIORITIES - 2)  /* Highest */
#define ENCODER_TASK_PRIORITY           (configMAX_PRIORITIES - 2)  /* High */
#define UI_TASK_PRIORITY                (configMAX_PRIORITIES - 3)  /* Normal */

/* Task periods in milliseconds */
#define MOTOR_CONTROL_TASK_PERIOD_TICKS  pdMS_TO_TICKS(MOTOR_CONTROL_TASK_PERIOD_MS)
#define ENCODER_TASK_PERIOD_TICKS        pdMS_TO_TICKS(ENCODER_UPDATE_PERIOD_MS)
#define UI_TASK_PERIOD_TICKS             pdMS_TO_TICKS(UI_TASK_PERIOD_MS)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Task handles */
static TaskHandle_t motorControlTaskHandle = NULL;
static TaskHandle_t encoderTaskHandle = NULL;
static TaskHandle_t uiTaskHandle = NULL;

/**
 * @brief Motor Control Task (Highest Priority)
 * Responsible for applying closed-loop PWM control
 */
static void MotorControlTask(void *argument);

/**
 * @brief Encoder Measurement Task
 * Responsible for computing motor speed from encoder pulses
 */
static void EncoderTask(void *argument);

/**
 * @brief User Interface Task
 * Handles LCD display, keypad input, and buzzer feedback
 **
 * @brief SystemModulesInit
 * Initialize all hardware-related modules before task scheduler starts
 */
static void SystemModulesInit(void)
{
    printf("[LAB4] Initializing hardware modules...\r\n");
    
    /* Initialize encoder measurement */
    Encoder_Init();
    printf("[LAB4] Encoder module initialized\r\n");
    
    /* Initialize motor control */
    Motor_Init();
    printf("[LAB4] Motor module initialized\r\n");
    
    /* Initialize closed-loop controller */
    Controller_Init();
    printf("[LAB4] Controller module initialized\r\n");
    
    /* Initialize keypad/touch interface */
    Keypad_Init();
    printf("[LAB4] Keypad module initialized\r\n");
    
    /* Initialize non-blocking buzzer */
    Buzzer_Init();
    printf("[LAB4] Buzzer module initialized\r\n");
    
    /* Initialize LCD display */
    LCD_Init();
    LCD_DisplayText("LAB 4: DC MOTOR", FIRST_LINE);
    LCD_DisplayText("Speed Control", SECOND_LINE);
    printf("[LAB4] LCD initialized\r\n");
    
    /* Allow a short time for LCD to stabilize */
    vTaskDelay(pdMS_TO_TICKS(500));
    
    systemReady = true;
    printf("[LAB4] All modules initialized successfully\r\n");
}

/**
 * @brief UpdateLCDDisplay
 * Display current target and measured speed
 */
static void UpdateLCDDisplay(uint16_t targetRPM, uint16_t measuredRPM)
{
    static char line1[17];
    static char line2[17];
    
    /* Line 1: Target Speed */
    snprintf(line1, sizeof(line1), "Target: %3u RPM", targetRPM);
    LCD_DisplayText(line1, FIRST_LINE);
    
    /* Line 2: Measured Speed */
    snprintf(line2, sizeof(line2), "Measured: %3u RM", measuredRPM);
    LCD_DisplayText(line2, SECOND_LINE);
}

/**
 * @brief MotorControlTask
 * 
 * Priority: Highest
 * Period: 10 ms (100 Hz control loop)
 * 
 * Responsible for:
 * - Reading latest measured speed
 * - Applying closed-loop control law
 * - Updating PWM duty cycle
 * - Safety enforcement (limits, timeouts)
 */
static void MotorControlTask(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint16_t targetRPM = 0;
    uint16_t measuredRPM = 0;
    uint16_t computedPWM = 0;
    uint16_t safetyCounter = 0;
    
    printf("[MOTOR] Task started (Period: %u ms)\r\n", MOTOR_CONTROL_TASK_PERIOD_MS);
    
    while (1) {
        /* Wait for next period */
        vTaskDelayUntil(&xLastWakeTime, MOTOR_CONTROL_TASK_PERIOD_TICKS);
        motorTaskExecutionCount++;
        
        /* Read shared control state (protected) */
        {
            taskENTER_CRITICAL();
            targetRPM = controlLoop.targetRPM;
            measuredRPM = controlLoop.measuredRPM;
            taskEXIT_CRITICAL();
        }
        
        if (targetRPM == 0) {
            /* Stop command */
            Motor_EmergencyStop();
            Controller_SetTarget(0);
            safetyCounter = 0;
        } else {
            /* Running control loop */
            Controller_SetTarget(targetRPM);
            
            /* Compute PWM adjustment based on measured speed */
            computedPWM = Controller_Update(measuredRPM);
            
            /* Apply PWM: assume CW direction for Lab 4 */
            Motor_SetCommand(MOTOR_CW, computedPWM);
            
            /* Safety check: if motor runs too long without reaching target, increase PWM */
            safetyCounter++;
            if (safetyCounter > 100 && computedPWM < 80) {
                /* After ~1 second at low speed, boost PWM slightly */
                Motor_SetDutyCycle(computedPWM + 5);
            }
        }
    }
}

/**
 * @brief EncoderTask
 * 
 * Priority: High (but lower than motor control)
 * Period: 10 ms (same as encoder measurement window)
 * 
 * Responsible for:
 * - Processing encoder pulses
 * - Computing RPM
 * - Publishing speed estimate to control loop
 */
static void EncoderTask(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint16_t speedRPM = 0;
    
    printf("[ENCODER] Task started (Period: %u ms)\r\n", ENCODER_UPDATE_PERIOD_MS);
    
    while (1) {
        /* Wait for next period */
        vTaskDelayUntil(&xLastWakeTime, ENCODER_TASK_PERIOD_TICKS);
        encoderTaskExecutionCount++;
        
        /* Process encoder measurements */
        Encoder_Update();
        
        /* Get computed RPM */
        speedRPM = Encoder_GetSpeedRPM();
        
        /* Publish to control loop (critical section) */
        {
            taskENTER_CRITICAL();
            controlLoop.measuredRPM = speedRPM;
            taskEXIT_CRITICAL();
        }
    }
}

/**
 * @brief UITask
 * 
 * Priority: Normal (lower than control tasks)
 * Period: 50 ms
 * 
 * Responsible for:
 * - Reading keypad/keyboard input
 * - Processing preset selection
 * - Updating LCD display
 * - Triggering buzzer patterns
 */
static void UITask(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    KeypadPreset currentPreset = KEYPAD_PRESET_NONE;
    KeypadPreset lastPreset = KEYPAD_PRESET_NONE;
    uint16_t displayedTarget = 0;
    uint16_t displayedMeasured = 0;
    
    printf("[UI] Task started (Period: %u ms)\r\n", UI_TASK_PERIOD_MS);
    
    while (1) {
        /* Wait for next period */
        vTaskDelayUntil(&xLastWakeTime, UI_TASK_PERIOD_TICKS);
        uiTaskExecutionCount++;
        
        /* Process keypad input */
        Keypad_Update();
        
        /* Check for new preset selection */
        if (Keypad_GetNewPreset(&currentPreset) && currentPreset != lastPreset) {
            uint16_t targetRPM = Keypad_GetTargetRPM(currentPreset);
            uint8_t beepCount = Keypad_GetBeepCount(currentPreset);
            
            /* Update control loop target */
            {
                taskENTER_CRITICAL();
                controlLoop.targetRPM = targetRPM;
                taskEXIT_CRITICAL();
            }
            
            /* Trigger buzzer pattern */
            if (beepCount > 0) {
                Buzzer_BeepPattern(beepCount);
            }
            
            printf("[UI] Preset selected: %u RPM (beeps: %u)\r\n", targetRPM, beepCount);
            lastPreset = currentPreset;
        }
        
        /* Update buzzer state machine */
        Buzzer_Update();
        
        /* Update LCD display */
        {
            taskENTER_CRITICAL();
            displayedTarget = controlLoop.targetRPM;
            displayedMeasured = controlLoop.measuredRPM;
            taskEXIT_CRITICAL();
        }
        UpdateLCDDisplay(displayedTarget, displayedMeasured);
    }
}

/**
 * @brief StartDefaultTask
 * Called by CubeMX starter code to initialize FreeRTOS
 */
void StartDefaultTask(void *argument)
{
    /* Initialize all hardware modules */
    SystemModulesInit();
    
    /* Create motor control task (highest priority) */
    xTaskCreate(
        MotorControlTask,
        "MotorControl",
        256,
        NULL,
        MOTOR_CONTROL_TASK_PRIORITY,
        &motorControlTaskHandle
    );
    
    /* Create encoder measurement task */
    xTaskCreate(
        EncoderTask,
        "Encoder",
        256,
        NULL,
        ENCODER_TASK_PRIORITY,
        &encoderTaskHandle
    );
    
    /* Create UI task for keypad/LCD interaction */
    xTaskCreate(
        UITask,
        "UI",
        512,
        NULL,
        UI_TASK_PRIORITY,
        &uiTaskHandle
    );
    
    printf("[FREERTOS] All tasks created and running\r\n");
    printf("[LAB4] System ready for operation\r\n");
    
    /* This task can now be deleted or used for monitoring */
    vTaskDelete(NULL);
}

/*/
static void UITask(void *argument);

/**
 * @brief Initialize all hardware modules (called before task creation)
 */
static void SystemModulesInit(void);

/**
 * @brief Update display with current speed and target
 */
static void UpdateLCDDisplay(uint16_t targetRPM, uint16_t measuredRPM);

/* Shared control loop state (protected with critical section) */
static ControlLoop controlLoop = {
    .targetRPM = 0,
    .measuredRPM = 0
};

/* Timing analysis variables for Lab 4 timing report */
static uint32_t motorTaskExecutionCount = 0;
static uint32_t encoderTaskExecutionCount = 0;
static uint32_t uiTaskExecutionCount = 0;

/* Flag to indicate initial mode (can be used for startup diagnostics) */
static bool systemReady = false;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

