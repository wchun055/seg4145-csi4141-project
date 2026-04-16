/**
  * @file keypad.c
  * @brief Keypad module implementation
  */

#include "keypad.h"
#include "main.h"
#include <stdbool.h>

#define TOUCH_EN_GPIO_Port GPIOB
#define TOUCH_EN_Pin       GPIO_PIN_5
#define TOUCH_INT_GPIO_Port GPIOB
#define TOUCH_INT_Pin      GPIO_PIN_6

#define TOUCH_AT42QT1070_ADDR         (0x1BU << 1)
#define TOUCH_AT42QT1070_CHIP_ID_REG  0x00U
#define TOUCH_AT42QT1070_KEY_REG      0x03U
#define TOUCH_AT42QT1070_CHIP_ID      0x2EU

#define TOUCH_MPR121_TOUCH_STATUS_REG 0x00U
#define TOUCH_MPR121_SOFTRESET_REG    0x80U
#define TOUCH_MPR121_ECR_REG          0x5EU
#define TOUCH_MPR121_DEBOUNCE_REG     0x5BU
#define TOUCH_MPR121_CONFIG1_REG      0x5CU
#define TOUCH_MPR121_CONFIG2_REG      0x5DU
#define TOUCH_MPR121_THRESHOLD_BASE   0x41U
#define TOUCH_MPR121_SOFTRESET_CMD    0x63U
#define TOUCH_MPR121_ECR_RUN_12ELE    0x8CU

#define TOUCH_MPR121_ADDR_0           (0x5AU << 1)
#define TOUCH_MPR121_ADDR_1           (0x5BU << 1)
#define TOUCH_MPR121_ADDR_2           (0x5CU << 1)
#define TOUCH_MPR121_ADDR_3           (0x5DU << 1)

extern I2C_HandleTypeDef hi2c2;

typedef enum {
    TOUCH_PROTO_NONE = 0,
    TOUCH_PROTO_AT42QT1070,
    TOUCH_PROTO_MPR121
} TouchProtocol;

static TouchProtocol touchProtocol = TOUCH_PROTO_NONE;
static uint16_t touchI2cAddr = 0U;
static bool touchEnableActiveHigh = true;
static uint16_t touchLastMask = 0xFFFFU;

void Keypad_Enable(bool enable)
{
  GPIO_PinState asserted = touchEnableActiveHigh ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState deasserted = touchEnableActiveHigh ? GPIO_PIN_RESET : GPIO_PIN_SET;
  HAL_GPIO_WritePin(TOUCH_EN_GPIO_Port, TOUCH_EN_Pin, enable ? asserted : deasserted);
}

static bool Touch_Mpr121Init(uint16_t addr)
{
  if (HAL_I2C_Mem_Write(&hi2c2, addr, TOUCH_MPR121_SOFTRESET_REG, I2C_MEMADD_SIZE_8BIT,
                        (uint8_t[]){TOUCH_MPR121_SOFTRESET_CMD}, 1, 20) != HAL_OK) {
    return false;
  }

  if (HAL_I2C_Mem_Write(&hi2c2, addr, TOUCH_MPR121_ECR_REG, I2C_MEMADD_SIZE_8BIT,
                        (uint8_t[]){0x00U}, 1, 20) != HAL_OK) {
    return false;
  }

  for (uint8_t electrode = 0; electrode < 12; electrode++) {
    uint8_t regs[2] = { 12U, 6U };
    uint8_t reg = (uint8_t)(TOUCH_MPR121_THRESHOLD_BASE + electrode * 2U);
    if (HAL_I2C_Mem_Write(&hi2c2, addr, reg, I2C_MEMADD_SIZE_8BIT, regs, 2, 20) != HAL_OK) {
      return false;
    }
  }

  if (HAL_I2C_Mem_Write(&hi2c2, addr, TOUCH_MPR121_DEBOUNCE_REG, I2C_MEMADD_SIZE_8BIT,
                        (uint8_t[]){0x11U}, 1, 20) != HAL_OK) {
    return false;
  }
  if (HAL_I2C_Mem_Write(&hi2c2, addr, TOUCH_MPR121_CONFIG1_REG, I2C_MEMADD_SIZE_8BIT,
                        (uint8_t[]){0x10U}, 1, 20) != HAL_OK) {
    return false;
  }
  if (HAL_I2C_Mem_Write(&hi2c2, addr, TOUCH_MPR121_CONFIG2_REG, I2C_MEMADD_SIZE_8BIT,
                        (uint8_t[]){0x20U}, 1, 20) != HAL_OK) {
    return false;
  }
  if (HAL_I2C_Mem_Write(&hi2c2, addr, TOUCH_MPR121_ECR_REG, I2C_MEMADD_SIZE_8BIT,
                        (uint8_t[]){TOUCH_MPR121_ECR_RUN_12ELE}, 1, 20) != HAL_OK) {
    return false;
  }

  return true;
}

static bool Touch_TryAutodetect(void)
{
  static const uint16_t mprAddrs[] = {
      TOUCH_MPR121_ADDR_0, TOUCH_MPR121_ADDR_1, TOUCH_MPR121_ADDR_2, TOUCH_MPR121_ADDR_3
  };
  uint8_t data[2] = {0};

  if (HAL_I2C_Mem_Read(&hi2c2, TOUCH_AT42QT1070_ADDR, TOUCH_AT42QT1070_CHIP_ID_REG,
                       I2C_MEMADD_SIZE_8BIT, data, 1, 20) == HAL_OK &&
      data[0] == TOUCH_AT42QT1070_CHIP_ID) {
    touchProtocol = TOUCH_PROTO_AT42QT1070;
    touchI2cAddr = TOUCH_AT42QT1070_ADDR;
    touchLastMask = 0xFFFFU;
    printf("[TOUCH] AT42QT1070 detected @ 0x%02X\r\n", TOUCH_AT42QT1070_ADDR >> 1);
    return true;
  }

  for (uint32_t i = 0; i < (sizeof(mprAddrs) / sizeof(mprAddrs[0])); i++) {
    uint16_t addr = mprAddrs[i];
    if (HAL_I2C_Mem_Read(&hi2c2, addr, TOUCH_MPR121_TOUCH_STATUS_REG,
                         I2C_MEMADD_SIZE_8BIT, data, 2, 20) == HAL_OK) {
      touchProtocol = TOUCH_PROTO_MPR121;
      touchI2cAddr = addr;
      touchLastMask = 0xFFFFU;
      printf("[TOUCH] MPR121-like controller detected @ 0x%02X\r\n", addr >> 1);
      if (Touch_Mpr121Init(addr)) {
        printf("[TOUCH] MPR121 initialized\r\n");
      } else {
        printf("[TOUCH] MPR121 init failed (using raw reads only)\r\n");
      }
      return true;
    }
  }

  touchProtocol = TOUCH_PROTO_NONE;
  touchI2cAddr = 0U;
  touchLastMask = 0xFFFFU;
  printf("[TOUCH] controller not detected (tried AT42 0x%02X, MPR121 0x5A-0x5D)\r\n",
         TOUCH_AT42QT1070_ADDR >> 1);
  return false;
}

bool Keypad_Init(void)
{
  touchEnableActiveHigh = true;
  Keypad_Enable(true);
  HAL_Delay(30);
  return Touch_TryAutodetect();
}

bool Keypad_ReadCommandState(MotorState *stateOut)
{
  uint8_t data[2] = {0};
  uint16_t mask = 0;

  if (stateOut == NULL) {
    return false;
  }

  if (touchProtocol == TOUCH_PROTO_NONE && !Touch_TryAutodetect()) {
    return false;
  }

  if (touchProtocol == TOUCH_PROTO_AT42QT1070) {
    if (HAL_I2C_Mem_Read(&hi2c2, touchI2cAddr, TOUCH_AT42QT1070_KEY_REG,
                         I2C_MEMADD_SIZE_8BIT, data, 1, 20) != HAL_OK) {
      return false;
    }
    mask = data[0];
  } else if (touchProtocol == TOUCH_PROTO_MPR121) {
    if (HAL_I2C_Mem_Read(&hi2c2, touchI2cAddr, TOUCH_MPR121_TOUCH_STATUS_REG,
                         I2C_MEMADD_SIZE_8BIT, data, 2, 20) != HAL_OK) {
      return false;
    }
    mask = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
  } else {
    return false;
  }

  if (mask != touchLastMask) {
    printf("[TOUCH] raw mask=0x%04X proto=%d addr=0x%02X\r\n",
           mask, (int)touchProtocol, (unsigned int)(touchI2cAddr >> 1));
    touchLastMask = mask;
  }

  if (mask & (1U << 0)) {
    *stateOut = STOP;
    return true;
  }
  if (mask & (1U << 2)) {
    *stateOut = CCW;
    return true;
  }
  if (mask & (1U << 1)) {
    *stateOut = CW;
    return true;
  }
  if (mask & (1U << 3)) {
    *stateOut = EMERGENCY;
    return true;
  }

  return false;
}