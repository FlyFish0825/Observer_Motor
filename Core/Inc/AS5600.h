#ifndef __AS5600_H
#define __AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* AS5600 I2C 地址 */
#define AS5600_I2C_ADDR_7BIT      0x36U
#define AS5600_I2C_ADDR           (AS5600_I2C_ADDR_7BIT << 1)

/* AS5600 分辨率 */
#define AS5600_RESOLUTION         4096.0f
#define AS5600_TIMEOUT_MS         100U

/* AS5600 寄存器地址 */
#define AS5600_REG_ZMCO           0x00U
#define AS5600_REG_ZPOS_H         0x01U
#define AS5600_REG_ZPOS_L         0x02U
#define AS5600_REG_MPOS_H         0x03U
#define AS5600_REG_MPOS_L         0x04U
#define AS5600_REG_MANG_H         0x05U
#define AS5600_REG_MANG_L         0x06U
#define AS5600_REG_CONF_H         0x07U
#define AS5600_REG_CONF_L         0x08U

#define AS5600_REG_STATUS         0x0BU
#define AS5600_REG_RAW_ANGLE_H    0x0CU
#define AS5600_REG_RAW_ANGLE_L    0x0DU
#define AS5600_REG_ANGLE_H        0x0EU
#define AS5600_REG_ANGLE_L        0x0FU

#define AS5600_REG_AGC            0x1AU
#define AS5600_REG_MAGNITUDE_H    0x1BU
#define AS5600_REG_MAGNITUDE_L    0x1CU

/* STATUS 寄存器位 */
#define AS5600_STATUS_MD          0x20U   // 磁铁检测到
#define AS5600_STATUS_ML          0x10U   // 磁场太弱
#define AS5600_STATUS_MH          0x08U   // 磁场太强




HAL_StatusTypeDef AS5600_init(void);

HAL_StatusTypeDef AS5600_IsReady(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef AS5600_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef AS5600_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t len);

HAL_StatusTypeDef AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c, uint16_t *raw_angle);
HAL_StatusTypeDef AS5600_ReadAngle(I2C_HandleTypeDef *hi2c, uint16_t *angle);

HAL_StatusTypeDef AS5600_ReadRawAngleDeg(I2C_HandleTypeDef *hi2c, float *deg);
HAL_StatusTypeDef AS5600_ReadAngleDeg(I2C_HandleTypeDef *hi2c, float *deg);

HAL_StatusTypeDef AS5600_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status);
HAL_StatusTypeDef AS5600_ReadAGC(I2C_HandleTypeDef *hi2c, uint8_t *agc);
HAL_StatusTypeDef AS5600_ReadMagnitude(I2C_HandleTypeDef *hi2c, uint16_t *magnitude);

uint8_t AS5600_MagnetDetected(uint8_t status);
uint8_t AS5600_MagnetTooWeak(uint8_t status);
uint8_t AS5600_MagnetTooStrong(uint8_t status);

#ifdef __cplusplus
}
#endif

#endif