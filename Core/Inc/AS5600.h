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




/**
 * @brief 初始化AS5600：检测传感器在线状态并打印磁场诊断信息。
 * @return HAL_OK传感器在线；HAL_ERROR传感器未响应（I2C无ACK）。
 * @note 使用hi2c1句柄，内部通过printf输出诊断文本。
 */
HAL_StatusTypeDef AS5600_init(void);

/**
 * @brief 通过I2C设备探测检查AS5600是否在线。
 * @param hi2c I2C句柄指针（CubeMX生成的hi2c1）。
 * @return HAL_OK设备响应；HAL_ERROR超时未响应。
 */
HAL_StatusTypeDef AS5600_IsReady(I2C_HandleTypeDef *hi2c);

/**
 * @brief 读取单个寄存器。
 * @param hi2c  I2C句柄指针。
 * @param reg   寄存器地址（见AS5600_REG_*宏）。
 * @param data  [out] 接收1字节读取结果的缓冲区。
 * @return HAL_OK读取成功；HAL_ERROR参数无效或I2C失败。
 */
HAL_StatusTypeDef AS5600_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data);

/**
 * @brief 从指定寄存器起始地址连续读取多个字节。
 * @param hi2c  I2C句柄指针。
 * @param reg   起始寄存器地址。
 * @param data  [out] 接收数据的缓冲区。
 * @param len   读取字节数。
 * @return HAL_OK读取成功；HAL_ERROR参数无效或I2C失败。
 */
HAL_StatusTypeDef AS5600_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t len);

/**
 * @brief 读取RAW_ANGLE原始12位角度（未经滤波）。
 * @param hi2c     I2C句柄指针。
 * @param raw_angle [out] 接收0~4095范围的角度码。
 * @return HAL_OK读取成功；HAL_ERROR参数无效或I2C失败。
 */
HAL_StatusTypeDef AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c, uint16_t *raw_angle);

/**
 * @brief 读取ANGLE滤波/缩放后的12位角度。
 * @param hi2c  I2C句柄指针。
 * @param angle [out] 接收0~4095范围的角度码。
 * @return HAL_OK读取成功；HAL_ERROR参数无效或I2C失败。
 */
HAL_StatusTypeDef AS5600_ReadAngle(I2C_HandleTypeDef *hi2c, uint16_t *angle);

/**
 * @brief 读取RAW_ANGLE并转换为角度制。
 * @param hi2c I2C句柄指针。
 * @param deg  [out] 接收0.0~360.0度。
 * @return HAL_OK读取成功；HAL_ERROR参数无效或I2C失败。
 */
HAL_StatusTypeDef AS5600_ReadRawAngleDeg(I2C_HandleTypeDef *hi2c, float *deg);

/**
 * @brief 读取ANGLE并转换为角度制。
 * @param hi2c I2C句柄指针。
 * @param deg  [out] 接收0.0~360.0度。
 * @return HAL_OK读取成功；HAL_ERROR参数无效或I2C失败。
 */
HAL_StatusTypeDef AS5600_ReadAngleDeg(I2C_HandleTypeDef *hi2c, float *deg);

/**
 * @brief 读取STATUS寄存器（含磁铁检测和磁场强弱标志）。
 * @param hi2c   I2C句柄指针。
 * @param status [out] 接收STATUS字节。
 * @return HAL_OK读取成功；HAL_ERROR失败。
 */
HAL_StatusTypeDef AS5600_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status);

/**
 * @brief 读取AGC自动增益值，用于观察芯片内部增益状态。
 * @param hi2c I2C句柄指针。
 * @param agc  [out] 接收AGC字节。
 * @return HAL_OK读取成功；HAL_ERROR失败。
 */
HAL_StatusTypeDef AS5600_ReadAGC(I2C_HandleTypeDef *hi2c, uint8_t *agc);

/**
 * @brief 读取MAGNITUDE磁场幅值（12位）。
 * @param hi2c     I2C句柄指针。
 * @param magnitude [out] 接收0~4095范围的幅值。
 * @return HAL_OK读取成功；HAL_ERROR参数无效或I2C失败。
 */
HAL_StatusTypeDef AS5600_ReadMagnitude(I2C_HandleTypeDef *hi2c, uint16_t *magnitude);

/**
 * @brief 判断STATUS寄存器是否检测到磁铁。
 * @param status STATUS寄存器值。
 * @return 1检测到；0未检测到。
 */
uint8_t AS5600_MagnetDetected(uint8_t status);

/**
 * @brief 判断STATUS寄存器是否提示磁场太弱。
 * @param status STATUS寄存器值。
 * @return 1磁场太弱；0正常。
 */
uint8_t AS5600_MagnetTooWeak(uint8_t status);

/**
 * @brief 判断STATUS寄存器是否提示磁场太强。
 * @param status STATUS寄存器值。
 * @return 1磁场太强；0正常。
 */
uint8_t AS5600_MagnetTooStrong(uint8_t status);

#ifdef __cplusplus
}
#endif

#endif