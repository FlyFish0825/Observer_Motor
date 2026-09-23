/* AS5600 磁编码器的 I2C 寄存器访问接口；本文件只描述用户层驱动契约。 */
#ifndef __AS5600_H
#define __AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* 7 位从机地址；HAL_I2C_* 接口使用左移后的 8 位地址。 */
#define AS5600_I2C_ADDR_7BIT      0x36U
#define AS5600_I2C_ADDR           (AS5600_I2C_ADDR_7BIT << 1)

/* 12 位角度数据的满量程及 I2C 操作超时时间。 */
#define AS5600_RESOLUTION         4096.0f
#define AS5600_TIMEOUT_MS         100U

/* 配置寄存器：高低字节分别占用相邻寄存器地址。 */
#define AS5600_REG_ZMCO           0x00U
#define AS5600_REG_ZPOS_H         0x01U
#define AS5600_REG_ZPOS_L         0x02U
#define AS5600_REG_MPOS_H         0x03U
#define AS5600_REG_MPOS_L         0x04U
#define AS5600_REG_MANG_H         0x05U
#define AS5600_REG_MANG_L         0x06U
#define AS5600_REG_CONF_H         0x07U
#define AS5600_REG_CONF_L         0x08U

/* 运行状态与角度数据寄存器。角度高字节只有低 4 位有效。 */
#define AS5600_REG_STATUS         0x0BU
#define AS5600_REG_RAW_ANGLE_H    0x0CU
#define AS5600_REG_RAW_ANGLE_L    0x0DU
#define AS5600_REG_ANGLE_H        0x0EU
#define AS5600_REG_ANGLE_L        0x0FU

/* 磁场增益和幅值监测寄存器；幅值同样是 12 位高低字节数据。 */
#define AS5600_REG_AGC            0x1AU
#define AS5600_REG_MAGNITUDE_H    0x1BU
#define AS5600_REG_MAGNITUDE_L    0x1CU

/* STATUS 寄存器位：MD 表示检测到磁铁，ML/MH 表示磁场过弱/过强。 */
#define AS5600_STATUS_MD          0x20U   // 磁铁检测到
#define AS5600_STATUS_ML          0x10U   // 磁场太弱
#define AS5600_STATUS_MH          0x08U   // 磁场太强




/* 使用工程默认 I2C1 探测器件并打印状态摘要。 */
HAL_StatusTypeDef AS5600_init(void);

/* 检查指定 I2C 总线上的 AS5600 是否响应。 */
HAL_StatusTypeDef AS5600_IsReady(I2C_HandleTypeDef *hi2c);

/* 读取一个或连续多个 8 位寄存器，data 由调用者提供存储空间。 */
HAL_StatusTypeDef AS5600_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef AS5600_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t len);

/* 返回 0..4095 的原始角度或内部处理后的角度码。 */
HAL_StatusTypeDef AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c, uint16_t *raw_angle);
HAL_StatusTypeDef AS5600_ReadAngle(I2C_HandleTypeDef *hi2c, uint16_t *angle);

/* 将 12 位角度码按 4096 份换算为 0..360 度浮点值。 */
HAL_StatusTypeDef AS5600_ReadRawAngleDeg(I2C_HandleTypeDef *hi2c, float *deg);
HAL_StatusTypeDef AS5600_ReadAngleDeg(I2C_HandleTypeDef *hi2c, float *deg);

/* 读取状态、自动增益和磁场幅值等传感器诊断数据。 */
HAL_StatusTypeDef AS5600_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status);
HAL_StatusTypeDef AS5600_ReadAGC(I2C_HandleTypeDef *hi2c, uint8_t *agc);
HAL_StatusTypeDef AS5600_ReadMagnitude(I2C_HandleTypeDef *hi2c, uint16_t *magnitude);

/* 从 STATUS 数据中提取磁铁检测和磁场强度告警。 */
uint8_t AS5600_MagnetDetected(uint8_t status);
uint8_t AS5600_MagnetTooWeak(uint8_t status);
uint8_t AS5600_MagnetTooStrong(uint8_t status);

#ifdef __cplusplus
}
#endif

#endif
