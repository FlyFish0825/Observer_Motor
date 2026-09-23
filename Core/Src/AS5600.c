#include "as5600.h"
#include "i2c.h"
#include <stdio.h>

/* 本驱动通过 HAL 访问 AS5600；角度、状态、AGC 和磁场幅值均为只读采样数据。 */
/**
  * @brief 初始化并输出一次传感器在线、磁铁和磁场状态。
  * @note  当前工程固定使用 hi2c1；失败只报告 HAL 错误，不修改传感器配置。
  */
HAL_StatusTypeDef AS5600_init(void)
{
    /* STATUS 是本次诊断读取的原始 8 位状态数据。 */
    uint8_t status = 0U;

    if (AS5600_IsReady(&hi2c1) != HAL_OK)
    {
        printf("AS5600 not found\r\n");
        return HAL_ERROR;
    }

    printf("AS5600 OK\r\n");

    if (AS5600_ReadStatus(&hi2c1, &status) == HAL_OK)
    {
        printf("AS5600 STATUS: 0x%02X\r\n", status);

        if (AS5600_MagnetDetected(status))
        {
            printf("Magnet detected\r\n");
        }
        else
        {
            printf("Magnet not detected\r\n");
        }

        if (AS5600_MagnetTooWeak(status))
        {
            printf("Magnet too weak\r\n");
        }

        if (AS5600_MagnetTooStrong(status))
        {
            printf("Magnet too strong\r\n");
        }
    }

    return HAL_OK;
}

/**
  * @brief  检查 AS5600 是否在线。
  * @param  hi2c  承载传感器的 I2C 外设句柄。
  */
HAL_StatusTypeDef AS5600_IsReady(I2C_HandleTypeDef *hi2c)
{
    return HAL_I2C_IsDeviceReady(hi2c, AS5600_I2C_ADDR, 3, AS5600_TIMEOUT_MS);
}

/**
  * @brief  读取单个 8 位寄存器。
  * @param  reg  AS5600 寄存器地址；@param data 输出字节。
  */
HAL_StatusTypeDef AS5600_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data)
{
    if (data == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(
        hi2c,
        AS5600_I2C_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        1,
        AS5600_TIMEOUT_MS
    );
}

/**
  * @brief  从起始地址连续读取多个 8 位寄存器。
  * @note   适用于角度和幅值这类高低字节连续排列的数据。
  */
HAL_StatusTypeDef AS5600_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(
        hi2c,
        AS5600_I2C_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        len,
        AS5600_TIMEOUT_MS
    );
}

/**
  * @brief  读取 RAW_ANGLE 原始角度
  * @note   范围：0 ~ 4095
  */
HAL_StatusTypeDef AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c, uint16_t *raw_angle)
{
    /* AS5600 角度寄存器为 12 位，buf[0] 的高 4 位被丢弃。 */
    uint8_t buf[2];

    if (raw_angle == NULL)
    {
        return HAL_ERROR;
    }

    if (AS5600_ReadRegs(hi2c, AS5600_REG_RAW_ANGLE_H, buf, 2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 高字节低 nibble 与低字节组合成 0..4095 的传感器角度码。 */
    *raw_angle = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];

    return HAL_OK;
}

/**
  * @brief  读取 ANGLE 滤波/缩放后的角度
  * @note   范围：0 ~ 4095
  */
HAL_StatusTypeDef AS5600_ReadAngle(I2C_HandleTypeDef *hi2c, uint16_t *angle)
{
    /* ANGLE 是芯片内部处理后的 12 位角度数据。 */
    uint8_t buf[2];

    if (angle == NULL)
    {
        return HAL_ERROR;
    }

    if (AS5600_ReadRegs(hi2c, AS5600_REG_ANGLE_H, buf, 2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *angle = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];

    return HAL_OK;
}

/**
  * @brief  读取 RAW_ANGLE 并转换为角度制
  * @note   范围：0.0 ~ 360.0 度
  */
HAL_StatusTypeDef AS5600_ReadRawAngleDeg(I2C_HandleTypeDef *hi2c, float *deg)
{
    /* raw 是未滤波角度码，输出 deg 为用户侧工程单位。 */
    uint16_t raw;

    if (deg == NULL)
    {
        return HAL_ERROR;
    }

    if (AS5600_ReadRawAngle(hi2c, &raw) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *deg = (float)raw * 360.0f / AS5600_RESOLUTION;

    return HAL_OK;
}

/**
  * @brief  读取 ANGLE 并转换为角度制
  * @note   范围：0.0 ~ 360.0 度
  */
HAL_StatusTypeDef AS5600_ReadAngleDeg(I2C_HandleTypeDef *hi2c, float *deg)
{
    /* angle 是芯片处理后的角度码，输出 deg 为用户侧工程单位。 */
    uint16_t angle;

    if (deg == NULL)
    {
        return HAL_ERROR;
    }

    if (AS5600_ReadAngle(hi2c, &angle) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *deg = (float)angle * 360.0f / AS5600_RESOLUTION;

    return HAL_OK;
}

/**
  * @brief  读取 STATUS 寄存器中的磁铁检测和磁场告警位。
  */
HAL_StatusTypeDef AS5600_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status)
{
    return AS5600_ReadReg(hi2c, AS5600_REG_STATUS, status);
}

/**
  * @brief  读取 AGC 自动增益值，作为磁场耦合强度诊断数据。
  */
HAL_StatusTypeDef AS5600_ReadAGC(I2C_HandleTypeDef *hi2c, uint8_t *agc)
{
    return AS5600_ReadReg(hi2c, AS5600_REG_AGC, agc);
}

/**
  * @brief  读取 MAGNITUDE 磁场幅值原始数据（12 位）。
  */
HAL_StatusTypeDef AS5600_ReadMagnitude(I2C_HandleTypeDef *hi2c, uint16_t *magnitude)
{
    /* 幅值与角度一样按两个连续寄存器返回 12 位数据。 */
    uint8_t buf[2];

    if (magnitude == NULL)
    {
        return HAL_ERROR;
    }

    if (AS5600_ReadRegs(hi2c, AS5600_REG_MAGNITUDE_H, buf, 2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *magnitude = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];

    return HAL_OK;
}

/**
  * @brief  根据 STATUS 的 MD 位判断磁铁是否检测到。
  */
uint8_t AS5600_MagnetDetected(uint8_t status)
{
    return (status & AS5600_STATUS_MD) ? 1U : 0U;
}

/**
  * @brief  根据 STATUS 的 ML 位判断磁场是否过弱。
  */
uint8_t AS5600_MagnetTooWeak(uint8_t status)
{
    return (status & AS5600_STATUS_ML) ? 1U : 0U;
}

/**
  * @brief  根据 STATUS 的 MH 位判断磁场是否过强。
  */
uint8_t AS5600_MagnetTooStrong(uint8_t status)
{
    return (status & AS5600_STATUS_MH) ? 1U : 0U;
}
