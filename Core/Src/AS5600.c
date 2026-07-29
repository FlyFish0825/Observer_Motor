#include "as5600.h"
#include "i2c.h"
#include <stdio.h>






/**
  * @brief 初始化
  */
HAL_StatusTypeDef AS5600_init(void)
{
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
  * @brief  检查 AS5600 是否在线
  */
HAL_StatusTypeDef AS5600_IsReady(I2C_HandleTypeDef *hi2c)
{
    return HAL_I2C_IsDeviceReady(hi2c, AS5600_I2C_ADDR, 3, AS5600_TIMEOUT_MS);
}

/**
  * @brief  读取单个寄存器
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
  * @brief  连续读取多个寄存器
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
    uint8_t buf[2];

    if (raw_angle == NULL)
    {
        return HAL_ERROR;
    }

    if (AS5600_ReadRegs(hi2c, AS5600_REG_RAW_ANGLE_H, buf, 2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *raw_angle = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];

    return HAL_OK;
}

/**
  * @brief  读取 ANGLE 滤波/缩放后的角度
  * @note   范围：0 ~ 4095
  */
HAL_StatusTypeDef AS5600_ReadAngle(I2C_HandleTypeDef *hi2c, uint16_t *angle)
{
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
  * @brief  读取 STATUS 寄存器
  */
HAL_StatusTypeDef AS5600_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status)
{
    return AS5600_ReadReg(hi2c, AS5600_REG_STATUS, status);
}

/**
  * @brief  读取 AGC 自动增益值
  */
HAL_StatusTypeDef AS5600_ReadAGC(I2C_HandleTypeDef *hi2c, uint8_t *agc)
{
    return AS5600_ReadReg(hi2c, AS5600_REG_AGC, agc);
}

/**
  * @brief  读取磁场幅值 MAGNITUDE
  */
HAL_StatusTypeDef AS5600_ReadMagnitude(I2C_HandleTypeDef *hi2c, uint16_t *magnitude)
{
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
  * @brief  是否检测到磁铁
  */
uint8_t AS5600_MagnetDetected(uint8_t status)
{
    return (status & AS5600_STATUS_MD) ? 1U : 0U;
}

/**
  * @brief  磁场是否太弱
  */
uint8_t AS5600_MagnetTooWeak(uint8_t status)
{
    return (status & AS5600_STATUS_ML) ? 1U : 0U;
}

/**
  * @brief  磁场是否太强
  */
uint8_t AS5600_MagnetTooStrong(uint8_t status)
{
    return (status & AS5600_STATUS_MH) ? 1U : 0U;
}