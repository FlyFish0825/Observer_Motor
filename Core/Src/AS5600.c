/**
 * @file AS5600.c
 * @brief AS5600 磁编码器的 I2C 读写和状态查询实现。
 *
 * 本文件是用户维护代码，封装寄存器访问、角度换算及磁场状态判断。
 * I2C 外设句柄由 CubeMX 生成的 i2c.c 提供，本文件不修改其初始化内容。
 */
#include "as5600.h"
#include "i2c.h"
#include <stdio.h>






/**
  * @brief 初始化
  */
HAL_StatusTypeDef AS5600_init(void)
{
    /* 用于保存传感器 STATUS 寄存器的当前值。 */
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
    /* 通过 HAL 的设备探测接口发送地址，确认传感器在总线上响应。 */
    return HAL_I2C_IsDeviceReady(hi2c, AS5600_I2C_ADDR, 3, AS5600_TIMEOUT_MS);
}

/**
  * @brief  读取单个寄存器
  */
HAL_StatusTypeDef AS5600_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data)
{
    /* 输出地址为空时不能写入结果，直接返回参数错误。 */
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
    /* 连续读取必须同时提供有效缓冲区和非零长度。 */
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
    /* RAW_ANGLE 高、低寄存器的原始字节。 */
    uint8_t buf[2];

    if (raw_angle == NULL)
    {
        return HAL_ERROR;
    }

    if (AS5600_ReadRegs(hi2c, AS5600_REG_RAW_ANGLE_H, buf, 2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 高字节只有低4位有效，与低字节合成12位角度码。 */
    *raw_angle = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];

    return HAL_OK;
}

/**
  * @brief  读取 ANGLE 滤波/缩放后的角度
  * @note   范围：0 ~ 4095
  */
HAL_StatusTypeDef AS5600_ReadAngle(I2C_HandleTypeDef *hi2c, uint16_t *angle)
{
    /* ANGLE 高、低寄存器的滤波角度字节。 */
    uint8_t buf[2];

    if (angle == NULL)
    {
        return HAL_ERROR;
    }

    if (AS5600_ReadRegs(hi2c, AS5600_REG_ANGLE_H, buf, 2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 高字节低4位和低字节组成传感器输出的12位角度值。 */
    *angle = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];

    return HAL_OK;
}

/**
  * @brief  读取 RAW_ANGLE 并转换为角度制
  * @note   范围：0.0 ~ 360.0 度
  */
HAL_StatusTypeDef AS5600_ReadRawAngleDeg(I2C_HandleTypeDef *hi2c, float *deg)
{
    /* 传感器返回的12位原始角度码。 */
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
    /* 传感器返回的滤波/缩放后12位角度码。 */
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
    /* STATUS 中的 MD、ML、MH 位分别表示磁铁检测及磁场强弱。 */
    return AS5600_ReadReg(hi2c, AS5600_REG_STATUS, status);
}

/**
  * @brief  读取 AGC 自动增益值
  */
HAL_StatusTypeDef AS5600_ReadAGC(I2C_HandleTypeDef *hi2c, uint8_t *agc)
{
    /* AGC 值用于观察芯片内部自动增益状态，不在此处进行闭环调整。 */
    return AS5600_ReadReg(hi2c, AS5600_REG_AGC, agc);
}

/**
  * @brief  读取磁场幅值 MAGNITUDE
  */
HAL_StatusTypeDef AS5600_ReadMagnitude(I2C_HandleTypeDef *hi2c, uint16_t *magnitude)
{
    /* MAGNITUDE 高、低寄存器的原始字节。 */
    uint8_t buf[2];

    if (magnitude == NULL)
    {
        return HAL_ERROR;
    }

    if (AS5600_ReadRegs(hi2c, AS5600_REG_MAGNITUDE_H, buf, 2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 幅值寄存器同样为12位有效数据。 */
    *magnitude = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];

    return HAL_OK;
}

/**
  * @brief  是否检测到磁铁
  */
uint8_t AS5600_MagnetDetected(uint8_t status)
{
    /* MD 位置1表示检测到了满足条件的磁场。 */
    return (status & AS5600_STATUS_MD) ? 1U : 0U;
}

/**
  * @brief  磁场是否太弱
  */
uint8_t AS5600_MagnetTooWeak(uint8_t status)
{
    /* ML 位置1表示磁场幅值低于传感器建议范围。 */
    return (status & AS5600_STATUS_ML) ? 1U : 0U;
}

/**
  * @brief  磁场是否太强
  */
uint8_t AS5600_MagnetTooStrong(uint8_t status)
{
    /* MH 位置1表示磁场幅值高于传感器建议范围。 */
    return (status & AS5600_STATUS_MH) ? 1U : 0U;
}
