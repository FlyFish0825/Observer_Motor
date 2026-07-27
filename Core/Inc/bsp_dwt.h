#ifndef __BSP_DWT_H
#define __BSP_DWT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/**
 * @brief  初始化 DWT 周期计数器
 * @retval 1：成功，0：失败
 */
uint8_t DWT_Delay_Init(void);

/**
 * @brief  获取当前 CPU 周期计数
 */
uint32_t DWT_GetCycle(void);

/**
 * @brief  计算从 start_cycle 到当前经过的周期数
 */
uint32_t DWT_ElapsedCycle(uint32_t start_cycle);

/**
 * @brief  计算从 start_cycle 到当前经过的时间，单位 us
 */
uint32_t DWT_ElapsedUs(uint32_t start_cycle);

/**
 * @brief  计算从 start_cycle 到当前经过的时间，单位 ms
 */
uint32_t DWT_ElapsedMs(uint32_t start_cycle);

/**
 * @brief  延时指定 CPU 周期数
 */
void DWT_Delay_Cycle(uint32_t cycles);

/**
 * @brief  微秒级延时
 */
void DWT_Delay_Us(uint32_t us);

/**
 * @brief  毫秒级延时
 * @note   可以在中断里用，但不建议中断里做 ms 级阻塞延时
 */
void DWT_Delay_Ms(uint32_t ms);

/**
 * @brief  判断是否超时，单位 us
 */
uint8_t DWT_IsTimeoutUs(uint32_t start_cycle, uint32_t timeout_us);

/**
 * @brief  判断是否超时，单位 ms
 */
uint8_t DWT_IsTimeoutMs(uint32_t start_cycle, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif