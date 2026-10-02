#ifndef __BSP_DWT_H
#define __BSP_DWT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/**
 * @file bsp_dwt.h
 * @brief 基于Cortex-M DWT CYCCNT的高分辨率计时、延时与超时判断接口。
 * 只访问内核调试计数器，不依赖任何外设定时器，因此可在中断里使用。
 * 时间换算基准是SystemCoreClock，初始化时记录一次HCLK频率。
 * 用途包括测量25 kHz中断耗时与短时间忙等待。
 */

/**
 * @brief  初始化 DWT 周期计数器，记录当前HCLK频率供后续换算。
 * @retval 1：成功，0：失败（CYCCNT未使能，可能是调试器占用或内核不支持）
 */
uint8_t DWT_Delay_Init(void);

/**
 * @brief  获取当前 CPU 周期计数
 * @return CYCCNT当前32位计数值，单位CPU周期，会自然回绕
 */
uint32_t DWT_GetCycle(void);

/**
 * @brief  计算从 start_cycle 到当前经过的周期数
 * @param  start_cycle 起始时刻保存的CYCCNT值
 * @return 经过的CPU周期数，单位周期；利用uint32_t自然溢出正确处理回绕
 */
uint32_t DWT_ElapsedCycle(uint32_t start_cycle);

/**
 * @brief  计算从 start_cycle 到当前经过的时间，单位 us
 * @param  start_cycle 起始时刻保存的CYCCNT值
 * @return 经过时间，单位微秒；内部用64位中间量避免乘法溢出
 */
uint32_t DWT_ElapsedUs(uint32_t start_cycle);

/**
 * @brief  计算从 start_cycle 到当前经过的时间，单位 ms
 * @param  start_cycle 起始时刻保存的CYCCNT值
 * @return 经过时间，单位毫秒
 */
uint32_t DWT_ElapsedMs(uint32_t start_cycle);

/**
 * @brief  延时指定 CPU 周期数
 * @param  cycles 要等待的CPU周期数
 * @note   忙等待，不进入低功耗；不允许在25 kHz中断里等待过长时间
 */
void DWT_Delay_Cycle(uint32_t cycles);

/**
 * @brief  微秒级延时
 * @param  us 要等待的时间，单位微秒
 */
void DWT_Delay_Us(uint32_t us);

/**
 * @brief  毫秒级延时
 * @param  ms 要等待的时间，单位毫秒
 * @note   可以在中断里用，但不建议中断里做 ms 级阻塞延时
 */
void DWT_Delay_Ms(uint32_t ms);

/**
 * @brief  判断是否超时，单位 us
 * @param  start_cycle 起始时刻保存的CYCCNT值
 * @param  timeout_us  超时门限，单位微秒
 * @return 1：已超时；0：尚未超时
 */
uint8_t DWT_IsTimeoutUs(uint32_t start_cycle, uint32_t timeout_us);

/**
 * @brief  判断是否超时，单位 ms
 * @param  start_cycle 起始时刻保存的CYCCNT值
 * @param  timeout_ms  超时门限，单位毫秒
 * @return 1：已超时；0：尚未超时
 */
uint8_t DWT_IsTimeoutMs(uint32_t start_cycle, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif