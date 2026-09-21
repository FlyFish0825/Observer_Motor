/**
 * @file bsp_dwt.c
 * @brief 基于 Cortex-M DWT CYCCNT 的高分辨率计时与忙等待实现。
 * 本文件仅访问内核调试计数器，不修改 CubeMX 生成的时钟或外设初始化。
 */
#include "bsp_dwt.h"

/* DWT 周期计数器对应的 HCLK 频率，单位为Hz。 */
static uint32_t dwt_cpu_freq_hz = 0;

/**
 * @brief 将微秒转换为 CPU 周期数。
 * @param us 目标时间，单位为微秒。
 * @return 按当前 HCLK 频率换算得到的周期数。
 */
static uint32_t DWT_UsToCycle(uint32_t us)
{
    return (uint32_t)(((uint64_t)dwt_cpu_freq_hz * us) / 1000000ULL);
}

/**
 * @brief 将毫秒转换为 CPU 周期数。
 * @param ms 目标时间，单位为毫秒。
 * @return 按当前 HCLK 频率换算得到的周期数。
 */
static uint32_t DWT_MsToCycle(uint32_t ms)
{
    return (uint32_t)(((uint64_t)dwt_cpu_freq_hz * ms) / 1000ULL);
}

/**
 * @brief 初始化 DWT CYCCNT 周期计数器。
 * @return 1 表示计数器成功使能，0 表示硬件未能打开计数。
 */
uint8_t DWT_Delay_Init(void)
{
    /* 记录当前 HCLK，后续时间换算都以该频率为基准。 */
    dwt_cpu_freq_hz = HAL_RCC_GetHCLKFreq();

    /*
     * 使能 DWT 外设
     * CoreDebug->DEMCR 的 TRCENA 位必须打开
     */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /*
     * 清零周期计数器
     */
    DWT->CYCCNT = 0;

    /*
     * 使能 CYCCNT 计数
     */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /*
     * 判断是否使能成功
     */
    if (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief 读取当前 DWT 周期计数器值。
 * @return CYCCNT 当前32位计数值。
 */
uint32_t DWT_GetCycle(void)
{
    return DWT->CYCCNT;
}

/**
 * @brief 计算从起始计数值到当前时刻经过的周期数。
 * @param start_cycle 起始时刻保存的 CYCCNT 值。
 * @return 无符号回绕语义下的经过周期数。
 */
uint32_t DWT_ElapsedCycle(uint32_t start_cycle)
{
    /*
     * uint32_t 自然溢出特性：
     * 即使 CYCCNT 回绕，只要单次测量时间不超过一次回绕周期，也能算对。
     */
    return DWT->CYCCNT - start_cycle;
}

/**
 * @brief 计算从起始计数值到当前时刻经过的微秒数。
 * @param start_cycle 起始时刻保存的 CYCCNT 值。
 * @return 经过时间，单位为微秒。
 */
uint32_t DWT_ElapsedUs(uint32_t start_cycle)
{
    /* 已经过的周期数。 */
    uint32_t elapsed_cycle = DWT_ElapsedCycle(start_cycle);

    return (uint32_t)(((uint64_t)elapsed_cycle * 1000000ULL) / dwt_cpu_freq_hz);
}

/**
 * @brief 计算从起始计数值到当前时刻经过的毫秒数。
 * @param start_cycle 起始时刻保存的 CYCCNT 值。
 * @return 经过时间，单位为毫秒。
 */
uint32_t DWT_ElapsedMs(uint32_t start_cycle)
{
    /* 已经过的周期数。 */
    uint32_t elapsed_cycle = DWT_ElapsedCycle(start_cycle);

    return (uint32_t)(((uint64_t)elapsed_cycle * 1000ULL) / dwt_cpu_freq_hz);
}

/**
 * @brief 忙等待指定数量的 CPU 周期。
 * @param cycles 要等待的周期数。
 */
void DWT_Delay_Cycle(uint32_t cycles)
{
    /* 忙等待开始时的计数值。 */
    uint32_t start_cycle = DWT_GetCycle();

    while (DWT_ElapsedCycle(start_cycle) < cycles)
    {
        __NOP();
    }
}

/**
 * @brief 忙等待指定的微秒数。
 * @param us 要等待的时间，单位为微秒。
 */
void DWT_Delay_Us(uint32_t us)
{
    /* 将目标微秒数换算成周期数。 */
    uint32_t cycles = DWT_UsToCycle(us);

    DWT_Delay_Cycle(cycles);
}

/**
 * @brief 忙等待指定的毫秒数。
 * @param ms 要等待的时间，单位为毫秒。
 */
void DWT_Delay_Ms(uint32_t ms)
{
    /*
     * 分成 1ms 一次，避免大延时导致周期数溢出
     */
    while (ms--)
    {
        DWT_Delay_Us(1000);
    }
}

/**
 * @brief 判断微秒级超时是否已经到达。
 * @param start_cycle 起始时刻保存的 CYCCNT 值。
 * @param timeout_us 超时时间，单位为微秒。
 * @return 1 表示已超时，0 表示尚未超时。
 */
uint8_t DWT_IsTimeoutUs(uint32_t start_cycle, uint32_t timeout_us)
{
    /* 超时时间对应的周期数。 */
    uint32_t timeout_cycle = DWT_UsToCycle(timeout_us);

    if (DWT_ElapsedCycle(start_cycle) >= timeout_cycle)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief 判断毫秒级超时是否已经到达。
 * @param start_cycle 起始时刻保存的 CYCCNT 值。
 * @param timeout_ms 超时时间，单位为毫秒。
 * @return 1 表示已超时，0 表示尚未超时。
 */
uint8_t DWT_IsTimeoutMs(uint32_t start_cycle, uint32_t timeout_ms)
{
    /* 超时时间对应的周期数。 */
    uint32_t timeout_cycle = DWT_MsToCycle(timeout_ms);

    if (DWT_ElapsedCycle(start_cycle) >= timeout_cycle)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
