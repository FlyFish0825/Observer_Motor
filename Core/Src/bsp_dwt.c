#include "bsp_dwt.h"

static uint32_t dwt_cpu_freq_hz = 0;

/**
 * @brief us 转换为 CPU 周期数
 */
static uint32_t DWT_UsToCycle(uint32_t us)
{
    return (uint32_t)(((uint64_t)dwt_cpu_freq_hz * us) / 1000000ULL);
}

/**
 * @brief ms 转换为 CPU 周期数
 */
static uint32_t DWT_MsToCycle(uint32_t ms)
{
    return (uint32_t)(((uint64_t)dwt_cpu_freq_hz * ms) / 1000ULL);
}

uint8_t DWT_Delay_Init(void)
{
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

uint32_t DWT_GetCycle(void)
{
    return DWT->CYCCNT;
}

uint32_t DWT_ElapsedCycle(uint32_t start_cycle)
{
    /*
     * uint32_t 自然溢出特性：
     * 即使 CYCCNT 回绕，只要单次测量时间不超过一次回绕周期，也能算对。
     */
    return DWT->CYCCNT - start_cycle;
}

uint32_t DWT_ElapsedUs(uint32_t start_cycle)
{
    uint32_t elapsed_cycle = DWT_ElapsedCycle(start_cycle);

    return (uint32_t)(((uint64_t)elapsed_cycle * 1000000ULL) / dwt_cpu_freq_hz);
}

uint32_t DWT_ElapsedMs(uint32_t start_cycle)
{
    uint32_t elapsed_cycle = DWT_ElapsedCycle(start_cycle);

    return (uint32_t)(((uint64_t)elapsed_cycle * 1000ULL) / dwt_cpu_freq_hz);
}

void DWT_Delay_Cycle(uint32_t cycles)
{
    uint32_t start_cycle = DWT_GetCycle();

    while (DWT_ElapsedCycle(start_cycle) < cycles)
    {
        __NOP();
    }
}

void DWT_Delay_Us(uint32_t us)
{
    uint32_t cycles = DWT_UsToCycle(us);

    DWT_Delay_Cycle(cycles);
}

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

uint8_t DWT_IsTimeoutUs(uint32_t start_cycle, uint32_t timeout_us)
{
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

uint8_t DWT_IsTimeoutMs(uint32_t start_cycle, uint32_t timeout_ms)
{
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