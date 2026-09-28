#include "bsp_dwt.h"

/* CYCCNT 的换算基准，单位 Hz，由当前 HCLK 初始化。 */
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

/**
 * @brief 初始化DWT周期计数器并使能CYCCNT，返回硬件是否成功启动。
 */
/* 初始化调试寄存器和周期计数器；后续换算函数依赖 dwt_cpu_freq_hz 非零。 */
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

/**
 * @brief 读取当前DWT CYCCNT值，作为高精度时间戳或延时起点。
 */
/* 该值可能自然回绕，调用者应使用 DWT_ElapsedCycle 做差。 */
uint32_t DWT_GetCycle(void)
{
    return DWT->CYCCNT;
}

/**
 * @brief 计算从指定起点到当前时刻经过的CPU周期数，利用无符号减法兼容计数器回绕。
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
 * @brief 将指定起点以来经过的CPU周期数转换为微秒。
 */
uint32_t DWT_ElapsedUs(uint32_t start_cycle)
{
    uint32_t elapsed_cycle = DWT_ElapsedCycle(start_cycle);

    return (uint32_t)(((uint64_t)elapsed_cycle * 1000000ULL) / dwt_cpu_freq_hz);
}

/**
 * @brief 将指定起点以来经过的CPU周期数转换为毫秒。
 */
uint32_t DWT_ElapsedMs(uint32_t start_cycle)
{
    uint32_t elapsed_cycle = DWT_ElapsedCycle(start_cycle);

    return (uint32_t)(((uint64_t)elapsed_cycle * 1000ULL) / dwt_cpu_freq_hz);
}

/**
 * @brief 按照CPU周期执行忙等待延时，适用于短时间、对时序要求高的代码。
 */
/* 忙等待期间不让出 CPU，仅适合短延时或严格时序窗口。 */
void DWT_Delay_Cycle(uint32_t cycles)
{
    uint32_t start_cycle = DWT_GetCycle();

    while (DWT_ElapsedCycle(start_cycle) < cycles)
    {
        __NOP();
    }
}

/**
 * @brief 按照微秒执行忙等待延时，内部将时间换算为DWT周期。
 */
void DWT_Delay_Us(uint32_t us)
{
    uint32_t cycles = DWT_UsToCycle(us);

    DWT_Delay_Cycle(cycles);
}

/**
 * @brief 按照毫秒执行忙等待延时，通过多个1毫秒延时避免长周期换算溢出。
 */
/* 分段执行毫秒延时，避免一次性周期换算过大。 */
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
 * @brief 判断从起点开始是否已经达到指定的微秒超时时间。
 */
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

/**
 * @brief 判断从起点开始是否已经达到指定的毫秒超时时间。
 */
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
