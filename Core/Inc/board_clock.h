#ifndef BOARD_CLOCK_H
#define BOARD_CLOCK_H

/**
 * @file board_clock.h
 * @brief 板级HSE晶振频率基准，是整条时钟树的根输入。
 * CBT6正式板为16 MHz，24 MHz测试板由CMake覆盖，源码不分叉。
 * 本文件只做常量声明与编译期校验，不含函数或寄存器操作。
 */

/*
 * 板载HSE晶振标称频率，单位Hz。
 * 保存的是物理晶振频率，不是PLL倍频后的SYSCLK：改这里是换晶振，
 * 改PLL参数才是改系统主频。
 *
 * 采用#ifndef可覆盖默认值：构建系统传入-DBOARD_HSE_HZ=24000000UL时
 * 外部定义优先生效；没有外部定义时回退到CBT6板的16 MHz。
 * UL后缀保证该常量在预处理器算术与比较中按unsigned long处理。
 *
 * 本宏影响PLL分频、SysTick延时换算、定时器预分频与PWM载波频率，
 * 进而改变电流环采样点、死区占比与观测器离散化系数，属高风险配置项。
 * 常见错误：填PLL后的170 MHz；直接改成24 MHz污染正式板配置。
 */
#ifndef BOARD_HSE_HZ
#define BOARD_HSE_HZ 16000000UL
#endif

/*
 * 编译期合法性校验：只承认16 MHz与24 MHz两种已定义的硬件配置。
 * 逻辑与表示"既不是16也不是24"，任一命中即静默通过；
 * 若误写成逻辑或，则正确的24 MHz也会触发报错。
 * 用预处理器而非运行时判断，可把时钟配置错误拦在编译阶段，
 * 避免烧片后表现为波特率乱码或电流环啸叫等难定位现象。
 */
#if (BOARD_HSE_HZ != 16000000UL) && (BOARD_HSE_HZ != 24000000UL)
#error "BOARD_HSE_HZ must be 16000000UL or 24000000UL"
#endif

#endif /* BOARD_CLOCK_H */
