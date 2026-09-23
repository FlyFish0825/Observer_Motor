#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* 手写板级配置只描述硬件差异；时钟初始化通过 HSE_VALUE 使用它。 */

/*
 * 板卡外部晶振频率：当前硬件为 24 MHz。
 * 更换为 16 MHz 晶振板时，只需把下面一行改成 16000000UL，
 * 不需要修改 CMake。24/16 MHz 两种配置都会生成 168 MHz 系统时钟。
 */
#define BOARD_HSE_HZ 24000000UL

#if (BOARD_HSE_HZ != 24000000UL) && (BOARD_HSE_HZ != 16000000UL)
#error "BOARD_HSE_HZ must be 24000000UL or 16000000UL"
#endif

/* HAL 和 CMSIS 使用同一个宏，避免时钟计算与实际晶振不一致。 */
#ifndef HSE_VALUE
#define HSE_VALUE BOARD_HSE_HZ
#endif

#endif /* BOARD_CONFIG_H */
