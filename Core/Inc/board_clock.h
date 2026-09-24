#ifndef BOARD_CLOCK_H
#define BOARD_CLOCK_H

/* CBT6 板默认 16 MHz；在 24 MHz 测试板上构建时由 CMake 覆盖。 */
#ifndef BOARD_HSE_HZ
#define BOARD_HSE_HZ 16000000UL
#endif

#if (BOARD_HSE_HZ != 16000000UL) && (BOARD_HSE_HZ != 24000000UL)
#error "BOARD_HSE_HZ must be 16000000UL or 24000000UL"
#endif

#endif /* BOARD_CLOCK_H */
