#ifndef APP_MEMORY_H
#define APP_MEMORY_H

/*
 * CMake 会根据当前构建模式覆盖 APP_FLASH_START。
 *
 * standalone:
 * APP_FLASH_START = 0x08000000
 *
 * boot:
 * APP_FLASH_START = 0x08005000
 */
#ifndef APP_FLASH_START
#define APP_FLASH_START 0x08000000UL
#endif

/* standalone 默认不依赖 Bootloader；CMake 的 boot 模式会把它定义为 1。 */
#ifndef APP_WITH_BOOTLOADER
#define APP_WITH_BOOTLOADER 0
#endif

/* Cortex-M4 向量表地址必须按 0x200 字节对齐；0x08005000 满足该要求。 */
#if ((APP_FLASH_START & 0x1FFUL) != 0UL)
#error "APP_FLASH_START must be aligned to 0x200 bytes"
#endif

#define APP_VTOR_OFFSET (APP_FLASH_START - 0x08000000UL)

#endif
