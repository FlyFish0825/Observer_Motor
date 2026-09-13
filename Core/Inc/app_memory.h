#ifndef APP_MEMORY_H
#define APP_MEMORY_H

/*
 * 同一套 APP 源码支持两种链接布局：
 *   standalone：直接从 0x08000000 启动；
 *   boot      ：由 Bootloader 从 0x08005000 启动。
 * CMake 会按 APP_LAYOUT 覆盖下面两个默认值。
 */
#ifndef APP_FLASH_START
#define APP_FLASH_START       0x08000000UL
#endif

#ifndef APP_WITH_BOOTLOADER
#define APP_WITH_BOOTLOADER   0
#endif

#endif /* APP_MEMORY_H */
