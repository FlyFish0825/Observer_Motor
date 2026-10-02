/**
 * @file app_memory.h
 * @brief APP Flash布局宏，同时控制链接地址与Bootloader相关编译开关。
 * CMake按APP_LAYOUT注入：standalone从0x08000000启动且不启用Boot返回，
 * boot从0x08005000启动并启用CAN返回Bootloader逻辑。
 * 源码不因布局而分叉，差异完全收敛到下面两个宏。
 */
#ifndef APP_MEMORY_H
#define APP_MEMORY_H

/*
 * APP 起始地址，默认 standalone 布局的 0x08000000。
 * 该值同时决定链接脚本的起始地址和 main() 开头写入 SCB->VTOR 的向量表地址，
 * 两者必须一致，否则中断会跳到错误的向量表。
 * boot 布局由 CMake 覆盖为 0x08005000。
 */
#ifndef APP_FLASH_START
#define APP_FLASH_START       0x08000000UL
#endif

/*
 * 是否启用"APP 返回 Bootloader"逻辑，默认 0（standalone 不启用）。
 * 只有 boot 构建为 1；为 1 时 app_boot_control.c 才会编译进有效路径，
 * 处理 0x000 上的 ENTER_BOOT 命令并写备份寄存器后复位。
 */
#ifndef APP_WITH_BOOTLOADER
#define APP_WITH_BOOTLOADER   0
#endif

#endif /* APP_MEMORY_H */
