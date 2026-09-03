cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Observer)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})


set(APP_LAYOUT "standalone" CACHE STRING
    "APP memory layout: standalone or boot")
set_property(
    CACHE APP_LAYOUT
    PROPERTY STRINGS standalone boot
)

# 同一套 APP 源码支持两种 Flash 布局：
#
# standalone：向量表位于 0x08000000，可由 MCU 复位后直接启动；
# boot      ：前 20 KiB 留给 Bootloader，APP 向量表位于 0x08005000，
#             Flash 最后 2 KiB（0x0801F800~0x0801FFFF）预留给配置数据。
#
# 链接脚本地址和 APP_FLASH_START 必须始终成对变化，否则 ELF 的实际
# 向量表地址与运行时 SCB->VTOR 会不一致。
if(APP_LAYOUT STREQUAL "standalone")
    set(APP_LINKER_SCRIPT
        "${CMAKE_CURRENT_SOURCE_DIR}/STM32G431CBTX_STANDALONE.ld")
    set(APP_FLASH_START 0x08000000UL)
    set(APP_WITH_BOOTLOADER 0)
    set(APP_OUTPUT_SUFFIX "_standalone")
elseif(APP_LAYOUT STREQUAL "boot")
    set(APP_LINKER_SCRIPT
        "${CMAKE_CURRENT_SOURCE_DIR}/STM32G431CBTX_BOOT_APP.ld")
    set(APP_FLASH_START 0x08005000UL)
    set(APP_WITH_BOOTLOADER 1)
    set(APP_OUTPUT_SUFFIX "_boot")
else()
    message(FATAL_ERROR
        "Unknown APP_LAYOUT='${APP_LAYOUT}'. Use 'standalone' or 'boot'.")
endif()

if(NOT EXISTS "${APP_LINKER_SCRIPT}")
    message(FATAL_ERROR "Linker script not found: ${APP_LINKER_SCRIPT}")
endif()

message(STATUS "APP layout       : ${APP_LAYOUT}")
message(STATUS "APP Flash start  : ${APP_FLASH_START}")
message(STATUS "APP linker script: ${APP_LINKER_SCRIPT}")

# 这里只添加当前模式对应的一个 -T 参数。工具链文件不再固定链接脚本，
# 避免同时向链接器传入两个脚本。
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    "-T${APP_LINKER_SCRIPT}"
)

# app_memory.h 使用这两个宏设置 VTOR，并允许以后只在 boot 模式编译
# ENTER_BOOT、备份寄存器请求等 Bootloader 相关代码。
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    APP_FLASH_START=${APP_FLASH_START}
    APP_WITH_BOOTLOADER=${APP_WITH_BOOTLOADER}
)



# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Release speed optimization for the high-frequency FOC control path.
# The toolchain currently uses -Os for Release; -O3 below appears later
# on the compiler command line and therefore takes precedence.
# LTO allows GCC to inline and optimize across main.c/foc_math.c/
# observer.c/controller.c without changing the source architecture.
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    "$<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:C>>:-O3;-flto>"
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

target_link_options(${PROJECT_NAME} PRIVATE
    -u _printf_float
    $<$<CONFIG:Release>:-flto>
)


# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/bsp_dwt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/foc_math.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/observer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/AS5600.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/debug_console.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/controller.c
)



# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# 从 ELF 生成可直接下载或交给 CAN Bootloader 的裸 BIN。
# 文件名带布局后缀，防止把 0x08000000 版本误发到 0x08005000，反之亦然。
add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY}
            -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}${APP_OUTPUT_SUFFIX}.bin"
    COMMENT "Generating ${CMAKE_PROJECT_NAME}${APP_OUTPUT_SUFFIX}.bin"
    VERBATIM
)
