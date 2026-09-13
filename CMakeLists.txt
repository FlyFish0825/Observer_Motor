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

# 同一套源码保留 standalone / boot 两种 Flash 布局。
set(APP_LAYOUT "standalone" CACHE STRING "APP memory layout: standalone or boot")
set_property(CACHE APP_LAYOUT PROPERTY STRINGS standalone boot)


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

if(APP_LAYOUT STREQUAL "boot")
    message(STATUS "APP layout: bootloader offset 0x08005000")
    set(APP_OUTPUT_SUFFIX "_boot")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        APP_FLASH_START=0x08005000UL
        APP_WITH_BOOTLOADER=1
    )
elseif(APP_LAYOUT STREQUAL "standalone")
    message(STATUS "APP layout: standalone 0x08000000")
    set(APP_OUTPUT_SUFFIX "_standalone")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        APP_FLASH_START=0x08000000UL
        APP_WITH_BOOTLOADER=0
    )
else()
    message(FATAL_ERROR "Unknown APP_LAYOUT=${APP_LAYOUT}; use standalone or boot")
endif()



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
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/debug_console.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/board_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/motor_app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/app_boot_control.c
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

# 输出名称明确区分可独立烧录版本和 Bootloader 偏移版本，避免误刷。
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}${APP_OUTPUT_SUFFIX}.bin
    COMMENT "Generating ${CMAKE_PROJECT_NAME}${APP_OUTPUT_SUFFIX}.bin"
)
