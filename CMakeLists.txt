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



# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

target_link_options(${PROJECT_NAME} PRIVATE
    -u _printf_float
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
    ${CMAKE_CURRENT_SOURCE_DIR}/vesc/Src/vesc_observer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/vesc/Src/vesc_current_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/vesc/Src/vesc_sensorless_foc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/vesc/Src/vesc_speed_controller.c
)



# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/vesc/Inc
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
