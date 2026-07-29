#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#include "stm32g4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 文本发送函数。
 *
 * data：需要发送的文本
 * len ：文本长度
 *
 * 如果传入NULL，控制台仍然可以接收并执行SET等命令，
 * 但不会回复OK、ERR、HELP等文本。
 */
typedef void (*DebugConsole_TxFn_t)(
    const uint8_t *data,
    uint16_t len);

/*
 * 自定义命令回调。
 *
 * 例如输入：
 * start 100
 *
 * argc = 2
 * argv[0] = "start"
 * argv[1] = "100"
 */
typedef void (*DebugConsole_CommandFn_t)(
    int argc,
    char *argv[]);

/**
 * @brief 初始化DMA空闲接收
 *
 * @param huart  使用的串口
 * @param tx_fn  文本回复函数；不需要回复时可以传NULL
 */
HAL_StatusTypeDef DebugConsole_Init(
    UART_HandleTypeDef *huart,
    DebugConsole_TxFn_t tx_fn);

/**
 * @brief 在主循环中反复调用
 *
 * 不能放在ADC高速中断中。
 */
void DebugConsole_Process(void);

/**
 * @brief 从HAL_UARTEx_RxEventCallback调用
 */
void DebugConsole_OnRxEvent(
    UART_HandleTypeDef *huart,
    uint16_t size);

/**
 * @brief 从HAL_UART_ErrorCallback调用
 */
void DebugConsole_OnError(
    UART_HandleTypeDef *huart);

/*
 * 注册可调变量。
 *
 * name字符串必须一直有效，建议直接传字符串常量。
 */
bool DebugConsole_RegisterF32(
    const char *name,
    volatile float *value,
    float minimum,
    float maximum,
    bool read_only);

bool DebugConsole_RegisterI32(
    const char *name,
    volatile int32_t *value,
    int32_t minimum,
    int32_t maximum,
    bool read_only);

bool DebugConsole_RegisterU32(
    const char *name,
    volatile uint32_t *value,
    uint32_t minimum,
    uint32_t maximum,
    bool read_only);

/*
 * BOOL使用uint32_t存储：
 *
 * 0 = false
 * 1 = true
 */
bool DebugConsole_RegisterBool(
    const char *name,
    volatile uint32_t *value,
    bool read_only);

/**
 * @brief 注册自定义命令
 */
bool DebugConsole_RegisterCommand(
    const char *name,
    DebugConsole_CommandFn_t handler,
    const char *help);

/**
 * @brief 由自定义命令输出文本
 */
void DebugConsole_Printf(
    const char *format,
    ...);

/**
 * @brief 获取接收环形缓冲区溢出次数
 */
uint32_t DebugConsole_GetOverflowCount(void);

#ifdef __cplusplus
}
#endif

#endif