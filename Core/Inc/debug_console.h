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

/**
 * @brief 注册一个可读写或只读的浮点变量及其合法范围。
 * @param name     命令行中使用的变量名，字符串必须全程有效（建议传字符串常量）。
 * @param value    被访问的浮点变量地址。
 * @param minimum  允许写入的最小值。
 * @param maximum  允许写入的最大值。
 * @param read_only true时只允许get，禁止set。
 * @return true注册成功；false名称重复、容量满或参数无效。
 */
bool DebugConsole_RegisterF32(
    const char *name,
    volatile float *value,
    float minimum,
    float maximum,
    bool read_only);

/**
 * @brief 注册一个可读写或只读的有符号32位整数变量。
 * @param name     变量名，必须全程有效。
 * @param value    被访问的int32变量地址。
 * @param minimum  允许写入的最小值。
 * @param maximum  允许写入的最大值。
 * @param read_only true时只允许get，禁止set。
 * @return true注册成功；false名称重复、容量满或参数无效。
 */
bool DebugConsole_RegisterI32(
    const char *name,
    volatile int32_t *value,
    int32_t minimum,
    int32_t maximum,
    bool read_only);

/**
 * @brief 注册一个可读写或只读的无符号32位整数变量。
 * @param name     变量名，必须全程有效。
 * @param value    被访问的uint32变量地址。
 * @param minimum  允许写入的最小值。
 * @param maximum  允许写入的最大值。
 * @param read_only true时只允许get，禁止set。
 * @return true注册成功；false名称重复、容量满或参数无效。
 */
bool DebugConsole_RegisterU32(
    const char *name,
    volatile uint32_t *value,
    uint32_t minimum,
    uint32_t maximum,
    bool read_only);

/**
 * @brief 注册一个以0/1存储的布尔变量。
 * @param name      变量名，必须全程有效。
 * @param value     被访问的uint32变量地址（0=false, 1=true）。
 * @param read_only true时只允许get，禁止set。
 * @return true注册成功；false名称重复或容量满。
 */
bool DebugConsole_RegisterBool(
    const char *name,
    volatile uint32_t *value,
    bool read_only);

/**
 * @brief 注册自定义命令。
 * @param name    命令名称，字符串必须全程有效；不能与内置命令（help/list/get/set）重名。
 * @param handler 收到命令后的回调函数，接收argc和argv。
 * @param help    help命令显示的说明文本，可为NULL。
 * @return true注册成功；false名称冲突、容量满或参数无效。
 */
bool DebugConsole_RegisterCommand(
    const char *name,
    DebugConsole_CommandFn_t handler,
    const char *help);

/**
 * @brief 格式化输出文本到串口（通过注册的发送回调）。
 * @param format printf风格的格式字符串，后跟对应参数。
 * @note 输出超过192字节会被截断；在中断中调用时须确保发送回调安全。
 */
void DebugConsole_Printf(
    const char *format,
    ...);

/**
 * @brief 获取接收环形缓冲区累计溢出（丢弃）的字节数。
 * @return 溢出字节总数，可用于诊断串口接收是否过快。
 */
uint32_t DebugConsole_GetOverflowCount(void);

#ifdef __cplusplus
}
#endif

#endif