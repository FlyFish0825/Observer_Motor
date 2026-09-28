/* UART DMA 空闲接收调试控制台：主循环解析命令，中断只搬运接收数据。 */
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

/* 注册带有最小值/最大值约束的有符号 32 位变量。 */
bool DebugConsole_RegisterI32(
    const char *name,
    volatile int32_t *value,
    int32_t minimum,
    int32_t maximum,
    bool read_only);

/* 注册带有最小值/最大值约束的无符号 32 位变量。 */
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

/* ISR 只提交数值；Rs 使用相别(高 8 位)+序号(低 8 位)，其他模块可用 VALUES。 */
typedef enum {
    DEBUG_LOG_VALUES = 0,       /* 通用：info + 三个 float。 */
    DEBUG_LOG_RS_POINT,         /* Rs 每档测量点。 */
    DEBUG_LOG_RS_RESISTANCE     /* Rs 单组线间电阻。 */
} DebugLogType_t;

/* 多中断安全入队；队列满时直接返回 false，不等待或格式化。 */
bool DebugConsole_LogFromISR(uint8_t type, uint16_t info,
                             float a, float b, float c);
/* 主循环分批格式化日志；Pending 用于等待最终汇总和 JustFloat 让路。 */
void DebugConsole_LogProcess(void);
bool DebugConsole_LogPending(void);

/* JustFloat 原始二进制：6 个 float + 0x7F800000 帧尾，DMA 忙时丢帧。 */
int Fast_Send_6Floats(float f0, float f1, float f2,
                      float f3, float f4, float f5);

/** @brief 主循环文本格式化后入 TX 队列，DMA 后台发送。 */
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
