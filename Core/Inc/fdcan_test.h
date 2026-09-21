/*
 * @file    fdcan_test.h
 * @brief   CAN FD 单帧测试接口声明。
 * @note    本文件属于用户维护代码，不由 CubeMX 生成，避免重新生成时被覆盖。
 */

/* 防止头文件被重复包含。 */
#ifndef FDCAN_TEST_H
#define FDCAN_TEST_H

/* 兼容 C++ 工程调用 C 接口，避免函数名被 C++ 编译器改名。 */
#ifdef __cplusplus
extern "C" {
#endif

/* STM32 HAL 状态类型定义。 */
#include "stm32g4xx_hal.h"

/**
 * @brief 向 FDCAN TX FIFO 加入一帧单帧测试报文。
 * @retval HAL_OK    报文已加入发送队列。
 * @retval HAL_BUSY 发送队列无可用空间。
 * @retval HAL_ERROR HAL 库发送接口返回错误。
 */
HAL_StatusTypeDef CANFD_SendSingleTestFrame(void);

#ifdef __cplusplus
}
#endif

#endif /* FDCAN_TEST_H：防止重复包含 */
