/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
/*
 * USART1（PB6=TX/PB7=RX）作为调试控制台，2 Mbaud。
 * 接收用DMA空闲中断：HAL_UARTEx_RxEventCallback -> DebugConsole_OnRxEvent
 * 只把字节推入环形队列，命令行解析留在主循环；
 * 发送用DMA + 发送环形队列，并靠motor_console_tx_active标志避免
 * VOFA波形帧与命令文本回复同时改写同一个USART。
 * main.c中的_write()也重定向到本串口，用于printf调试输出。
 */
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
extern DMA_HandleTypeDef hdma_usart1_tx; /* USART1发送DMA通道，用于环形队列与VOFA波形发送 */
extern DMA_HandleTypeDef hdma_usart1_rx; /* USART1接收DMA通道，空闲中断方式接收命令行 */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

