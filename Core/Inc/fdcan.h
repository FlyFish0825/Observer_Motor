/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
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
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
/*
 * FDCAN1（PA11=RX/PA12=TX）用于电机协议与返回Bootloader。
 * 仲裁段与数据段均配置为1 Mbit/s，且关闭BRS（位速率切换）：
 * 这是板载CAN收发器带宽限制决定的，协议层用MOTOR_PROTOCOL_CANFD_BRS_ENABLED=0
 * 记住这一约束，即使FDFormat为FD_CAN也不会打开BRS。
 * 接收：主循环轮询FIFO0，把帧放入协议环形队列后再解析，不在中断里解析；
 * 两个中断入口FDCAN1_IT0/IT1都只调用HAL_FDCAN_IRQHandler。
 */
/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */
/* 协议层的节点号、帧ID与长度宏定义在motor_protocol.h，本文件不重复定义。 */
/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

