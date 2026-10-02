/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    cordic.h
  * @brief   This file contains all the function prototypes for
  *          the cordic.c file
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
#ifndef __CORDIC_H__
#define __CORDIC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
/*
 * CORDIC硬件加速器用于高频三角运算（Park/逆Park与PLL鉴相）。
 * 配置为COSINE功能、Q1.31定点角度、1次写入2次读取、6个周期：
 * 写入WDATA后第一次读RDATA得到cos，第二次得到sin，顺序不能颠倒。
 * MX_CORDIC_Init()之后必须调用一次CORDIC_SinCos_RegisterConfig()
 * 再使用；高频路径用头文件里的CORDIC_SinCos_FastF32()内联函数直接读写寄存器，
 * 避免函数调用与指针传参开销。
 */
/* USER CODE END Includes */

extern CORDIC_HandleTypeDef hcordic;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CORDIC_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CORDIC_H__ */

