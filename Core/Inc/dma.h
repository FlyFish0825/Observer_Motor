/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.h
  * @brief   This file contains all the function prototypes for
  *          the dma.c file
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
#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* DMA memory to memory transfer handles -------------------------------------*/

/* USER CODE BEGIN Includes */
/*
 * MX_DMA_Init()使能DMA控制器时钟并配置NVIC，各通道的具体参数在其外设的
 * MspInit里分别设置（见adc.c与usart.c）。本工程用到四个通道：
 * DMA1_Channel1=USART1_RX、DMA1_Channel2=USART1_TX、
 * DMA1_Channel3=ADC1规则组、DMA1_Channel4=ADC2规则组。
 * 注意：ADC注入组不使用DMA，由中断直接读JDRx寄存器，因此电流环路径不受DMA影响。
 */
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_DMA_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H__ */

