/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
/*
 * MX_GPIO_Init()在CubeMX生成的初始化区，配置各外设复用引脚、模拟输入
 * （ADC/OPAMP通道）以及排针方向为模拟模式。
 * 三相栅极驱动引脚在上电的最初阶段并不安全：main()在HAL_Init()之前先调用
 * MotorApp_ForcePowerStageSafe()把它们强制拉低，之后才由本文件重新配置为
 * 定时器复用功能。修改本文件的引脚配置时不要破坏这个上电顺序的前提。
 */
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

