/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    opamp.h
  * @brief   This file contains all the function prototypes for
  *          the opamp.c file
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
#ifndef __OPAMP_H__
#define __OPAMP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
/*
 * OPAMP1/2/3构成三相电流采样模拟前端，配合5 mΩ分流电阻与24倍外部增益级。
 * 三个运放都以内部跟随器方式接在各自ADC通道前，用于缓冲与中点偏置（1.65 V）。
 * 换算关系：电流增益 = 3.3 V / 4096 / (0.005 Ω × 24) = 0.0067138671875 A/count，
 * 该值写在foc_math.c的FOC_Data_Init()里，改动运放增益或分流电阻时必须同步。
 */
/* USER CODE END Includes */

extern OPAMP_HandleTypeDef hopamp1;

extern OPAMP_HandleTypeDef hopamp2;

extern OPAMP_HandleTypeDef hopamp3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_OPAMP1_Init(void);
void MX_OPAMP2_Init(void);
void MX_OPAMP3_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __OPAMP_H__ */

