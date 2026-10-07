/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
/*
 * 本头文件声明两个ADC的句柄。本工程的实际用途：
 * - ADC1：注入组4个Rank（Ia=IN3、Ic=IN12、U端=IN11、W端=IN14），由TIM1 CH4
 *   上升沿触发的T1_TRGO下降沿驱动，序列结束产生ADC1_2中断，是25 kHz电流环入口；
 *   规则组3个Rank（PA0母线电压、内部温度、VREFINT）由主循环软件触发、间断模式逐Rank读取。
 * - ADC2：注入组2个Rank（Ib=IN3、V端=IN17）；规则组1个Rank。
 * 三相端电压必须与电流同拍采样，因此U/V/W端电压放在注入组而不是规则组。
 * 注入序列的期望值在motor_app.c中用MOTOR_APP_ADC*_INJ_*宏做上电回读自检，
 * 改动本文件的注入通道后必须同步那些宏，否则自检会拒绝启动。
 */
/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1; /* ADC1：注入组4路(电流Ia/Ic、U/W端电压) + 规则组3路(母线、温度、VREFINT) */

extern ADC_HandleTypeDef hadc2; /* ADC2：注入组2路(电流Ib、V端电压) + 规则组1路 */

/* USER CODE BEGIN Private defines */
extern DMA_HandleTypeDef hdma_adc1; /* ADC1规则组DMA通道句柄（注入组不使用DMA） */
extern DMA_HandleTypeDef hdma_adc2; /* ADC2规则组DMA通道句柄 */
/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

