/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
/*
 * TIM1：三相PWM与采样触发时基。中心对齐模式1，PSC=0、ARR=3399，
 * 计数时钟170 MHz，得到25 kHz载波与40 µs的电流环周期；
 * CH1~CH3为PWM1互补输出驱动三相桥，CH4不输出引脚而作为ADC触发源
 * （TRGO=OC4REF），其比较值决定注入采样相对载波的位置；
 * 死区DTG=90（约529 ns），OSSR/OSSI使能保证IDLE关断时引脚被强制到安全状态。
 * 注意：MX_TIM1_Init()后半段会用foc.timer中的pwm_arr/adc_trigger/dead_time
 * 覆盖CubeMX写的默认值，因此FOC_Data_Init()必须在此之前调用。
 *
 * TIM6：PSC=169、ARR=999，170 MHz/170/1000=1 kHz，提供1 ms节拍，
 * 由MotorProtocol_TimerTick()调度CAN周期反馈与高速调试反馈。
 */
/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1; /* TIM1：25 kHz三相PWM + CH4触发ADC注入组 */
extern TIM_HandleTypeDef htim6; /* TIM6：1 ms节拍，调度CAN周期反馈 */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM6_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

