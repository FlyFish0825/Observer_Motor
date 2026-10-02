/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
/*
 * 本文件由CubeMX维护，只有USER CODE块会在重新生成时保留，因此说明写在这里。
 *
 * MX_GPIO_Init()只使能了GPIOF/GPIOA/GPIOB三个端口的时钟，没有在本文件里
 * 配置任何引脚：本工程用到的引脚都在各自外设的HAL_xxx_MspInit/MspPostInit中配置
 * （PWM与复用功能在tim.c，ADC与OPAMP模拟输入在adc.c/opamp.c，
 * CAN与串口引脚在fdcan.c/usart.c），因此这里保持为空。
 *
 * 上电安全的前提：main()在HAL_Init()之前先调用MotorApp_ForcePowerStageSafe()
 * 把六路栅极驱动输出拉低，之后才由各MspInit把引脚切成定时器复用功能。
 * 修改引脚配置时不要破坏这个先后顺序。
 */
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
