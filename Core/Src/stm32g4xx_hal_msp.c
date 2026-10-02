/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32g4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */
/*
 * 本文件由CubeMX维护，只有USER CODE块会在重新生成时保留，因此说明写在这里。
 *
 * HAL_MspInit()是HAL库的全局MCU级初始化钩子，由HAL_Init()在最早阶段调用：
 * 使能SYSCFG（引脚复用与EXTI配置所必需）与PWR时钟；
 * 关闭UCPD死电池引脚上的内部上拉（该引脚本工程不用，但保留CubeMX的默认处理）。
 *
 * 各外设自己的引脚、时钟与NVIC配置不在这里，而是在各自文件的MspInit里：
 *   HAL_ADC_MspInit  -> adc.c
 *   HAL_TIM_Base_MspInit / HAL_TIM_MspPostInit -> tim.c
 *   HAL_UART_MspInit -> usart.c
 *   HAL_FDCAN_MspInit -> fdcan.c
 *   HAL_OPAMP_MspInit -> opamp.c
 *   HAL_CORDIC_MspInit -> cordic.c
 */
/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
  */
  HAL_PWREx_DisableUCPDDeadBattery();

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
