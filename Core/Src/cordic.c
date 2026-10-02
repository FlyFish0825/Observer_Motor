/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    cordic.c
  * @brief   This file provides code for the configuration
  *          of the CORDIC instances.
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
#include "cordic.h"

/* USER CODE BEGIN 0 */
/*
 * 本文件由CubeMX维护，只有USER CODE块会在重新生成时保留，因此说明写在这里。
 *
 * CORDIC硬件加速器用于高频三角运算（Park/逆Park与PLL鉴相），
 * 配置为COSINE功能、Q1.31定点角度、1次写入2次读取、6个周期：
 * 写WDATA后第一次读RDATA得cos，第二次读RDATA得sin，顺序不能颠倒。
 * 实际使用入口在foc_math.c：CORDIC_SinCos_RegisterConfig()配置CSR，
 * CORDIC_SinCos_FastF32()（头文件内联）直接读写寄存器避免调用开销。
 */
/* USER CODE END 0 */

CORDIC_HandleTypeDef hcordic;

/* CORDIC init function */
void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */
/*
 * MX_CORDIC_Init()只做HAL_CORDIC_Init()把外设时钟与句柄准备好，
 * 并没有设置功能号和精度：功能（COSINE）、定点格式（Q1.31）、
 * 1写2读、6周期这些都由foc_math.c的CORDIC_SinCos_RegisterConfig()写入CSR，
 * 因此外设初始化之后必须先调用它一次再使用CORDIC。
 */
/* USER CODE END CORDIC_Init 2 */

}

void HAL_CORDIC_MspInit(CORDIC_HandleTypeDef* cordicHandle)
{

  if(cordicHandle->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspInit 0 */

  /* USER CODE END CORDIC_MspInit 0 */
    /* CORDIC clock enable */
    __HAL_RCC_CORDIC_CLK_ENABLE();

    /* CORDIC interrupt Init */
    HAL_NVIC_SetPriority(CORDIC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CORDIC_IRQn);
  /* USER CODE BEGIN CORDIC_MspInit 1 */

  /* USER CODE END CORDIC_MspInit 1 */
  }
}

void HAL_CORDIC_MspDeInit(CORDIC_HandleTypeDef* cordicHandle)
{

  if(cordicHandle->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspDeInit 0 */

  /* USER CODE END CORDIC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CORDIC_CLK_DISABLE();

    /* CORDIC interrupt Deinit */
    HAL_NVIC_DisableIRQ(CORDIC_IRQn);
  /* USER CODE BEGIN CORDIC_MspDeInit 1 */

  /* USER CODE END CORDIC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
