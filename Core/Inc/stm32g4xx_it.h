/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __STM32G4xx_IT_H
#define __STM32G4xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/*
 * 中断向量清单与本工程的对应关系（供对照startup_stm32g431xx.s）：
 *   ADC1_2_IRQHandler      25 kHz控制入口：HAL_ADC_IRQHandler(&hadc1)触发注入完成回调，
 *                          最终进入MotorApp_OnInjectedConversion()。本固件的核心实时路径。
 *   TIM6_DAC_IRQHandler    1 ms节拍：调度CAN周期反馈与高速调试反馈。
 *   DMA1_Channel1/2        USART1接收/发送DMA。
 *   DMA1_Channel3/4        ADC1/ADC2规则组DMA（电流环不使用）。
 *   FDCAN1_IT0/IT1         CAN收发中断，只调用HAL_FDCAN_IRQHandler，帧解析在主循环。
 *   USART1_IRQHandler      控制台错误/空闲事件。
 *   CORDIC_IRQHandler      本工程用轮询方式读CORDIC结果，此中断实际不被触发。
 *   SysTick_Handler        只调用HAL_IncTick()维护HAL毫秒计数。
 * 故障类异常（HardFault/MemManage/BusFault/UsageFault/NMI）都停在死循环里，
 * 现场调试时可用调试器查看栈帧定位出错地址。
 */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel2_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);
void ADC1_2_IRQHandler(void);
void FDCAN1_IT0_IRQHandler(void);
void FDCAN1_IT1_IRQHandler(void);
void TIM1_UP_TIM16_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void TIM3_IRQHandler(void);
void USART1_IRQHandler(void);
void CORDIC_IRQHandler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32G4xx_IT_H */
