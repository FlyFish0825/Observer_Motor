/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "adc.h"
#include "cordic.h"
#include "dma.h"
#include "fdcan.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_memory.h"
#include "board_config.h"
#include "debug_console.h"
#include "foc_math.h"
#include "motor_app.h"
#include "motor_protocol.h"
#include <stddef.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/** @brief 配置系统时钟：HSE晶振经PLL倍频输出168MHz SYSCLK/FDCAN时钟。 */
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
/**
 * @brief 应用程序入口，完成硬件初始化、FOC启动和后台协议处理。
 * @note CubeMX生成的外设初始化保持原样，项目业务逻辑集中在初始化后的主循环中。
 */
/** @brief 应用程序入口，完成向量表重定位、外设初始化、FOC控制环启动和主循环调度。 */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /*
   * 必须在 HAL_Init() 开启 SysTick 中断前完成向量表重定位。
   * standalone：APP_FLASH_START = 0x08000000
   * boot      ：APP_FLASH_START = 0x08005000
   */
  SCB->VTOR = APP_FLASH_START;
  __DSB();
  __ISB();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  FOC_Data_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_CORDIC_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  /* 实时电机业务由 MotorApp 统一初始化，保留既有 ADC 零偏校准流程。 */
  if (MotorApp_Init() != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* CAN 初始化仍在电流零偏校准之后，原有时序与配置不变。 */
  /* 仅BRS高速数据段需要TDC；当前1M无BRS测试不启用。 */
#if MOTOR_PROTOCOL_CANFD_BRS_ENABLED
  if (HAL_FDCAN_ConfigTxDelayCompensation(
          &hfdcan1,
          hfdcan1.Init.DataPrescaler * hfdcan1.Init.DataTimeSeg1,
          0U) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
#endif

  /*
   * ADC/运放校准完成后再启动CAN反馈定时器，避免100 us协议中断干扰校准。
   * 电机运行、转速设定、反馈和ENTER_BOOT共用一套CAN协议。
   */
  if (MotorProtocol_Init(&hfdcan1, MotorApp_GetControl()) != HAL_OK) {
    Error_Handler();
  }

  uint32_t led_task_tick = HAL_GetTick();
  while (1) {
    MotorApp_Process();
    if ((HAL_GetTick() - led_task_tick) >= 500U) {
      led_task_tick = HAL_GetTick();
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  /*
   * 24 MHz 与 16 MHz 晶振都生成精确的 168 MHz SYSCLK/FDCAN 时钟：
   * 24 / 2 * 28 / 2 = 168 MHz；16 / 2 * 42 / 2 = 168 MHz。
   * BOARD_HSE_HZ 在 Core/Inc/board_config.h 中配置，防止刷入不匹配晶振的固件。
   */
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
#if BOARD_HSE_HZ == 24000000UL
  RCC_OscInitStruct.PLL.PLLN = 28;
#elif BOARD_HSE_HZ == 16000000UL
  RCC_OscInitStruct.PLL.PLLN = 42;
#else
#error "Unsupported BOARD_HSE_HZ: use 24000000 or 16000000"
#endif
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/** @brief HAL回调只转发电机实时任务，不在入口重复实现控制逻辑。 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  MotorApp_OnInjectedConversion(hadc);
}

/** @brief printf底层输出重定向：通过USART2阻塞发送，供调试输出使用。 */
int _write(int file, char *ptr, int len) {
  (void)file;

  if ((ptr == NULL) || (len <= 0)) {
    return 0;
  }

  if (HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len,
                        HAL_MAX_DELAY) == HAL_OK) {
    return len;
  }

  return -1;
}

/** @brief 串口DMA空闲接收完成回调，将数据交给调试控制台处理。 */
void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef *huart,
    uint16_t Size)
{
    /* 回调仅转交事件；命令解析在主循环执行，避免DMA/USART中断中执行重活。 */
    DebugConsole_OnRxEvent(huart, Size);
}

/** @brief 串口错误回调，通知调试控制台标记重启接收。 */
void HAL_UART_ErrorCallback(
    UART_HandleTypeDef *huart)
{
    /* 错误路径只记录并安排恢复，不在USART错误中断中阻塞等待。 */
    DebugConsole_OnError(huart);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
/** @brief HAL错误处理入口：关闭中断后进入死循环，便于调试器定位故障。 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
/** @brief assert_param断言失败回调（仅USE_FULL_ASSERT启用时编译）。 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */