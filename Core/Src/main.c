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
#include "gpio.h"
#include "i2c.h"
#include "opamp.h"
#include "stm32g4xx_hal_adc.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AS5600.h"
#include "arm_math.h"
#include "bsp_dwt.h"
#include "foc_math.h"
#include "stdio.h"
#include <math.h>
#include <stdint.h>
#include "debug_console.h"
#include "controller.h"
#include "vesc_project_config.h"
#include "vesc_sensorless_foc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


static void DebugConsole_Tx(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)data,
        len,
        100U);
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

typedef struct {
  float data[6];
  uint32_t tail;
} JustFloatFrame_t;

// Cortex-M4 是小端模式： 0x7F800000 在内存中排列为 00 00 80 7F
static JustFloatFrame_t tx_frame __attribute__((aligned(4)));

uint16_t as5600_raw = 0U;
float as5600_elec_rad = 0.0f;

/* VESC核心：观测器、电流环和无感启动状态机 */
static VESC_SensorlessFOC_t g_vesc_foc;
static VESC_SensorlessFOC_Output_t g_vesc_out;
static float g_vesc_iq_command_a = 0.3f;

static float VESC_Port_Atan2(float y, float x) {
  return FOC_Atan2_Fast(y, x);
}

static void VESC_Port_SinCos(float angle_rad, float *s, float *c) {
  uint32_t angle_q31 = (uint32_t)CORDIC_RadToQ31(angle_rad);
  CORDIC_SinCos_FastF32(angle_q31, s, c);
}

static void VESC_Port_Init(void) {
  VESC_SensorlessFOC_Config_t cfg = VESC_Project_DefaultConfig();
  cfg.observer.atan2_fn = VESC_Port_Atan2;
  cfg.sincos_fn = VESC_Port_SinCos;
  VESC_SensorlessFOC_Init(&g_vesc_foc, &cfg);
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void FOC_ADC_AND_OPAMP_Calibration_Start(void);

void JustFloat_Init(void);
int Fast_Send_6Floats(float f0, float f1, float f2, float f3, float f4,
                      float f5);





/* ======================== 电流零偏校准 ======================== */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_CORDIC_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();
  CORDIC_SinCos_RegisterConfig();
  JustFloat_Init();
  AS5600_init();

  VESC_Port_Init();


  if (DebugConsole_Init(&huart2, DebugConsole_Tx) != HAL_OK) {
    Error_Handler();
  }
  DebugConsole_RegisterF32("ud", &foc.state.u_dq.d, 
    -8.0f, 8.0f, false);
  DebugConsole_RegisterF32("uq", &foc.state.u_dq.q, 
    -8.0f, 8.0f, false);

  FOC_ADC_AND_OPAMP_Calibration_Start();

  // FOC_Iabc_Calibration();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK) {
    Error_Handler();
  }

  uint8_t buf[2];
  while (1) {
    HAL_I2C_Mem_Read(&hi2c1, (0x36U << 1), 0x0CU, I2C_MEMADD_SIZE_8BIT, buf, 2U,
                     10U);

    as5600_raw = ((uint16_t)(buf[0] & 0x0FU) << 8) | (uint16_t)buf[1];
    int32_t delta_raw = (int32_t)as5600_raw - 1017;

    as5600_elec_rad = (float)delta_raw * 7.0f * (2.0f * FOC_PI / 4096.0f);

    as5600_elec_rad = FOC_WrapToPi(-as5600_elec_rad);

    // ADC1 采样母线电压
    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK) {
      uint16_t adc_value = (uint16_t)HAL_ADC_GetValue(&hadc1);

      foc.state.vbus = (float)adc_value * 26.0f * 3.3f / 4096.0f;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


    DebugConsole_Process();
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 28;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {

  static uint16_t calibration_count = 0U;
  static uint16_t debug_divider = 0U;
  uint32_t adc[3] = {0U};

  if (hadc->Instance != ADC1) {
    return;
  }

  if (foc.calibration.calibrated == 0U) {
    calibration_count++;

    foc.calibration.ia_offset +=
        HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    foc.calibration.ib_offset +=
        HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    foc.calibration.ic_offset +=
        HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

    if (calibration_count >= CURRENT_OFFSET_SAMPLE_NUM) {
      foc.calibration.ia_offset /= (float)CURRENT_OFFSET_SAMPLE_NUM;
      foc.calibration.ib_offset /= (float)CURRENT_OFFSET_SAMPLE_NUM;
      foc.calibration.ic_offset /= (float)CURRENT_OFFSET_SAMPLE_NUM;
      foc.calibration.calibrated = 1U;
    }
    return;
  }

  adc[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
  adc[1] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
  adc[2] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

  FOC_Get_Iabc(&foc, adc[0], adc[1], adc[2]);
  FOC_Clarke(&foc.state.i_abc, &foc.state.i_alpha_beta);

  /*
   * foc.svpwm.duty_a/b/c 是上一控制拍真正施加的占空比，
   * 正好用于本拍观测器的电压重构。
   */
  VESC_SensorlessFOC_Input_t vesc_in = {
      .enable = (uint8_t)(foc_motor_state != FOC_MOTOR_IDLE),
      .iq_ref_a = g_vesc_iq_command_a,
      .i_alpha = foc.state.i_alpha_beta.alpha,
      .i_beta = foc.state.i_alpha_beta.beta,
      .duty_a = foc.svpwm.duty_a,
      .duty_b = foc.svpwm.duty_b,
      .duty_c = foc.svpwm.duty_c,
      .vbus = foc.state.vbus,
  };

  VESC_SensorlessFOC_Run(&g_vesc_foc, &vesc_in, &g_vesc_out);

  /* 保存到原项目数据结构，便于原有调试工具继续使用。 */
  foc.state.i_dq.d = g_vesc_out.id;
  foc.state.i_dq.q = g_vesc_out.iq;
  foc.state.u_dq.d = g_vesc_out.ud;
  foc.state.u_dq.q = g_vesc_out.uq;
  foc.state.u_alpha_beta.alpha = g_vesc_out.u_alpha;
  foc.state.u_alpha_beta.beta = g_vesc_out.u_beta;

  /* IDLE/FAULT时g_vesc_out电压为0，SVPWM输出零线电压，避免保留旧CCR。 */
  FOC_InvClarke(&foc.state.u_alpha_beta, &foc.state.u_abc);
  FOC_SVPWM_Run(&foc.state.u_abc,
                foc.state.vbus,
                &foc.timer,
                &foc.svpwm);

  TIM1->CCR1 = foc.svpwm.ccr_a;
  TIM1->CCR2 = foc.svpwm.ccr_b;
  TIM1->CCR3 = foc.svpwm.ccr_c;

  /* 25kHz中断不能每拍发串口；25分频后为1kHz。 */
  debug_divider++;
  if (debug_divider >= 25U) {
    debug_divider = 0U;


    Fast_Send_6Floats(
    (float)g_vesc_out.state,       /* CH0 状态 */
    g_vesc_out.transition_offset,  /* CH1 保存的角度偏移 */
    g_vesc_out.iq_ref,             /* CH2 Iq参考 */
    g_vesc_out.iq,                 /* CH3 实际Iq */
    g_vesc_out.uq,                 /* CH4 Uq输出 */
    g_vesc_out.omega_e             /* CH5 电角速度 */
);




  }
}

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

void FOC_ADC_AND_OPAMP_Calibration_Start(void) {
  // 校准三个内部运放 */
  if (HAL_OPAMP_SelfCalibrate(&hopamp1) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_OPAMP_SelfCalibrate(&hopamp2) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_OPAMP_SelfCalibrate(&hopamp3) != HAL_OK) {
    Error_Handler();
  }

  /* 启动三个内部运放 */
  if (HAL_OPAMP_Start(&hopamp1) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_OPAMP_Start(&hopamp2) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_OPAMP_Start(&hopamp3) != HAL_OK) {
    Error_Handler();
  }

  DWT_Delay_Ms(10); // 等待运放稳定

  /* ADC 自校准 */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler();
  }

  DWT_Delay_Ms(10); // 等待运放稳定
}

/**
 * @brief 初始化 JustFloat 和 USART2 TX DMA
 */
void JustFloat_Init(void) {
  /* VOFA+ JustFloat 帧尾 */
  tx_frame.tail = 0x7F800000UL;

  /* 暂时关闭 USART DMA 发送请求 */
  CLEAR_BIT(USART2->CR3, USART_CR3_DMAT);

  /* 禁用 DMA 通道 */
  __HAL_DMA_DISABLE(&hdma_usart2_tx);

  while ((hdma_usart2_tx.Instance->CCR & DMA_CCR_EN) != 0U) {
  }

  /* 清除该 DMA 通道的全部标志 */
  __HAL_DMA_CLEAR_FLAG(&hdma_usart2_tx,
                       __HAL_DMA_GET_GI_FLAG_INDEX(&hdma_usart2_tx));

  /*
   * 关键：DMA 外设地址必须是 USART2 发送数据寄存器
   */
  hdma_usart2_tx.Instance->CPAR = (uint32_t)&USART2->TDR;

  /*
   * 内存地址指向发送缓冲区
   */
  hdma_usart2_tx.Instance->CMAR = (uint32_t)&tx_frame;

  hdma_usart2_tx.Instance->CNDTR = 0U;

  /*
   * 关键：允许 USART2 产生 TX DMA 请求
   */
  SET_BIT(USART2->CR3, USART_CR3_DMAT);
}

/**
 * @brief 非阻塞发送6个float
 * @retval  0：发送启动成功
 * @retval -1：上一次数据仍未发送完成
 */
int Fast_Send_6Floats(float f0, float f1, float f2, float f3, float f4,
                      float f5) {
  /*
   * TC=1 表示：
   * DMA、TDR、移位寄存器中的数据全部发送完成。
   */
  if ((USART2->ISR & USART_ISR_TC) == 0U) {
    return -1;
  }

  tx_frame.data[0] = f0;
  tx_frame.data[1] = f1;
  tx_frame.data[2] = f2;
  tx_frame.data[3] = f3;
  tx_frame.data[4] = f4;
  tx_frame.data[5] = f5;

  /*
   * 修改 CNDTR、CMAR 之前，必须关闭 DMA。
   */
  __HAL_DMA_DISABLE(&hdma_usart2_tx);

  while ((hdma_usart2_tx.Instance->CCR & DMA_CCR_EN) != 0U) {
  }

  /*
   * 必须清除上一次传输的 TC、HT、TE 等 DMA 标志。
   */
  __HAL_DMA_CLEAR_FLAG(&hdma_usart2_tx,
                       __HAL_DMA_GET_GI_FLAG_INDEX(&hdma_usart2_tx));

  /*
   * CPAR正常情况下初始化一次即可。
   * 这里再次写入，方便排除初始化错误。
   */
  hdma_usart2_tx.Instance->CPAR = (uint32_t)&USART2->TDR;

  hdma_usart2_tx.Instance->CMAR = (uint32_t)&tx_frame;

  /*
   * 注意：
   * 只有DMA内存和外设数据宽度均为Byte时，
   * CNDTR才等于字节数量28。
   */
  hdma_usart2_tx.Instance->CNDTR = sizeof(JustFloatFrame_t);

  /*
   * 清除USART发送完成标志。
   * 新数据真正发送完后，TC才会重新置1。
   */
  USART2->ICR = USART_ICR_TCCF;

  /*
   * 保证CPU写入缓冲区的数据在DMA启动前完成。
   * STM32G431没有D-Cache，但保留此屏障更加稳妥。
   */
  __DMB();

  /*
   * 如果担心其他代码关闭了DMAT，可以再次确保使能。
   */
  SET_BIT(USART2->CR3, USART_CR3_DMAT);

  /*
   * 开启DMA，USART的TX请求会立即触发数据搬运。
   */
  __HAL_DMA_ENABLE(&hdma_usart2_tx);

  return 0;
}



void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef *huart,
    uint16_t Size)
{
    DebugConsole_OnRxEvent(huart, Size);
}

void HAL_UART_ErrorCallback(
    UART_HandleTypeDef *huart)
{
    DebugConsole_OnError(huart);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */