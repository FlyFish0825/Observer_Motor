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
#include "board_config.h"
#include "arm_math.h"
#include "bsp_dwt.h"
#include "foc_math.h"
#include "stdio.h"
#include <math.h>
#include <stdint.h>
#include "debug_console.h"
#include "controller.h"

#include "app_memory.h"
#include "motor_protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


//时钟选择，内部还是外部 
// 1-表示内部   0-表示外部
#define USE_INTERNAL_CLOCK 1  



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

static FOC_Control_t motor_control;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/**
 * @brief 通过调试串口发送控制台格式化后的数据。
 * @note 该回调由调试控制台调用，统一使用USART2阻塞发送，避免业务模块直接依赖串口句柄。
 */
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

/* BOOL接口使用uint32_t，避免把uint8_t强转成uint32_t指针。 */
static volatile uint32_t just_float_on_off = 1U;

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
/**
 * @brief 应用程序入口，完成硬件初始化、FOC启动和后台协议处理。
 * @note CubeMX生成的外设初始化保持原样，项目业务逻辑集中在初始化后的主循环中。
 */
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
  DWT_Delay_Init();
  CORDIC_SinCos_RegisterConfig();
  JustFloat_Init();
  /*
   * 初始化电流环和速度环。
   * 电流环参数仍为当前已经跑通的参数：
   * Id: Kp=0.2, Ki=100, 输出-7~7V
   * Iq: Kp=0.5, Ki=300, 输出-8~8V
   */
  FOC_Control_Init(&motor_control, foc.timer.Ts);

  if (DebugConsole_Init(&huart2, DebugConsole_Tx) != HAL_OK) {
    Error_Handler();
  }


  DebugConsole_RegisterF32("id", &motor_control.id_ref,
    -8.0f, 8.0f, false);
  DebugConsole_RegisterF32("iq", &motor_control.iq_ref,
    -8.0f, 8.0f, false);
  /* 速度模式参数：speed单位rpm，speed_en为0/1。 */
  DebugConsole_RegisterF32("speed", &motor_control.speed_ref_rpm,
    -10000.0f, 10000.0f, false);
  DebugConsole_RegisterF32("speed_kp", &motor_control.speed_pi.kp,
    0.0f, 1.0f, false);
  DebugConsole_RegisterF32("speed_ki", &motor_control.speed_pi.ki,
    0.0f, 100.0f, false);
  DebugConsole_RegisterBool("speed_en",
    &motor_control.speed_loop_enable, false);
  DebugConsole_RegisterBool("just_float",
    &just_float_on_off, false);

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

  /*
   * ADC/运放校准完成后再启动CAN反馈定时器，避免100 us协议中断干扰校准。
   * 电机运行、转速设定、反馈和ENTER_BOOT共用一套CAN协议。
   */
  if (MotorProtocol_Init(&hfdcan1, &motor_control) != HAL_OK) {
    Error_Handler();
  }

  uint32_t slow_task_tick = HAL_GetTick();
  while (1) {
    MotorProtocol_Process();
    DebugConsole_Process();

    /* 母线电压和指示灯保持500ms周期，但不阻塞CAN协议处理。 */
    if ((HAL_GetTick() - slow_task_tick) >= 500U) {
      slow_task_tick = HAL_GetTick();

      /* ADC1规则组依次采集母线电压和MCU内部温度传感器。 */
      if (HAL_ADC_Start(&hadc1) == HAL_OK) {
        if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK) {
          uint16_t adc_vbus = (uint16_t)HAL_ADC_GetValue(&hadc1);
          foc.state.vbus = (float)adc_vbus * 26.0f * 3.3f / 4096.0f;
        }

        if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK) {
          uint16_t adc_temperature = (uint16_t)HAL_ADC_GetValue(&hadc1);
          int32_t temperature_c = __HAL_ADC_CALC_TEMPERATURE(
              3300U, adc_temperature, ADC_RESOLUTION_12B);
          foc.state.temperature_c = (float)temperature_c;
        }
        HAL_ADC_Stop(&hadc1);
      }

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

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {

  static uint16_t calibration_count = 0;
  uint32_t DWT_Cycle_Count; // 获取当前的DWT计数器值
  uint16_t adc_a;
  uint16_t adc_b;
  uint16_t adc_c;

  static uint32_t FOC_State_Count = 0U;

  static float observer_control_offset = 0.0f;
  static uint8_t observer_offset_valid = 0U;

  if (hadc->Instance != ADC1) {
    return; // 只处理 ADC1 的注入转换完成事件
  }

  if (foc.calibration.calibrated == 0) {

    calibration_count++;
    // 校准阶段，计算零偏

    foc.calibration.ia_offset +=
        HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    foc.calibration.ib_offset +=
        HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    foc.calibration.ic_offset +=
        HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

    if (calibration_count >= CURRENT_OFFSET_SAMPLE_NUM) {
      // 校准完成，计算平均值
      foc.calibration.ia_offset /= (float)CURRENT_OFFSET_SAMPLE_NUM;
      foc.calibration.ib_offset /= (float)CURRENT_OFFSET_SAMPLE_NUM;
      foc.calibration.ic_offset /= (float)CURRENT_OFFSET_SAMPLE_NUM;
      foc.calibration.calibrated = 1; // 设置校准完成标志
    }

    // 校准完成
  } else {
    

    /*
     * 高频ISR直接读取注入数据寄存器。
     * ADC注入Rank与工程配置固定：ADC1 JDR1=Ia，ADC2 JDR1=Ib，ADC1 JDR2=Ic。
     */
    DWT_Cycle_Count = DWT->CYCCNT;

    adc_a = (uint16_t)ADC1->JDR1;
    adc_b = (uint16_t)ADC2->JDR1;
    adc_c = (uint16_t)ADC1->JDR2;

    FOC_Get_Iabc(&foc, adc_a, adc_b, adc_c);

    FOC_Clarke(&foc.state.i_abc, &foc.state.i_alpha_beta);

   
    Observer_Input_t obs_in;
    // SVPWM输出的真实占空比
    obs_in.duty_a = foc.svpwm.duty_a;
    obs_in.duty_b = foc.svpwm.duty_b;
    obs_in.duty_c = foc.svpwm.duty_c;
    // 母线电压
    obs_in.vbus = foc.state.vbus;
    // 电流
    obs_in.i_alpha = foc.state.i_alpha_beta.alpha;
    obs_in.i_beta = foc.state.i_alpha_beta.beta;

    Observer_Run(&foc.observer, &obs_in);


    switch (foc_motor_state) {

    case FOC_MOTOR_IDLE:
      break;

    case FOC_MOTOR_OPEN_LOOP: {
      float theta_open_rad;
      float theta_obs_rad;

      FOC_State_Count++;

      FOC_Open_Loop(0.0f, 1.0f);

      /*
       * 开环阶段的Id/Iq也使用开环角度计算。
       * 这样切换前的电流反馈与当前实际输出坐标系一致。
       */
      FOC_Park(&foc.state.i_alpha_beta, &foc_sin_cos, &foc.state.i_dq);

      if (FOC_State_Count >= 25000U) {
        theta_open_rad =
            FOC_WrapToPi((float)foc.state.theta_q31 * Q32_TO_RAD_F);

        theta_obs_rad = FOC_WrapToPi(foc.observer.state.phase_raw);

        /* 保存当前开环控制角与观测器角的关系。 */
        observer_control_offset = FOC_WrapToPi(theta_open_rad - theta_obs_rad);

        observer_offset_valid = 1U;
        FOC_State_Count = 0U;

        /*
         * 一次预装载完成Id、Iq电流环；
         * 若提前启用了速度模式，也会预装载速度环。
         */
        FOC_Control_PreloadClosedLoop(
            &motor_control,
            foc.state.u_dq.d,
            foc.state.u_dq.q,
            foc.state.i_dq.d,
            foc.state.i_dq.q,
            foc.observer.state.speed_rpm);

        foc_motor_state = FOC_MOTOR_CLOSED_LOOP;
      }

      break;
    }

    case FOC_MOTOR_CLOSED_LOOP: {
      float phase_control;
      uint32_t phase_q31;

      if (observer_offset_valid == 0U) {
        foc_motor_state = FOC_MOTOR_OPEN_LOOP;
        break;
      }

      // /* 逐步释放开环切换时保存的角度偏移。 */
      // if (observer_control_offset > 0.001f) {
      //   observer_control_offset -= 0.0001f;
      // } else if (observer_control_offset < -0.001f) {
      //   observer_control_offset += 0.0001f;
      // } else {
      //   observer_control_offset = 0.0f;
      // }
      // observer_control_offset = 0.0f;
      // phase_control = FOC_WrapToPiFast(
      //     foc.observer.state.pll_phase + observer_control_offset);


      phase_control = FOC_WrapToPiFast(
      foc.observer.state.pll_phase);

      phase_q31 =
          (uint32_t)CORDIC_RadToQ31_WrappedFast(phase_control);

      CORDIC_SinCos_FastF32(phase_q31, &observer_sin_cos.sin,
                            &observer_sin_cos.cos);

      /*
       * Park、控制器、反Park严格使用同一个phase_control。
       */
      FOC_Park(&foc.state.i_alpha_beta, &observer_sin_cos,
               &foc.state.i_dq);

      FOC_Control_Run(
          &motor_control,
          foc.state.i_dq.d,
          foc.state.i_dq.q,
          foc.observer.state.speed_rpm,
          &foc.state.u_dq.d,
          &foc.state.u_dq.q);

      FOC_InvPark(&foc.state.u_dq, &observer_sin_cos,
                  &foc.state.u_alpha_beta);

      FOC_InvClarke(&foc.state.u_alpha_beta, &foc.state.u_abc);

      FOC_SVPWM_Run(&foc.state.u_abc, foc.state.vbus,
                    &foc.timer, &foc.svpwm);

      break;
    }
    }

    
    TIM1->CCR1 = foc.svpwm.ccr_a;
    TIM1->CCR2 = foc.svpwm.ccr_b;
    TIM1->CCR3 = foc.svpwm.ccr_c;









    /*
     * UART忙时不要先计算6个实参再进入发送函数返回。
     * 115200波特率下DMA绝大多数电流环周期都处于忙状态，
     * 外层先判断TC可减少函数调用、周期差计算和浮点角度换算。
     */
    if ((just_float_on_off != 0U) &&
        ((USART2->ISR & USART_ISR_TC) != 0U)) {
      Fast_Send_6Floats(
          foc.state.i_dq.d,
          foc.state.i_dq.q,
          foc.observer.state.speed_rpm,
          foc.observer.state.psi_mag,
          foc.observer.state.phase_raw * RAD_TO_DEG_F,
          foc.observer.state.pll_phase * RAD_TO_DEG_F);
    }
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
  just_float_on_off = 1U;
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
   * CPAR和CMAR已在JustFloat_Init()中固定配置，
   * 每次发送只需重新装载传输数量。
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
   * DMAT已在JustFloat_Init()中保持使能。
   * 开启DMA后USART TX请求立即开始搬运。
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
