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

#include "arm_math.h"
#include "bsp_dwt.h"
#include "foc_math.h"
#include "stdio.h"
#include <math.h>
#include <stdint.h>
#include "debug_console.h"
#include "controller.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

static FOC_Control_t motor_control;





#define OBSERVER_LOCK_SAMPLE_COUNT      2000U
#define VBUS_DIVIDER_GAIN               ((100.0f + 4.7f) / 4.7f)
/*
 * 角度偏移释放速度(rad/s)
 * 10rad/s × 40us ≈ 0.0004rad/周期
 * 约2500周期释放完1rad偏移
 */
#define HANDOVER_OFFSET_RATE_RAD_S       10.0f



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


static void DebugConsole_Tx(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(
        &huart1,
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

uint16_t as5600_raw = 0U;
float as5600_elec_rad = 0.0f;

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
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_CORDIC_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
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



  /* 临时CAN FD发波测试：1 Mbps仲裁段，5 Mbps数据段。 */
  FDCAN_TxHeaderTypeDef can_tx_header = {0};
  uint8_t can_tx_data[12] = {
      0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C
  };

  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
  hfdcan1.Init.DataPrescaler = 2U;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }

  can_tx_header.Identifier = 0x123U;
  can_tx_header.IdType = FDCAN_STANDARD_ID;
  can_tx_header.TxFrameType = FDCAN_DATA_FRAME;
  can_tx_header.DataLength = FDCAN_DLC_BYTES_12;
  can_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  can_tx_header.BitRateSwitch = FDCAN_BRS_ON;
  can_tx_header.FDFormat = FDCAN_FD_CAN;
  can_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  can_tx_header.MessageMarker = 0U;

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
  uint8_t pData[] = "Hello, World!\r\n";

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  while (1) {
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0U) {
      if (HAL_FDCAN_AddMessageToTxFifoQ(
              &hfdcan1, &can_tx_header, can_tx_data) != HAL_OK) {
        Error_Handler();
      }
    }
    HAL_UART_Transmit(&huart1, pData, sizeof(pData) - 1, HAL_MAX_DELAY);
    HAL_Delay(10U);
  }

  if (DebugConsole_Init(&huart1, DebugConsole_Tx) != HAL_OK) {
    Error_Handler();
  }


  DebugConsole_RegisterF32("id", &motor_control.id_ref,
    -8.0f, 8.0f, false);
  DebugConsole_RegisterF32("iq", &motor_control.iq_ref,
    -8.0f, 8.0f, false);
  /* 速度模式参数：speed单位rpm，speed_en为0/1。 */
  DebugConsole_RegisterF32("speed", &motor_control.direction.speed_command_rpm,
    -15000.0f, 15000.0f, false);
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

  while (1) {

    // ADC1 采样母线电压
    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK) {
      uint16_t adc_value = (uint16_t)HAL_ADC_GetValue(&hadc1);

      foc.state.vbus =
          (float)adc_value * VBUS_DIVIDER_GAIN * 3.3f / 4096.0f;
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
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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
  uint16_t adc_a;
  uint16_t adc_b;
  uint16_t adc_c;

  static uint32_t observer_lock_count = 0U;
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
      float theta_raw_rad;
      float phase_raw_pll_error;
      float phase_open_raw_error;
      float speed_error;
      float target_omega_e;
      int32_t open_loop_step_q32;

      uint8_t psi_valid;
      uint8_t observer_locked;
      uint32_t observer_lock_sample_count;

      FOC_State_Count++;

      if (motor_control.direction.open_loop_initialized == 0U) {
        FOC_DirectionControl_PrepareOpenLoop(&motor_control);
        observer_lock_count = 0U;
        observer_offset_valid = 0U;
      }

      open_loop_step_q32 =
          FOC_DirectionControl_UpdateOpenLoop(&motor_control);

      target_omega_e =
          OPEN_LOOP_TARGET_OMEGA_E *
          (float)motor_control.direction.open_loop_direction;

      /*
       * theta_step和Uq必须保持同一转矩方向。
       * 正转：step>0，Uq>0
       * 反转：step<0，Uq<0
       */
      FOC_Open_Loop(
          0.0f,
          2.0f*(float)motor_control.direction.open_loop_direction,
          (uint32_t)open_loop_step_q32);

      /*
       * 当前开环坐标系中的Id、Iq。
       */
      FOC_Park(&foc.state.i_alpha_beta, &foc_sin_cos, &foc.state.i_dq);

      theta_open_rad =
          FOC_WrapToPiFast((float)foc.state.theta_q31 * Q32_TO_RAD_F);

      theta_raw_rad = FOC_WrapToPiFast(foc.observer.state.phase_raw);

      /*
       * PLL是否真正跟随原始磁链角。
       */
      phase_raw_pll_error =
          fabsf(FOC_WrapToPiFast(theta_raw_rad - foc.observer.state.pll_phase));

      /*
       * 观测磁链角是否与开环旋转方向大致一致。
       *
       * 正常同步运行时，负载角应小于90°左右。
       */
      phase_open_raw_error =
          fabsf(FOC_WrapToPiFast(theta_raw_rad - theta_open_rad));

      speed_error =
          fabsf(foc.observer.state.pll_omega_e - target_omega_e);

      psi_valid = isfinite(foc.observer.state.psi_mag) &&
                  (foc.observer.state.psi_mag >= foc.observer.config.psi_min) &&
                  (foc.observer.state.psi_mag <= foc.observer.config.psi_max);

      observer_lock_sample_count =
          (motor_control.direction.state == FOC_REVERSAL_RESTART)
              ? FOC_REVERSAL_LOCK_SAMPLE_COUNT
              : OBSERVER_LOCK_SAMPLE_COUNT;

      observer_locked =
                        (open_loop_step_q32 ==
                         (int32_t)OPEN_LOOP_TARGET_STEP_Q32 *
                         (int32_t)motor_control.direction.open_loop_direction) &&
                        (psi_valid != 0U) &&

                        /* 方向必须与当前开环方向一致 */
                        ((float)motor_control.direction.open_loop_direction *
                         foc.observer.state.pll_omega_e > 0.0f) &&

                        /* 速度误差小于目标速度30% */
                        (speed_error < OPEN_LOOP_TARGET_OMEGA_E * 0.30f) &&

                        /* PLL与raw误差小于15° */
                        (phase_raw_pll_error < 15.0f * FOC_PI / 180.0f) &&

                        /* 观测角与开环角相差不超过90° */
                        (phase_open_raw_error < 90.0f * FOC_PI / 180.0f);

      if (observer_locked != 0U) {
        if (observer_lock_count < OBSERVER_LOCK_SAMPLE_COUNT) {
          observer_lock_count++;
        }
      } else {
        observer_lock_count = 0U;
      }

      /*
       * 不再按固定1秒强制切换。
       * 必须连续锁定80ms才切换。
       */
      if (observer_lock_count >= observer_lock_sample_count) {
        /*
         * 注意：闭环使用pll_phase，
         * 所以偏移必须相对于pll_phase计算。
         */
        observer_control_offset =
            FOC_WrapToPiFast(theta_open_rad - foc.observer.state.pll_phase);

        observer_offset_valid = 1U;
        observer_lock_count = 0U;
        motor_control.direction.open_loop_initialized = 0U;

        /*
         * 切换期间关闭速度环。
         */
        motor_control.speed_loop_enable = 0U;

        /*
         * 如果开环阶段仍然是固定Uq，
         * 就保留这次电流PI预装载。
         */
        FOC_Control_PreloadClosedLoop(
            &motor_control, foc.state.u_dq.d, foc.state.u_dq.q,
            foc.state.i_dq.d, foc.state.i_dq.q, foc.observer.state.speed_rpm);

        foc_motor_state = FOC_MOTOR_TRANSITION;
      }

      break;
    }

    case FOC_MOTOR_TRANSITION: {
      float phase_control;
      float offset_step;
      uint32_t phase_q31;

      if (observer_offset_valid == 0U) {
        foc_motor_state = FOC_MOTOR_OPEN_LOOP;
        break;
      }

      /*
       * 第一拍严格满足：
       *
       * pll_phase + offset = theta_open
       *
       * 所以不会产生角度阶跃。
       */
      phase_control = FOC_WrapToPiFast(foc.observer.state.pll_phase +
                                       observer_control_offset);

      phase_q31 = (uint32_t)CORDIC_RadToQ31_WrappedFast(phase_control);

      CORDIC_SinCos_FastF32(phase_q31, &observer_sin_cos.sin,
                            &observer_sin_cos.cos);

      FOC_Park(&foc.state.i_alpha_beta, &observer_sin_cos, &foc.state.i_dq);

      /*
       * 转换期间只运行电流环，不运行速度环。
       */
      motor_control.speed_loop_enable = 0U;

      FOC_Control_Run(&motor_control, foc.state.i_dq.d, foc.state.i_dq.q,
                      foc.observer.state.speed_rpm, &foc.state.u_dq.d,
                      &foc.state.u_dq.q);

      FOC_InvPark(&foc.state.u_dq, &observer_sin_cos, &foc.state.u_alpha_beta);

      FOC_InvClarke(&foc.state.u_alpha_beta, &foc.state.u_abc);

      FOC_SVPWM_Run(&foc.state.u_abc, foc.state.vbus, &foc.timer, &foc.svpwm);

      /*
       * 用rad/s定义释放速度，
       * 不要使用没有物理意义的固定魔数。
       */
      if (motor_control.direction.state == FOC_REVERSAL_RESTART) {
        offset_step =
            FOC_REVERSAL_HANDOVER_RATE_RAD_S * foc.timer.Ts;
      } else {
        offset_step = HANDOVER_OFFSET_RATE_RAD_S * foc.timer.Ts;
      }

      if (observer_control_offset > offset_step) {
        observer_control_offset -= offset_step;
      } else if (observer_control_offset < -offset_step) {
        observer_control_offset += offset_step;
      } else {
        observer_control_offset = 0.0f;

        /*
         * 进入完全闭环。
         */
        foc_motor_state = FOC_MOTOR_CLOSED_LOOP;

        FOC_DirectionControl_ClosedLoopEntered(
            &motor_control, foc.observer.state.speed_rpm);
      }

      break;
    }

    case FOC_MOTOR_CLOSED_LOOP: {
      float phase_control;
      uint32_t phase_q31;

      if (FOC_DirectionControl_RunClosedLoop(
              &motor_control, foc.observer.state.speed_rpm,
              foc.timer.Ts) != 0U) {
        /*
         * 无感观测器在零速附近不可靠。
         * 从最后一个可信磁链角重新进入反向开环启动。
         */
        foc.state.theta_q31 =
            (uint32_t)CORDIC_RadToQ31_WrappedFast(
                foc.observer.state.phase_raw);

        /*
         * PLL上一方向的积分和正速度不能带入反向开环。
         * 只复位PLL，保留磁链观测器x/psi状态。
         */
        Observer_PLL_ResetToPhase(
            &foc.observer, foc.observer.state.phase_raw);

        observer_lock_count = 0U;
        observer_offset_valid = 0U;

        foc_motor_state = FOC_MOTOR_OPEN_LOOP;
        break;
      }

      phase_control = FOC_WrapToPiFast(foc.observer.state.phase_raw);

      phase_q31 = (uint32_t)CORDIC_RadToQ31_WrappedFast(phase_control);

      CORDIC_SinCos_FastF32(phase_q31, &observer_sin_cos.sin,
                            &observer_sin_cos.cos);

      FOC_Park(&foc.state.i_alpha_beta, &observer_sin_cos, &foc.state.i_dq);

      FOC_Control_Run(&motor_control, foc.state.i_dq.d, foc.state.i_dq.q,
                      foc.observer.state.speed_rpm, &foc.state.u_dq.d,
                      &foc.state.u_dq.q);

      FOC_InvPark(&foc.state.u_dq, &observer_sin_cos, &foc.state.u_alpha_beta);

      FOC_InvClarke(&foc.state.u_alpha_beta, &foc.state.u_abc);

      FOC_SVPWM_Run(&foc.state.u_abc, foc.state.vbus, &foc.timer, &foc.svpwm);

      break;
    }
    }

    TIM1->CCR1 = foc.svpwm.ccr_a;
    TIM1->CCR2 = foc.svpwm.ccr_b;
    TIM1->CCR3 = foc.svpwm.ccr_c;





    /*
     * UART忙时不要先计算6个实参再进入发送函数返回。
     * 2 Mbps波特率下仍先判断TC，避免DMA忙时计算无用的发送数据。
     * 外层先判断TC可减少函数调用、周期差计算和浮点角度换算。
     */
    if ((just_float_on_off != 0U) &&
        ((USART1->ISR & USART_ISR_TC) != 0U)) {
      Fast_Send_6Floats(
          foc.state.i_abc.a,
          foc.observer.state.psi_alpha,
          foc.observer.state.psi_beta,
          foc.observer.state.speed_rpm,
          foc.observer.state.phase_raw * RAD_TO_DEG_F,
          foc.state.vbus);
    }
  }
}

int _write(int file, char *ptr, int len) {
  (void)file;

  if ((ptr == NULL) || (len <= 0)) {
    return 0;
  }

  if (HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len,
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
 * @brief 初始化 JustFloat 和 USART1 TX DMA
 */
void JustFloat_Init(void) {
  /* VOFA+ JustFloat 帧尾 */
  tx_frame.tail = 0x7F800000UL;
  just_float_on_off = 1U;
  /* 暂时关闭 USART DMA 发送请求 */
  CLEAR_BIT(USART1->CR3, USART_CR3_DMAT);

  /* 禁用 DMA 通道 */
  __HAL_DMA_DISABLE(&hdma_usart1_tx);

  while ((hdma_usart1_tx.Instance->CCR & DMA_CCR_EN) != 0U) {
  }

  /* 清除该 DMA 通道的全部标志 */
  __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx,
                       __HAL_DMA_GET_GI_FLAG_INDEX(&hdma_usart1_tx));

  /*
   * 关键：DMA 外设地址必须是 USART1 发送数据寄存器
   */
  hdma_usart1_tx.Instance->CPAR = (uint32_t)&USART1->TDR;

  /*
   * 内存地址指向发送缓冲区
   */
  hdma_usart1_tx.Instance->CMAR = (uint32_t)&tx_frame;

  hdma_usart1_tx.Instance->CNDTR = 0U;

  /*
   * 关键：允许 USART1 产生 TX DMA 请求
   */
  SET_BIT(USART1->CR3, USART_CR3_DMAT);
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
  if ((USART1->ISR & USART_ISR_TC) == 0U) {
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
  __HAL_DMA_DISABLE(&hdma_usart1_tx);

  while ((hdma_usart1_tx.Instance->CCR & DMA_CCR_EN) != 0U) {
  }

  /*
   * 必须清除上一次传输的 TC、HT、TE 等 DMA 标志。
   */
  __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx,
                       __HAL_DMA_GET_GI_FLAG_INDEX(&hdma_usart1_tx));

  /*
   * CPAR和CMAR已在JustFloat_Init()中固定配置，
   * 每次发送只需重新装载传输数量。
   */
  hdma_usart1_tx.Instance->CNDTR = sizeof(JustFloatFrame_t);

  /*
   * 清除USART发送完成标志。
   * 新数据真正发送完后，TC才会重新置1。
   */
  USART1->ICR = USART_ICR_TCCF;

  /*
   * 保证CPU写入缓冲区的数据在DMA启动前完成。
   * STM32G431没有D-Cache，但保留此屏障更加稳妥。
   */
  __DMB();

  /*
   * DMAT已在JustFloat_Init()中保持使能。
   * 开启DMA后USART TX请求立即开始搬运。
   */
  __HAL_DMA_ENABLE(&hdma_usart1_tx);

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
