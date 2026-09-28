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

#include <stdint.h>
#include <string.h>
#include "debug_console.h"
#include "controller.h"

#include "motor_calibration.h"
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

/* BOOL接口使用uint32_t，避免把uint8_t强转成uint32_t指针。 */
static volatile uint32_t just_float_on_off = 0U;
/* 串口请求的速度环对称 Iq 限幅；主循环统一应用到 PI。 */
static volatile float iq_max = FOC_SPEED_PI_OUTPUT_MAX_DEFAULT;

/* DebugConsole 文本入队；USART2 TX DMA 由主循环统一调度。 */
#define DEBUG_TX_SIZE 4096U
static uint8_t debug_tx_ring[DEBUG_TX_SIZE];
static volatile uint16_t debug_tx_head, debug_tx_tail, debug_tx_dma_len;
static volatile uint32_t debug_tx_dropped;

static void DebugConsole_Tx(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U))
        return;

    /* 预留一个空位，整条消息入队，禁止覆盖 DMA 正在发送的数据。 */
    uint16_t used = (uint16_t)((debug_tx_head - debug_tx_tail) & (DEBUG_TX_SIZE - 1U));
    if (len > (DEBUG_TX_SIZE - 1U - used)) {
        debug_tx_dropped++;
        return;
    }

    uint16_t first = (uint16_t)(DEBUG_TX_SIZE - debug_tx_head);
    if (first > len)
        first = len;
    memcpy(&debug_tx_ring[debug_tx_head], data, first);
    if (len > first)
        memcpy(debug_tx_ring, data + first, len - first);
    debug_tx_head = (uint16_t)((debug_tx_head + len) & (DEBUG_TX_SIZE - 1U));
}

/* 仅主循环调用：等上一段完全发送后，启动下一段连续内存的 DMA。 */
static void DebugConsole_TxProcess(void)
{
    if (debug_tx_dma_len != 0U) {
        /* 普通 DMA 完成后 EN 可能仍为 1；剩余计数为 0 且串口 TC=1 才是真正发完。 */
        if ((hdma_usart2_tx.Instance->CNDTR != 0U) ||
            ((USART2->ISR & USART_ISR_TC) == 0U))
            return;
        __HAL_DMA_DISABLE(&hdma_usart2_tx);
        debug_tx_tail = (uint16_t)((debug_tx_tail + debug_tx_dma_len) & (DEBUG_TX_SIZE - 1U));
        debug_tx_dma_len = 0U;
    }

    if (debug_tx_head == debug_tx_tail) {
        if (debug_tx_dropped == 0U)
            return;
        debug_tx_dropped = 0U;
        static const uint8_t warning[] = "WARN: USART2 TX queue overflow\r\n";
        DebugConsole_Tx(warning, sizeof(warning) - 1U);
    }

    /* 与中断中的 JustFloat 共用 DMA，检查空闲和占用标志必须原子完成。 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (((USART2->ISR & USART_ISR_TC) == 0U) ||
        (hdma_usart2_tx.Instance->CNDTR != 0U)) {
        __set_PRIMASK(primask);
        return;
    }

    debug_tx_dma_len = (debug_tx_head > debug_tx_tail) ?
        (uint16_t)(debug_tx_head - debug_tx_tail) :
        (uint16_t)(DEBUG_TX_SIZE - debug_tx_tail);

    /* 禁止中断直到 DMA 配置完成，防止 JustFloat 中途改写同一通道。 */
    CLEAR_BIT(USART2->CR3, USART_CR3_DMAT);
    __HAL_DMA_DISABLE(&hdma_usart2_tx);
    __HAL_DMA_CLEAR_FLAG(&hdma_usart2_tx,
                         __HAL_DMA_GET_GI_FLAG_INDEX(&hdma_usart2_tx));
    hdma_usart2_tx.Instance->CPAR = (uint32_t)&USART2->TDR;
    hdma_usart2_tx.Instance->CMAR = (uint32_t)&debug_tx_ring[debug_tx_tail];
    hdma_usart2_tx.Instance->CNDTR = debug_tx_dma_len;
    USART2->ICR = USART_ICR_TCCF;
    SET_BIT(USART2->CR3, USART_CR3_DMAT);
    __DMB();
    __HAL_DMA_ENABLE(&hdma_usart2_tx);
    __set_PRIMASK(primask);
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*
 * 以下状态由主循环、FOC注入ADC中断和协议定时器共同访问；新增字段时
 * 必须明确访问上下文，避免在高频ISR中引入阻塞调用或不可原子共享。
 */
typedef struct {
  float data[6];
  uint32_t tail;
} JustFloatFrame_t;
_Static_assert(sizeof(JustFloatFrame_t) == 28U, "VOFA JustFloat frame must be 28 bytes");

// Cortex-M4 是小端模式： 0x7F800000 在内存中排列为 00 00 80 7F
static JustFloatFrame_t tx_frame __attribute__((aligned(4)));

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/** @brief 配置系统时钟：HSE晶振经PLL倍频输出168MHz SYSCLK/FDCAN时钟。 */
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/** @brief 启动运放自校准和ADC自校准，完成后等待运放稳定。 */
void FOC_ADC_AND_OPAMP_Calibration_Start(void);

/** @brief 初始化VOFA+ JustFloat帧发送：配置USART2发送DMA通道指向帧缓冲区。 */
void JustFloat_Init(void);

/* ======================== 电流零偏校准 ======================== */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 串口状态查询只在主循环执行，不占用 25 kHz ADC 中断。 */
static void MotorDebug_Status(int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  DebugConsole_Printf("MOTOR state=%u cal=%u MOE=%u ARR=%lu CCR4=%lu\r\n",
                      (unsigned)foc_motor_state, (unsigned)foc.calibration.calibrated,
                      (unsigned)((TIM1->BDTR & TIM_BDTR_MOE) != 0U),
                      (unsigned long)TIM1->ARR, (unsigned long)TIM1->CCR4);
  DebugConsole_Printf("SPEED target=%.1f rpm=%.1f mode=%s iq_max=%.2fA\r\n",
                      motor_control.speed_ref_rpm, foc.observer.state.speed_rpm,
                      motor_control.speed_loop_enable ? "speed" : "current",
                      motor_control.speed_pi.output_max);
  DebugConsole_Printf("CURRENT Id=%.3f/%.3f Iq=%.3f/%.3f Ud=%.3f Uq=%.3f Vbus=%.2f\r\n",
                      motor_control.id_ref, motor_control.id_feedback,
                      motor_control.iq_ref_active, motor_control.iq_feedback,
                      motor_control.ud_output, motor_control.uq_output, foc.state.vbus);
  DebugConsole_Printf("PI speed=%.5g/%.5g id=%.5g/%.5g iq=%.5g/%.5g\r\n",
                      motor_control.speed_pi.kp, motor_control.speed_pi.ki,
                      motor_control.id_pi.kp, motor_control.id_pi.ki,
                      motor_control.iq_pi.kp, motor_control.iq_pi.ki);
  DebugConsole_Printf("SAT speed=%d id=%d iq=%d phase=%.3f flux=%.6f\r\n",
                      (int)motor_control.speed_pi.saturation,
                      (int)motor_control.id_pi.saturation,
                      (int)motor_control.iq_pi.saturation,
                      foc.observer.state.phase_raw, foc.observer.state.psi_mag);
  DebugConsole_Printf("PWM CCR=%lu,%lu,%lu CCER=0x%08lX\r\n",
                      (unsigned long)TIM1->CCR1, (unsigned long)TIM1->CCR2,
                      (unsigned long)TIM1->CCR3, (unsigned long)TIM1->CCER);
}
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
  /*
   * 这里是应用集成初始化区：只启动业务模块，不重复改写CubeMX外设配置。
   * 任一校准、协议或调试通道初始化失败都进入Error_Handler，禁止带故障运行。
   */
  DWT_Delay_Init();
  CORDIC_SinCos_RegisterConfig();
  JustFloat_Init();
  /*
   * 初始化电流环和速度环。
   * 电流环参数仍为当前已经跑通的参数：
   * Id: Kp=0.2, Ki=100, PI输出默认±20V
   * Iq: Kp=0.5, Ki=300, PI输出默认±20V
   */
  FOC_Control_Init(&motor_control, foc.timer.Ts);

  if (DebugConsole_Init(&huart2, DebugConsole_Tx) != HAL_OK) {
    Error_Handler();
  }


  /* 速度模式为主：Iq 命令和模式切换暂不暴露，电流模式留待后续。 */
  DebugConsole_RegisterF32("speed", &motor_control.speed_ref_rpm,
    -10000.0f, 10000.0f, false);
  DebugConsole_RegisterF32("speed_kp", &motor_control.speed_pi.kp,
    0.0f, 1.0f, false);
  DebugConsole_RegisterF32("speed_ki", &motor_control.speed_pi.ki,
    0.0f, 100.0f, false);
  DebugConsole_RegisterF32("iq_max", &iq_max, 0.0f, 10.0f, false);
  DebugConsole_RegisterBool("just_float", &just_float_on_off, false);

  DebugConsole_RegisterF32("id", &motor_control.id_ref,
    -8.0f, 8.0f, false);
  DebugConsole_RegisterF32("id_kp", &motor_control.id_pi.kp,
    0.0f, 10.0f, false);
  DebugConsole_RegisterF32("id_ki", &motor_control.id_pi.ki,
    0.0f, 5000.0f, false);
  DebugConsole_RegisterF32("iq_kp", &motor_control.iq_pi.kp,
    0.0f, 10.0f, false);
  DebugConsole_RegisterF32("iq_ki", &motor_control.iq_pi.ki,
    0.0f, 5000.0f, false);

  DebugConsole_RegisterF32("rpm", &foc.observer.state.speed_rpm,
    -100000.0f, 100000.0f, true);
  DebugConsole_RegisterF32("iq_target", &motor_control.iq_ref_active,
    -10.0f, 10.0f, true);
  DebugConsole_RegisterF32("vbus", &foc.state.vbus,
    0.0f, 70.0f, true);
  DebugConsole_RegisterCommand("status", MotorDebug_Status,
    "status: show motor, PI and PWM state");

  FOC_ADC_AND_OPAMP_Calibration_Start();

  // FOC_Iabc_Calibration();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /*
   * 注入ADC中断负责FOC实时环；主循环只处理低速规则组采样、协议和调试命令，
   * 不应在此处加入会长时间关闭中断或阻塞串口的操作。
   */
  if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK) {
    Error_Handler();
  }

  /* 上电时仅启动TIM1 CH4内部ADC触发，不使能三相功率输出。
   * 1000次注入采样约需40 ms；完成零偏校准后关闭触发，保持电机停机。 */
  if ((TIM1->CCER & (TIM_CCER_CC1E | TIM_CCER_CC1NE |
                      TIM_CCER_CC2E | TIM_CCER_CC2NE |
                      TIM_CCER_CC3E | TIM_CCER_CC3NE)) != 0U) {
    Error_Handler();
  }
  TIM1->CCR4 = foc.timer.adc_trigger;
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  {
    uint32_t calibration_start_tick = HAL_GetTick();
    while (foc.calibration.calibrated == 0U) {
      if ((HAL_GetTick() - calibration_start_tick) >= 200U) {
        (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
        Error_Handler();
      }
    }
  }
  if (HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }



  /* 仅BRS高速数据段需要TDC；当前1M无BRS测试不启用。 */
#if MOTOR_PROTOCOL_CANFD_BRS_ENABLED
if (HAL_FDCAN_ConfigTxDelayCompensation(
        &hfdcan1,
        hfdcan1.Init.DataPrescaler *
        hfdcan1.Init.DataTimeSeg1,
        0U) != HAL_OK)
{
    Error_Handler();
}

 if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
{
    Error_Handler();
}
#endif










  /*
   * ADC/运放校准完成后再启动CAN反馈定时器，避免100 us协议中断干扰校准。
   * 电机运行、转速设定、反馈和ENTER_BOOT共用一套CAN协议。
   */
  if (MotorProtocol_Init(&hfdcan1, &motor_control) != HAL_OK) {
    Error_Handler();
  }





  uint32_t led_task_tick = HAL_GetTick();
  while (1) {
    ADC_Regular_Service(HAL_GetTick());
    MotorProtocol_Process();
    DebugConsole_Process();
    /* 串口修改的单个正数限幅在这里一次性应用，保持正反转和积分限幅对称。 */
    if (motor_control.speed_pi.output_max != iq_max) {
      uint32_t primask = __get_PRIMASK();
      __disable_irq();
      PI_Controller_SetLimits(&motor_control.speed_pi, -iq_max, iq_max);
      __set_PRIMASK(primask);
    }
    DebugConsole_LogProcess(); // 主循环分批格式化各模块中断提交的数值。
    MotorCalibration_DebugProcess();
    DebugConsole_TxProcess(); // USART2 DMA 后台发送，不等待串口。

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

/*
 * 该回调运行在PWM同步的ADC注入中断上下文，必须保持确定性：不使用HAL延时、
 * 不等待外设完成，不调用会阻塞的协议/日志接口；慢速工作下放到主循环或TIM6。
 */
/** @brief ADC注入组转换完成回调（每个PWM周期触发）：校准阶段累加零偏，运行阶段执行FOC电流环闭环控制。 */
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

    if (foc_motor_state == FOC_MOTOR_CALIBRATION) {
      MotorCalibration_Run(foc.state.i_abc.a,
                           foc.state.i_abc.b,
                           foc.state.i_abc.c,
                           foc.state.vbus);
      return;
    }

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

    /* FOC实时估算母线电流；100 Hz CAN反馈读取低通滤波后的快照。 */
    FOC_UpdateBusCurrentEstimate(&foc);

    
    /* 辨识分支已提前 return，正常更新三相 PWM 比较值。 */
    TIM1->CCR1 = foc.svpwm.ccr_a;
    TIM1->CCR2 = foc.svpwm.ccr_b;
    TIM1->CCR3 = foc.svpwm.ccr_c;








    /* JustFloat 直接输出 6 个 float + 帧尾；文本/日志待发时不启动新帧。 */
    if ((just_float_on_off != 0U) &&
        !DebugConsole_LogPending() &&
        (debug_tx_head == debug_tx_tail) &&
        (debug_tx_dma_len == 0U) &&
        ((USART2->ISR & USART_ISR_TC) != 0U) &&
        (hdma_usart2_tx.Instance->CNDTR == 0U)) {
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
  just_float_on_off = 0U; /* 上电默认关闭，串口 set just_float 1 手动启用。 */
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
  /* 文本待发或 DMA 正在发送文本时，让出 USART2 TX 通道。 */
  if ((just_float_on_off == 0U) ||
      (debug_tx_dma_len != 0U) ||
      (debug_tx_head != debug_tx_tail) ||
      DebugConsole_LogPending() ||
      ((USART2->ISR & USART_ISR_TC) == 0U) ||
      (hdma_usart2_tx.Instance->CNDTR != 0U)) {
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

  /* 文本 DMA 会修改 CMAR，发送 JustFloat 前必须恢复帧缓冲区地址。 */
  hdma_usart2_tx.Instance->CMAR = (uint32_t)&tx_frame;
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
