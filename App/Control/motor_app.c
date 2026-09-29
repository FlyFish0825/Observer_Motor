/**
 * @file motor_app.c
 * @brief 电机应用层：控制器、ADC实时控制、调试与USART2 DMA。
 * @note 保留main原有控制算法、辨识提前返回和串口发送仲裁。
 */
#include "motor_app.h"
#include "adc.h"
#include "bsp_dwt.h"
#include "cordic.h"
#include "debug_console.h"
#include "foc_math.h"
#include "main.h"
#include "motor_calibration.h"
#include "motor_protocol.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "arm_math.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static FOC_Control_t motor_control;

/* BOOL接口使用uint32_t，避免把uint8_t强转成uint32_t指针。 */
static volatile uint32_t just_float_on_off = 0U;
/* 串口请求的速度环对称 Iq 限幅；主循环统一应用到 PI。 */
static volatile float iq_max = FOC_SPEED_PI_OUTPUT_MAX_DEFAULT;
/* PC10/PC11/PC13事件位；EXTI只关MOE并置位，主循环完成HAL操作。 */
static volatile uint16_t motor_button_pending;

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

/* rs / rs stop：与PC13按键使用同一Rs启动顺序，便于COM27自动测试。 */
static void MotorDebug_Rs(int argc, char *argv[])
{
  if ((argc == 2) && (strcmp(argv[1], "stop") == 0)) {
    MotorCalibration_Stop();
    DebugConsole_Printf("Rs stopped\r\n");
    return;
  }
  if ((argc != 1) || (foc_motor_state != FOC_MOTOR_IDLE) ||
      (foc.calibration.calibrated == 0U) ||
      (foc.state.vbus < 5.0f) || (foc.state.vbus > 50.0f) ||
      (TIM1->BDTR & TIM_BDTR_MOE)) {
    DebugConsole_Printf("Rs rejected: use rs | rs stop; check idle, ADC, Vbus\r\n");
    return;
  }

  /* 保持与PC13原入口完全一致：停止FOC、配置Rs桥臂，再启动CH4 ADC触发。 */
  FOC_PWM_Stop();
  MotorCalibration_Start();
  foc_motor_state = FOC_MOTOR_CALIBRATION;
  TIM1->CCR4 = foc.timer.adc_trigger;
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
    MotorCalibration_Stop();
    motor_cal.state = CAL_ERROR;
    DebugConsole_Printf("Rs start FAILED: TIM1 CH4\r\n");
    return;
  }
  DebugConsole_Printf("Rs started\r\n");
}

/* ls默认自动辨识AB→BC→CA；指定相别时仍可单独诊断，均用2×Rs补偿。 */
static void MotorDebug_Ls(int argc, char *argv[])
{
  if ((argc == 2) && (strcmp(argv[1], "stop") == 0)) {
    MotorCalibration_Stop();
    DebugConsole_Printf("Ls stopped\r\n");
    return;
  }
  CalPhase_t phase = CAL_PHASE_AB;
  uint8_t all = 1U;
  int pos = 1;
  if (argc > 1) {
    if (strcmp(argv[1], "ab") == 0) { phase = CAL_PHASE_AB; all = 0U; pos = 2; }
    else if (strcmp(argv[1], "bc") == 0) { phase = CAL_PHASE_BC; all = 0U; pos = 2; }
    else if (strcmp(argv[1], "ca") == 0) { phase = CAL_PHASE_CA; all = 0U; pos = 2; }
    else if (strcmp(argv[1], "all") == 0) { pos = 2; }
  }
  if (argc > pos + 1) {
    DebugConsole_Printf("Usage: ls [all|ab|bc|ca] [Rs_ohm] | ls stop\r\n");
    return;
  }
  if (argc == pos + 1) {
    char *end;
    float rs = strtof(argv[pos], &end);
    if ((*end != '\0') || !(rs >= 0.1f && rs <= 2.0f) ||
        (foc_motor_state != FOC_MOTOR_IDLE)) {
      DebugConsole_Printf("Usage: ls [all|ab|bc|ca] [Rs_ohm] | ls stop\r\n");
      return;
    }
    motor_cal.rs = rs;
    DebugConsole_Printf("Ls using supplied Rs=%.6fohm\r\n", rs);
  }
  if (all) {
    if (MotorCalibration_LsStartAll())
      DebugConsole_Printf("Ls sequence started: AB -> BC -> CA\r\n");
    else
      DebugConsole_Printf("Ls rejected: stop motor, measure Rs, check Vbus\r\n");
  } else if (MotorCalibration_LsStart(phase)) {
    DebugConsole_Printf("Ls %s started\r\n",
      phase == CAL_PHASE_AB ? "AB" : phase == CAL_PHASE_BC ? "BC" : "CA");
  } else {
    DebugConsole_Printf("Ls rejected: stop motor, measure Rs, check Vbus\r\n");
  }
}

/* 只显示本次上电后 RAM 中的标定结果，不读取或写入 Flash。 */
static void MotorDebug_Cal(int argc, char *argv[])
{
  if ((argc != 2) || (strcmp(argv[1], "show") != 0)) {
    DebugConsole_Printf("Usage: cal show\r\n");
    return;
  }
  DebugConsole_Printf("Rs=%.6fohm Ls AB/BC/CA=%.3f/%.3f/%.3fuH\r\n",
                      motor_cal.rs, motor_cal.ls_ab * 1e6f,
                      motor_cal.ls_bc * 1e6f, motor_cal.ls_ca * 1e6f);
}

/* 调试变量直接绑定已有控制器；维持当前串口接口和设定范围。 */
static void MotorDebug_Register(void)
{
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
  DebugConsole_RegisterCommand("rs", MotorDebug_Rs,
    "rs: identify AB/BC/CA resistance; rs stop: abort");
  DebugConsole_RegisterCommand("ls", MotorDebug_Ls,
    "ls: AB->BC->CA; ls [all|ab|bc|ca] [Rs_ohm]; ls stop: abort");
  DebugConsole_RegisterCommand("cal", MotorDebug_Cal,
    "cal show (RAM calibration only)");

}

FOC_Control_t *MotorApp_GetControl(void)
{
  return &motor_control;
}

static void FOC_ADC_AND_OPAMP_Calibration_Start(void) {
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
static void JustFloat_Init(void) {
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



/** @brief 初始化电机业务，保留原始电流零偏、ADC触发和输出安全时序。 */
HAL_StatusTypeDef MotorApp_Init(void)
{
  // 初始化DWT计数器，便于后续精确延时
  DWT_Delay_Init();
  // 初始化CORDIC，便于后续正余弦计算
  CORDIC_SinCos_RegisterConfig();
  // 初始化浮点数发送功能
  JustFloat_Init();
  // 初始化FOC控制器
  FOC_Control_Init(&motor_control, foc.timer.Ts);

  // 初始化串口调试控制台，绑定USART2和发送函数
  if (DebugConsole_Init(&huart2, DebugConsole_Tx) != HAL_OK) {
    return HAL_ERROR;
  }

  // 注册调试命令和变量
  MotorDebug_Register();

  // 初始化ADC和运放的自校准，确保电流采样准确
  FOC_ADC_AND_OPAMP_Calibration_Start();


  // 启动ADC注入转换中断，便于FOC实时控制
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

  // 设置TIM1 CH4为ADC触发输出，保持电机停机
  TIM1->CCR4 = foc.timer.adc_trigger;

  // 启动TIM1 CH4 PWM输出，触发ADC注入采样
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }

  // 设置FOC状态为校准阶段，等待零偏采样完成
  {
    uint32_t calibration_start_tick = HAL_GetTick();
    while (foc.calibration.calibrated == 0U) {
      if ((HAL_GetTick() - calibration_start_tick) >= 200U) {
        (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
        Error_Handler();
      }
    }
  }

  // 校准完成后停止TIM1 CH4 PWM输出，保持电机停机
  if (HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }

  return HAL_OK;
}

/* EXTI入口：关MOE后只投递按键位，决不在中断中调用HAL初始化。 */
void MotorApp_RequestButton(uint16_t pin)
{
    if ((pin == GPIO_PIN_10) || (pin == GPIO_PIN_11) || (pin == GPIO_PIN_13)) {
        __HAL_TIM_MOE_DISABLE(&htim1);
        MotorCalibration_LsFault(3U); /* GPIO脉冲不受MOE控制，必须同步停止TIM2/3。 */
        motor_cal.state = CAL_IDLE;    /* 禁止Rs ADC中断重新打开MOE。 */
        motor_button_pending |= pin;
    }
}

/** @brief 主循环业务：先处理按键，再运行原有ADC/CAN/串口任务。 */
void MotorApp_Process(void)
{
    if (motor_button_pending != 0U) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        uint16_t buttons = motor_button_pending; /* 一次取走事件，停机优先。 */
        motor_button_pending = 0U;
        __set_PRIMASK(primask);

        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);

        switch (buttons & (GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_10)) {
        case GPIO_PIN_11:
          MotorCalibration_Stop();
          break;

        case GPIO_PIN_13:
          MotorCalibration_Stop();
          MotorDebug_Rs(1, NULL);
          break;

        case GPIO_PIN_10:
          MotorCalibration_Stop();
          foc_motor_state = FOC_MOTOR_OPEN_LOOP;
          FOC_PWM_Start();
          break;

        default:
          break;
        }
    }
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
    MotorCalibration_Process(); /* Rs/Ls统一主循环入口：计算、推进及安全恢复。 */
    DebugConsole_TxProcess(); // USART2 DMA 后台发送，不等待串口。
}

/*
 * 该回调运行在PWM同步的ADC注入中断上下文，必须保持确定性：不使用HAL延时、
 * 不等待外设完成，不调用会阻塞的协议/日志接口；慢速工作下放到主循环或TIM6。
 */
/** @brief 电机应用ADC注入处理（每个PWM周期触发）：校准阶段累加零偏，运行阶段执行FOC电流环闭环控制。 */
void MotorApp_OnInjectedConversion(ADC_HandleTypeDef *hadc) {

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
  /* Ls使用ADC2规则组DMA，禁止ADC1残余注入中断覆盖TIM1的CCR1。 */
  if ((foc_motor_state == FOC_MOTOR_CALIBRATION) &&
      (motor_cal.state == CAL_IDLE)) return;

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
