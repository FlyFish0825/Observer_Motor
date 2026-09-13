/**
 * @file motor_app.c
 * @brief 电机应用调度：初始化、零偏校准、闭环控制和波形发送。
 *
 * 主循环负责规则组电压采样、启动请求和串口命令；ADC1注入完成中断
 * 负责电流换算、磁链观测、双闭环计算及PWM更新。控制周期按40us配置。
 * voltage_source由控制中断维护，规则组数据通过BoardAdc快照交接。
 */
#include "motor_app.h"

#include "adc.h"
#include "board_adc.h"
#include "bsp_dwt.h"
#include "controller.h"
#include "cordic.h"
#include "debug_console.h"
#include "foc_math.h"
#include "main.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"

#include "arm_math.h"
#include <stddef.h>
#include <stdint.h>

/*
 * 端电压RC：100kΩ / 5.1kΩ / 68nF，截止频率约482Hz。
 * 7极对电机在1200rpm时电频约140Hz，低速使用实测端电压。
 */
#define MOTOR_APP_CALCULATED_VOLTAGE_ENTER_RPM 1200.0f
#define MOTOR_APP_MEASURED_VOLTAGE_RETURN_RPM   900.0f
#define MOTOR_APP_VOLTAGE_BLEND_TIME_S            0.020f

/* 25kHz控制周期下250次为10ms，超时后不再使用陈旧端电压。 */
#define MOTOR_APP_PHASE_VOLTAGE_STALE_COUNT       250U

typedef struct {
  /* 实测电压保存在foc.state中；此处仅记录新数据序号与切换状态。 */
  uint32_t sequence;
  /* 未取得新数据的控制周期数；达到超时条件后选择重构电压。 */
  uint32_t stale_count;
  /* 选择目标与实际融合权重分开保存，使切换过程可以逐拍渐变。 */
  volatile uint32_t measured_selected;
  volatile float measured_weight;
} MotorApp_VoltageSource_t;

typedef struct {
  /* 小端float六通道，加4字节帧尾，总计28字节，直接交给DMA发送。 */
  float data[6];
  uint32_t tail;
} MotorApp_JustFloatFrame_t;

static FOC_Control_t motor_control;

static MotorApp_VoltageSource_t voltage_source = {
    .stale_count = MOTOR_APP_PHASE_VOLTAGE_STALE_COUNT + 1U,
    .measured_selected = 1U,
    .measured_weight = 1.0f,
};

static MotorApp_JustFloatFrame_t just_float_frame
    __attribute__((aligned(4)));
static volatile uint32_t just_float_enabled = 1U;
/* 串口set run 1请求启动，set run 0请求停止；上电默认不运行。 */
static volatile uint32_t motor_run_requested = 0U;
static volatile uint8_t motor_idle_reset_done = 0U;
static volatile uint8_t motor_console_tx_active = 0U;
static volatile uint32_t motor_adc_irq_count = 0U;

/**
 * @brief IDLE关闭功率输出，首次进入时清除闭环历史。
 * 直接关闭MOE立即撤销输出；停止六个功率通道使HAL通道状态回到READY。
 * 保留CH4及计数器，用于继续产生ADC触发。零偏与外部目标命令保留。
 */
static void MotorApp_ResetIdle(void) {
  CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
  if (motor_idle_reset_done != 0U) {
    return;
  }

  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  TIM1->CCR3 = 0U;

  FOC_Control_Reset(&motor_control);
  foc.state.u_dq = (FOC_DQ_t){0};
  foc.state.u_alpha_beta = (FOC_AlphaBeta_t){0};
  foc.state.u_abc = (FOC_ABC_t){0};
  foc.svpwm = (FOC_SVPWM_Output_t){0};
  foc.observer.state = (Observer_State_t){0};
  Observer_Init(&foc.observer, &foc.observer.motor, &foc.observer.config);
  voltage_source.measured_selected = 1U;
  voltage_source.measured_weight = 1.0f;
  motor_run_requested = 0U;
  motor_idle_reset_done = 1U;
}

/* 文本控制台的阻塞发送接口，由主循环命令处理调用。 */
static void MotorApp_DebugConsoleTx(const uint8_t *data, uint16_t length) {
  /* 先禁止中断发起下一帧，等待当前DMA帧完全发完，再发送文本。 */
  motor_console_tx_active = 1U;
  __DMB();
  uint32_t start = HAL_GetTick();
  while ((USART1->ISR & USART_ISR_TC) == 0U) {
    if ((HAL_GetTick() - start) >= 100U) {
      motor_console_tx_active = 0U;
      return;
    }
  }
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)data, length, 100U);
  motor_console_tx_active = 0U;
}

/* 一条纯文本状态回复，用于区分命令接收、校准等待和功率启动。 */
static void MotorApp_DebugStatus(int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  DebugConsole_Printf(
      "STATUS run=%lu state=%u cal=%u idle_reset=%u adc_irq=%lu ARR=%lu CCR4=%lu MOE=%u\r\n",
      (unsigned long)motor_run_requested, (unsigned int)foc_motor_state,
      (unsigned int)foc.calibration.calibrated,
      (unsigned int)motor_idle_reset_done, (unsigned long)motor_adc_irq_count,
      (unsigned long)TIM1->ARR, (unsigned long)TIM1->CCR4,
      (unsigned int)((TIM1->BDTR & TIM_BDTR_MOE) != 0U));
  /* 分行输出避免超过控制台192字节缓冲；读数用于诊断，不保证同一拍快照。 */
  DebugConsole_Printf("DRIVE Vbus=%.3f cmd=%.1f ref=%.1f rpm=%.1f speed_en=%lu\r\n",
      (double)foc.state.vbus, (double)motor_control.speed_command_rpm,
      (double)motor_control.speed_ref_active_rpm,
      (double)foc.observer.state.speed_rpm,
      (unsigned long)motor_control.speed_loop_enable);
  DebugConsole_Printf("CURRENT Iq_ref=%.3f Id=%.3f Iq=%.3f Ud=%.3f Uq=%.3f\r\n",
      (double)motor_control.iq_ref_active, (double)foc.state.i_dq.d,
      (double)foc.state.i_dq.q, (double)foc.state.u_dq.d,
      (double)foc.state.u_dq.q);
  DebugConsole_Printf("PWM CCR=%lu,%lu,%lu CCER=0x%08lX CR1=0x%08lX\r\n",
      (unsigned long)TIM1->CCR1, (unsigned long)TIM1->CCR2,
      (unsigned long)TIM1->CCR3, (unsigned long)TIM1->CCER,
      (unsigned long)TIM1->CR1);
  DebugConsole_Printf("OBSERVER phase=%.3f flux=%.6f weight=%.3f stale=%lu\r\n",
      (double)foc.observer.state.phase_raw, (double)foc.observer.state.psi_mag,
      (double)voltage_source.measured_weight,
      (unsigned long)voltage_source.stale_count);
}

/**
 * @brief 将命令名绑定到现有控制结构，避免另外维护一份参数副本。
 * 注册接口最后一个参数为只读标志；范围只约束控制台写入，
 * 不等同于控制器输出限幅。例如速度PI的Iq限幅仍由controller.h定义。
 */
static HAL_StatusTypeDef MotorApp_RegisterDebugVariables(void) {
  uint8_t success = 1U;

  success &= DebugConsole_RegisterF32("id", &motor_control.id_ref,
                                      -8.0f, 8.0f, false);
  success &= DebugConsole_RegisterF32("iq", &motor_control.iq_ref,
                                      -8.0f, 8.0f, false);
  success &= DebugConsole_RegisterF32("speed", &motor_control.speed_command_rpm,
                                      -15000.0f, 15000.0f, false);
  success &= DebugConsole_RegisterF32("speed_kp", &motor_control.speed_pi.kp,
                                      0.0f, 1.0f, false);
  success &= DebugConsole_RegisterF32("speed_ki", &motor_control.speed_pi.ki,
                                      0.0f, 100.0f, false);
  success &= DebugConsole_RegisterF32(
      "speed_slew", &motor_control.speed_slew_rpm_per_s,
      0.0f, 30000.0f, false);
  success &= DebugConsole_RegisterBool("speed_en",
                                       &motor_control.speed_loop_enable,
                                       false);
  success &= DebugConsole_RegisterBool("just_float", &just_float_enabled,
                                       false);
  success &= DebugConsole_RegisterBool("run", &motor_run_requested, false);
  success &= DebugConsole_RegisterCommand("status", MotorApp_DebugStatus,
                                           "status: show motor startup state");

  success &= DebugConsole_RegisterF32("vbus", &foc.state.vbus,
                                      0.0f, 70.0f, false);
  success &= DebugConsole_RegisterF32("phase_u", &foc.state.u_abc_measured.a,
                                      0.0f, 70.0f, true);
  success &= DebugConsole_RegisterF32("phase_v", &foc.state.u_abc_measured.b,
                                      0.0f, 70.0f, true);
  success &= DebugConsole_RegisterF32("phase_w", &foc.state.u_abc_measured.c,
                                      0.0f, 70.0f, true);
  success &= DebugConsole_RegisterU32(
      "volt_src", &voltage_source.measured_selected, 0U, 1U, true);
  success &= DebugConsole_RegisterF32(
      "volt_weight", &voltage_source.measured_weight, 0.0f, 1.0f, true);

  return (success != 0U) ? HAL_OK : HAL_ERROR;
}

/**
 * @brief 在功率桥未启动时校准运放和ADC的硬件偏差。
 * 运放启动后预留模拟电路稳定时间；这一步不代替后续三相电流零偏平均。
 * @return 任一硬件操作失败返回HAL_ERROR，由初始化调用方处理。
 */
static HAL_StatusTypeDef MotorApp_CalibrateAnalogFrontEnd(void) {
  if (HAL_OPAMP_SelfCalibrate(&hopamp1) != HAL_OK) {
    return HAL_ERROR;
  }
  if (HAL_OPAMP_SelfCalibrate(&hopamp2) != HAL_OK) {
    return HAL_ERROR;
  }
  if (HAL_OPAMP_SelfCalibrate(&hopamp3) != HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_OPAMP_Start(&hopamp1) != HAL_OK) {
    return HAL_ERROR;
  }
  if (HAL_OPAMP_Start(&hopamp2) != HAL_OK) {
    return HAL_ERROR;
  }
  if (HAL_OPAMP_Start(&hopamp3) != HAL_OK) {
    return HAL_ERROR;
  }

  DWT_Delay_Ms(10U);

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
    return HAL_ERROR;
  }
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
    return HAL_ERROR;
  }

  DWT_Delay_Ms(10U);

  return HAL_OK;
}

/* 预先设置DMA外设地址和固定帧地址，控制中断只需更新内容及传输长度。 */
static void MotorApp_JustFloatInit(void) {
  /* STM32小端存储后依次为00 00 80 7F，即JustFloat帧尾。 */
  just_float_frame.tail = 0x7F800000UL;
  just_float_enabled = 1U;

  CLEAR_BIT(USART1->CR3, USART_CR3_DMAT);
  __HAL_DMA_DISABLE(&hdma_usart1_tx);
  while ((hdma_usart1_tx.Instance->CCR & DMA_CCR_EN) != 0U) {
  }

  __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx,
                       __HAL_DMA_GET_GI_FLAG_INDEX(&hdma_usart1_tx));

  hdma_usart1_tx.Instance->CPAR = (uint32_t)&USART1->TDR;
  hdma_usart1_tx.Instance->CMAR = (uint32_t)&just_float_frame;
  hdma_usart1_tx.Instance->CNDTR = 0U;

  SET_BIT(USART1->CR3, USART_CR3_DMAT);
}

/**
 * @brief 尝试启动一帧波形发送，返回0表示已启动，-1表示串口仍忙。
 * 检查TC后才修改共用帧，防止DMA尚未读完时覆盖内容。
 * 忙时丢弃本拍数据，不等待整帧串口发送，因此波形帧率低于控制频率。
 */
static int MotorApp_SendJustFloat(float f0, float f1, float f2,
                                  float f3, float f4, float f5) {
  if ((USART1->ISR & USART_ISR_TC) == 0U) {
    return -1;
  }

  just_float_frame.data[0] = f0;
  just_float_frame.data[1] = f1;
  just_float_frame.data[2] = f2;
  just_float_frame.data[3] = f3;
  just_float_frame.data[4] = f4;
  just_float_frame.data[5] = f5;

  __HAL_DMA_DISABLE(&hdma_usart1_tx);
  while ((hdma_usart1_tx.Instance->CCR & DMA_CCR_EN) != 0U) {
  }

  __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx,
                       __HAL_DMA_GET_GI_FLAG_INDEX(&hdma_usart1_tx));
  hdma_usart1_tx.Instance->CNDTR = sizeof(MotorApp_JustFloatFrame_t);
  USART1->ICR = USART_ICR_TCCF;

  __DMB();
  __HAL_DMA_ENABLE(&hdma_usart1_tx);

  return 0;
}

/**
 * @brief 主循环响应串口启动请求，零偏校准完成后开启观测器闭环。
 * 启动时将斜坡内部参考同步到目标速度；后续变速才经过斜坡。
 * 当前流程没有定位或开环拖动阶段，静止转子角度依赖观测器初始状态。
 */
static void MotorApp_StartClosedLoop(void) {
  /* 开启六路PWM前暂停CH4，避免中途插入一次控制中断。 */
  if (HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }

  motor_control.speed_ref_rpm = motor_control.speed_command_rpm;
  motor_control.speed_ref_active_rpm = motor_control.speed_ref_rpm;
  motor_control.speed_loop_enable = 1U;
  motor_control.speed_loop_enable_last = 0U;

  /* 每次启动均从实测端电压开始观测。 */
  voltage_source.measured_selected = 1U;
  voltage_source.measured_weight = 1.0f;

  foc_motor_state = FOC_MOTOR_CLOSED_LOOP;
  motor_idle_reset_done = 0U;
  FOC_PWM_Start();
}

/**
 * @brief 每个控制周期更新观测器电压输入和实测权重。
 * 快照读取失败时沿用缓存；连续无更新超过10ms后目标切向重构电压。
 * 1200/900rpm构成迟滞区间，区间内保持上次选择；正反转均按绝对值判断。
 * 包括超时切换在内，实际权重都按20ms渐变，不会立即跳到目标值。
 */
static void MotorApp_UpdateObserverVoltage(Observer_Input_t *input) {
  BoardAdcMeasurements_t snapshot;
  float blend_step;
  float speed_abs_rpm;
  uint32_t sequence;
  uint8_t snapshot_valid;

  snapshot_valid = BoardAdc_GetSnapshot(&snapshot, &sequence);

  if ((snapshot_valid != 0U) && (sequence != voltage_source.sequence)) {
    /* 仅接受完整快照，主循环不直接逐相改写控制中断正在使用的状态。 */
    foc.state.u_abc_measured.a = snapshot.phase_u_voltage;
    foc.state.u_abc_measured.b = snapshot.phase_v_voltage;
    foc.state.u_abc_measured.c = snapshot.phase_w_voltage;
    FOC_Clarke(&foc.state.u_abc_measured, &foc.state.u_alpha_beta_measured);
    voltage_source.sequence = sequence;
    voltage_source.stale_count = 0U;
  } else if (voltage_source.stale_count < UINT32_MAX) {
    voltage_source.stale_count++;
  }

  snapshot_valid =
      (voltage_source.stale_count <= MOTOR_APP_PHASE_VOLTAGE_STALE_COUNT)
          ? 1U
          : 0U;

  /* 三端电压均为对地电压；完整Clarke变换会消去三相共有的零序分量。 */
  input->measured_u_alpha = foc.state.u_alpha_beta_measured.alpha;
  input->measured_u_beta = foc.state.u_alpha_beta_measured.beta;

  /* 标量fabsf可直接生成FPU的VABS指令，无需调用面向数组的arm_abs_f32。 */
  speed_abs_rpm = fabsf(foc.observer.state.speed_rpm);

  if (snapshot_valid == 0U) {
    voltage_source.measured_selected = 0U;
  } else if (speed_abs_rpm >= MOTOR_APP_CALCULATED_VOLTAGE_ENTER_RPM) {
    voltage_source.measured_selected = 0U;
  } else if (speed_abs_rpm <= MOTOR_APP_MEASURED_VOLTAGE_RETURN_RPM) {
    voltage_source.measured_selected = 1U;
  }

  blend_step = foc.timer.Ts / MOTOR_APP_VOLTAGE_BLEND_TIME_S;
  if (blend_step > 1.0f) {
    blend_step = 1.0f;
  }

  if (voltage_source.measured_selected != 0U) {
    voltage_source.measured_weight += blend_step;
    if (voltage_source.measured_weight > 1.0f) {
      voltage_source.measured_weight = 1.0f;
    }
  } else {
    voltage_source.measured_weight -= blend_step;
    if (voltage_source.measured_weight < 0.0f) {
      voltage_source.measured_weight = 0.0f;
    }
  }

  input->measured_voltage_weight = voltage_source.measured_weight;
}

/**
 * @brief 完成电流坐标变换、速度/电流PI以及电压到占空比的转换。
 * Park和逆Park共用本拍磁链原始电角度phase_raw；速度反馈取PLL估计值。
 * 这里生成下一次PWM比较值，实际寄存器写入在注入中断尾部统一执行。
 */
static void MotorApp_RunClosedLoop(void) {
  float phase_control;
  uint32_t phase_q31;

  /* 外部阶跃先由控制器内部转换为20000rpm/s速度斜坡。 */
  motor_control.speed_ref_rpm = motor_control.speed_command_rpm;

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
  (void)FOC_SVPWM_Run(&foc.state.u_abc, foc.state.vbus, &foc.timer,
                      &foc.svpwm);
}


/**
 * @brief  强制将三相功率级的 6 路 PWM 控制引脚置为安全低电平
 *
 * @note
 *  该函数应在系统启动早期、TIM1 和相关 GPIO 被配置为复用 PWM 功能之前调用。
 *  目的是防止 MCU 上电/复位过程中 PWM 引脚处于不确定状态，
 *  导致栅极驱动器误导通 MOSFET。
 *
 *  当前三相 PWM 引脚：
 *    PA8  -> TIM1_CH1
 *    PA9  -> TIM1_CH2
 *    PA10 -> TIM1_CH3
 *    PB13 -> TIM1_CH1N
 *    PB14 -> TIM1_CH2N
 *    PB15 -> TIM1_CH3N
 *
 *  执行完成后，6 路引脚均被配置为普通推挽输出，并保持低电平。
 *  后续由 CubeMX 生成的 GPIO/TIM 初始化代码重新配置为 TIM1 复用功能。
 */
void MotorApp_ForcePowerStageSafe(void) {
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
                    GPIO_PIN_RESET);

  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOB, &gpio);
}



/**
 * @brief  初始化电机控制应用层
 *
 * @return HAL_OK    初始化成功
 * @return HAL_ERROR 任一步骤初始化失败
 *
 * @note
 * 初始化流程大致为：
 *
 *  前置条件：main已在MX_TIM1_Init之前调用FOC_Data_Init，完成板级参数初始化。
 *
 *  1. 使用已初始化的 FOC 数据结构
 *  2. 初始化 DWT 微秒延时
 *  3. 配置 CORDIC 正余弦计算
 *  4. 初始化调试波形输出
 *  5. 初始化电机控制器
 *  6. 保持功率桥关闭
 *  7. 初始化调试串口及在线变量
 *  8. 校准模拟前端
 *  9. 获取母线电压、端电压初值
 * 10. 启动 ADC 注入组
 * 11. 启动 TIM1 CH4，产生 ADC 同步采样触发
 *
 * 此函数结束时：
 * - ADC 注入采样已经开始工作；
 * - TIM1 CH4 已经运行，用于产生电流采样触发；
 * - HAL启动CH4会置位MOE，但CH1~3及其互补通道尚未使能；
 * - 三相功率桥不会在本函数中直接开始输出 PWM。
 */
HAL_StatusTypeDef MotorApp_Init(void) {
  if (DWT_Delay_Init() == 0U) {
    return HAL_ERROR;
  }

  CORDIC_SinCos_RegisterConfig();
  MotorApp_JustFloatInit();
  FOC_Control_Init(&motor_control, foc.timer.Ts);

  /* 保持功率桥关闭，只启动CH4完成静止电流零偏校准。 */
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  TIM1->CCR3 = 0U;
  TIM1->CCR4 = 0U;
  __HAL_TIM_MOE_DISABLE(&htim1);

  if (DebugConsole_Init(&huart1, MotorApp_DebugConsoleTx) != HAL_OK) {
    return HAL_ERROR;
  }
  if (MotorApp_RegisterDebugVariables() != HAL_OK) {
    return HAL_ERROR;
  }
  if (MotorApp_CalibrateAnalogFrontEnd() != HAL_OK) {
    return HAL_ERROR;
  }

  /* 在电流采样中断开始前先取得母线和端电压初值。 */
  if (BoardAdc_Update() == HAL_OK) {
    foc.state.vbus = BoardAdc_GetMeasurements()->vbus_voltage;
  }

  /*
   * ADC2只有一个注入通道，先启动但不打开中断；其结果会在ADC1完整序列
   * 结束时统一读取。这样共享的ADC1_2 IRQ只由ADC1产生控制中断。
   */
  if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) {
    return HAL_ERROR;
  }

  /*
   * hadc1.Init.EOCSelection必须保留ADC_EOC_SINGLE_CONV，供规则组逐Rank
   * 轮询读取；HAL据此默认打开JEOC。这里仅把注入组中断切换到JEOS，
   * 确保ADC1的U/W两个Rank全部完成后，每个PWM周期只进入一次控制回调。
   */
  __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOC);
  __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOS);

  TIM1->CCR4 = foc.timer.adc_trigger;
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}



/**
 * @brief 电机应用层主循环处理函数
 *
 * @note
 * 该函数在 main() 的 while(1) 中循环调用，只处理不要求严格实时性的任务。
 * 规则组 ADC 轮询和串口命令解析可能产生等待或不确定执行时间，
 * 因此不放入 25 kHz 的电流环控制中断，避免影响 FOC 控制周期。
 */
void MotorApp_Process(void) {
  static uint8_t ready_reported = 0U;
  if (ready_reported == 0U) {
    ready_reported = 1U;
    DebugConsole_Printf("READY: set run 1 / set run 0 / status\r\n");
  }
  /* 优先解析命令，避免规则组的轮询等待增加启停请求延迟。 */
  DebugConsole_Process();
  /*
   * 轮询 ADC 规则组，采集母线电压及三相端电压。
   * 采样成功后更新 FOC 使用的母线电压。
   *
   * 规则组采用软件触发，与 25 kHz PWM/电流采样不同步，
   * 主要用于母线电压监测以及低速时的端电压观测。
   */
  if (BoardAdc_Update() == HAL_OK) {
    foc.state.vbus = BoardAdc_GetMeasurements()->vbus_voltage;
  }

  /* 校准完成且IDLE复位已完成才能启动，重复run 1不会重新初始化运行电机。
   * 校准期间收到run 1则等待校准结束；run 0可以取消该请求。
   */
  if ((motor_run_requested != 0U) &&
      (foc.calibration.calibrated != 0U) &&
      (motor_idle_reset_done != 0U) &&
      (foc_motor_state == FOC_MOTOR_IDLE)) {
    MotorApp_StartClosedLoop();
  }
}
/**
 * @brief 注入转换完成后的实时入口，只有ADC1回调执行完整控制流程。
 * ADC1 rank1/rank2对应U/W，ADC2 rank1对应V；ADC1的两路顺序转换，
 * 三相并非严格同时采样。ADC2不打开注入中断，控制入口仅由ADC1的
 * 注入序列结束JEOS产生，因此读到JDR2时本拍的两个Rank都已完成。
 */
void MotorApp_OnInjectedConversion(ADC_HandleTypeDef *hadc) {
  static uint16_t calibration_count = 0U;
  Observer_Input_t observer_input = {0};
  uint16_t adc_a;
  uint16_t adc_b;
  uint16_t adc_c;

  if ((hadc == NULL) || (hadc->Instance != ADC1)) {
    return;
  }
  motor_adc_irq_count++;

  /* 串口解析完run 0后，下一次采样中断先退出闭环并关断功率输出。 */
  if (motor_run_requested == 0U) {
    foc_motor_state = FOC_MOTOR_IDLE;
  }

  /* 状态不是闭环时先关桥，包含零偏校准期间及无效状态值。 */
  if (foc_motor_state != FOC_MOTOR_CLOSED_LOOP) {
    foc_motor_state = FOC_MOTOR_IDLE;
    MotorApp_ResetIdle();
  } else {
    motor_idle_reset_done = 0U;
  }

  /* 前1000拍仅累加原始计数，约40ms；桥未驱动且无相电流是零偏校准前提。 */
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
      /* 仅标记校准就绪；保持IDLE，启动由串口run命令决定。 */
    }
    return;
  }

  adc_a = (uint16_t)ADC1->JDR1;
  adc_b = (uint16_t)ADC2->JDR1;
  adc_c = (uint16_t)ADC1->JDR2;

  FOC_Get_Iabc(&foc, adc_a, adc_b, adc_c);
  FOC_Clarke(&foc.state.i_abc, &foc.state.i_alpha_beta);

  /* 先用上次计算的占空比与本拍电流观测，再计算新的电压命令。 */
  observer_input.duty_a = foc.svpwm.duty_a;
  observer_input.duty_b = foc.svpwm.duty_b;
  observer_input.duty_c = foc.svpwm.duty_c;
  observer_input.vbus = foc.state.vbus;
  observer_input.i_alpha = foc.state.i_alpha_beta.alpha;
  observer_input.i_beta = foc.state.i_alpha_beta.beta;

  //选择观测器电压来源
  MotorApp_UpdateObserverVoltage(&observer_input);
  if (foc_motor_state == FOC_MOTOR_CLOSED_LOOP) {
    Observer_Run(&foc.observer, &observer_input);
    MotorApp_RunClosedLoop();
    /* 只有闭环状态允许写入新的功率PWM比较值。 */
    TIM1->CCR1 = foc.svpwm.ccr_a;
    TIM1->CCR2 = foc.svpwm.ccr_b;
    TIM1->CCR3 = foc.svpwm.ccr_c;
  }

  /* VOFA通道：Iu(A)、Iv(A)、Iw(A)、机械转速(rpm)、电角度(deg)、母线(V)。 */
  if ((just_float_enabled != 0U) &&
      (motor_console_tx_active == 0U) &&
      (foc_motor_state == FOC_MOTOR_CLOSED_LOOP) &&
      ((USART1->ISR & USART_ISR_TC) != 0U)) {
    (void)MotorApp_SendJustFloat(
        foc.state.i_abc.a, foc.state.i_abc.b, foc.state.i_abc.c,
        motor_control.speed_ref_active_rpm,
        foc.observer.state.phase_raw * RAD_TO_DEG_F, foc.state.vbus);
  }
}
