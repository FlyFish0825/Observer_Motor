/**
 * @file motor_app.c
 * @brief 电机应用层：启动时序状态机、控制参考仲裁、端电压融合和调试接口。
 *
 * 分层约定：
 * - 实时层：ADC1注入中断MotorApp_OnInjectedConversion()按40us运行，负责电流
 *   换算、磁链观测、状态机推进、电流环计算和PWM比较值写入。
 * - 非实时层：主循环MotorApp_Process()负责规则组母线/温度采样、串口命令解析和
 *   run启停请求；两层之间只通过volatile标志和控制器命令字段交互。
 * - 状态机只产生控制参考FOC_Control_Reference_t，公共的Park/逆Park/电流PI/
 *   SVPWM统一由MotorApp_RunCurrentLoop()执行，不随状态复制实现。
 *
 * 启动路径：IDLE -> ALIGN(固定电角度定位) -> OPEN_LOOP_IF(虚拟电角度拖动)
 * -> CLOSED_LOOP。I/F期间只做观测器可信判断，控制角接管属于后续步骤。
 * 电流与三相端电压都由TIM1同步Injected采样，规则组只保留母线电压和温度。
 */
#include "motor_app.h"

#include "adc.h"
#include "board_adc.h"
#include "bsp_dwt.h"
#include "controller.h"
#include "debug_console.h"
#include "foc_math.h"
#include "main.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"

#include "arm_math.h"
#include <stddef.h>
#include <stdint.h>

/* ======================== 应用层参数 ======================== */

/* ADC注入中断频率，与PWM载波频率一致；所有时间量都折算成该节拍的拍数。 */
#define MOTOR_APP_CONTROL_HZ 25000U

/* 毫秒折算为控制拍数：用于定位时长和可信判据的持续时间。 */
#define MOTOR_APP_MS_TO_TICKS(ms) \
  (((uint32_t)(ms) * MOTOR_APP_CONTROL_HZ) / 1000U)

/*
 * 端电压RC：100kΩ/5.1kΩ/68nF，截止频率约482Hz。
 * 7极对电机在1200rpm时电频约140Hz，因此低速用实测端电压、高速切到占空比重构
 * 电压；两个阈值之间构成迟滞区间，避免切换点附近反复抖动。
 */
#define MOTOR_APP_CALCULATED_VOLTAGE_ENTER_RPM 1200.0f
#define MOTOR_APP_MEASURED_VOLTAGE_RETURN_RPM   900.0f
#define MOTOR_APP_VOLTAGE_BLEND_TIME_S            0.020f

/* ALIGN：固定电角度只给定Id建立磁场；时长和电流都是待标定量。 */
#define MOTOR_APP_ALIGN_TIME_MS 30U
#define MOTOR_APP_ALIGN_ID_A 1.0f

/* I/F：虚拟电角度按受限电角加速度爬升，只给定Iq。 */
#define MOTOR_APP_IF_IQ_A 1.5f
#define MOTOR_APP_IF_ACCEL_RAD_S2 200.0f /* 电角加速度，rad/s^2；必须与J_eq和可用电流匹配 */

/*
 * 拖动电角速度上限＝首轮检查速度1000rpm机械转速：
 * 1000 * 2pi/60 * 7极对 = 733 rad/s。低于该速度时BEMF太小（300rpm约38个ADC计数、
 * 1000rpm约125个），观测器角度容易被死区与噪声支配，因此先拖到这里再判断接管。
 */
#define MOTOR_APP_IF_MAX_SPEED_RAD_S 733.0f
/* 到达检查速度后仍不可信的验证窗口；超时则判为启动失败并退流。 */
#define MOTOR_APP_IF_VERIFY_TIME_MS 300U

/*
 * ObserverReady判据（接管前置条件）。每条判据写入motor_ready_fail_mask的独立位，
 * 便于现场定位是哪一条不成立。转速下限300rpm对应BEMF约0.62V（约38个ADC计数），
 * 是本硬件上实测端电压明显高于噪声与死区模型误差的量级；该值是第一标定旋钮。
 */
#define MOTOR_APP_OBSERVER_READY_TIME_MS 5U
#define MOTOR_APP_OBSERVER_READY_MIN_RPM 300.0f
#define MOTOR_APP_OBSERVER_READY_SPAN_RAD 1.04719755f      /* 至少覆盖60°电角度 */
#define MOTOR_APP_OBSERVER_READY_MAX_ANGLE_STEP 0.35f      /* 单拍角度跳变上限，rad */
#define MOTOR_APP_OBSERVER_READY_MAX_SPEED_STEP 100.0f     /* 单拍转速跳变上限，rpm */
#define MOTOR_APP_OBSERVER_READY_MAX_LOAD_ANGLE 1.57079633f/* I/F与观测角允许角差，90° */
#define MOTOR_APP_OBSERVER_READY_MAX_ID_A 0.8f             /* 电流跟踪：|Id|上限，A */
#define MOTOR_APP_OBSERVER_READY_SUSTAIN_MS 2U             /* "持续失跟/饱和"去抖时间 */

/* 判据失败位。 */
#define MOTOR_APP_READY_FAIL_PSI (1UL << 0)
#define MOTOR_APP_READY_FAIL_INIT (1UL << 1)
#define MOTOR_APP_READY_FAIL_SPEED (1UL << 2)
#define MOTOR_APP_READY_FAIL_DIR (1UL << 3)
#define MOTOR_APP_READY_FAIL_ANGLE_STEP (1UL << 4)
#define MOTOR_APP_READY_FAIL_SPEED_STEP (1UL << 5)
#define MOTOR_APP_READY_FAIL_SYNC (1UL << 6)
#define MOTOR_APP_READY_FAIL_TRACK (1UL << 7)

/* 接管：角偏置渐消的最大角速度与时间上下限（首轮值，按角差与电流裕量标定）。 */
#define MOTOR_APP_HANDOVER_RATE_RAD_S 20.0f
#define MOTOR_APP_HANDOVER_MIN_TIME_MS 20U
#define MOTOR_APP_HANDOVER_MAX_TIME_MS 150U

/* 闭环健康：|Id|持续偏大或磁链出界视为失步，退流并报告。 */
#define MOTOR_APP_CLOSED_LOOP_MAX_ID_A 1.5f
#define MOTOR_APP_CLOSED_LOOP_FAULT_TIME_MS 20U

/* 启动失败原因，供status的fail字段显示。 */
#define MOTOR_APP_FAIL_NONE 0UL
#define MOTOR_APP_FAIL_OBSERVER_NOT_READY 1UL
#define MOTOR_APP_FAIL_LOST_SYNC 2UL

/*
 * ADC注入序列期望值，与adc.c的MX_ADC1_Init/MX_ADC2_Init保持一致，只用于上电回读自检。
 * JL是"注入转换次数-1"：ADC1为4次(Ia/Ic/U端/W端)，ADC2为2次(Ib/V端)。
 * 改动adc.c的注入通道后必须同步这里；不一致时自检会拒绝启动——这是"注入序列被
 * 静默截断"（V相端电压曾因此从未被采样、JDR2恒为0）的防护。
 */
#define MOTOR_APP_ADC1_INJ_JL 3U
#define MOTOR_APP_ADC1_INJ_JSQ1 3U  /* Ia  ADC1_IN3  */
#define MOTOR_APP_ADC1_INJ_JSQ2 12U /* Ic  ADC1_IN12 */
#define MOTOR_APP_ADC1_INJ_JSQ3 11U /* U端 ADC1_IN11 */
#define MOTOR_APP_ADC1_INJ_JSQ4 14U /* W端 ADC1_IN14 */
#define MOTOR_APP_ADC2_INJ_JL 1U
#define MOTOR_APP_ADC2_INJ_JSQ1 3U  /* Ib  ADC2_IN3  */
#define MOTOR_APP_ADC2_INJ_JSQ2 17U /* V端 ADC2_IN17 */

/* ======================== 应用层数据结构 ======================== */

/* 实测/重构端电压融合状态：选择目标与渐变权重分开保存，切换过程逐拍渐变。 */
typedef struct {
  volatile uint32_t measured_selected; /* 1=选定实测端电压，0=选定占空比重构 */
  volatile float measured_weight;      /* 实测端电压在融合结果中的权重0~1 */
} MotorApp_VoltageSource_t;

/* 启动时序中间状态：只保存角度、判据中间量和计数，不参与电流环计算。 */
typedef struct {
  float align_angle;           /* 定位固定电角度，rad */
  uint32_t align_count;        /* 已运行的定位拍数 */
  float if_angle;              /* I/F虚拟电角度，rad */
  float if_speed_rad_s;        /* I/F当前电角速度，rad/s */
  float if_speed_target_rad_s; /* I/F目标电角速度，rad/s */
  float if_direction;          /* I/F拖动方向，+1或-1 */

  /* ObserverReady判据的中间量。 */
  float ready_last_phase_rad;   /* 上一拍观测磁链角，用于单拍跳变判据 */
  float ready_last_speed_rpm;   /* 上一拍观测转速 */
  float ready_span_rad;         /* 判据连续成立期间累计覆盖的电角度 */
  uint32_t ready_count;         /* 判据连续成立的拍数 */
  uint32_t ready_sustain_count; /* "持续失跟/饱和"连续失败的拍数 */
  uint32_t ready_fail_mask;     /* 最近一次判据失败位 */
  uint8_t observer_ready;       /* 1=观测器可信，准许进入接管 */

  /* 检查速度处的验证计数与闭环健康计数。 */
  uint32_t verify_count;
  uint32_t fault_count;

  /* 接管：初始角偏置、已运行拍数与总拍数。 */
  float handover_offset_rad;
  uint32_t handover_ticks;
  uint32_t handover_ticks_total;
} MotorApp_Startup_t;

/* 小端float六通道加4字节帧尾，共28字节，直接交给DMA发送。 */
typedef struct {
  float data[6];
  uint32_t tail;
} MotorApp_JustFloatFrame_t;

/* ======================== 应用层状态变量 ======================== */

/* 应用层唯一的FOC控制器实例；调试控制台和CAN协议都通过它修改外部命令。 */
static FOC_Control_t motor_control;

/* 端电压融合状态；由控制中断逐拍更新，启动流程只做复位。 */
static MotorApp_VoltageSource_t voltage_source = {
    .measured_selected = 1U,
    .measured_weight = 1.0f,
};

/* 启动时序状态；由主循环启动流程初始化，由控制中断推进。 */
static MotorApp_Startup_t startup = {0};

/* DMA直接读取的波形帧，必须保持4字节对齐以满足外设访问要求。 */
static MotorApp_JustFloatFrame_t just_float_frame __attribute__((aligned(4)));
/* JustFloat波形开关：非零时允许控制中断尝试提交诊断帧。 */
static volatile uint32_t just_float_enabled = 1U;
/* 串口set run 1请求启动，set run 0请求停止；上电默认不运行。 */
static volatile uint32_t motor_run_requested = 0U;
/* 标记IDLE复位是否已经执行，避免每个控制周期重复停止HAL通道。 */
static volatile uint8_t motor_idle_reset_done = 0U;
/* 标记文本串口发送占用期，防止DMA波形与命令回复同时改写USART。 */
static volatile uint8_t motor_console_tx_active = 0U;
/* ADC注入中断累计次数，仅用于状态诊断和校准进度观察。 */
static volatile uint32_t motor_adc_irq_count = 0U;
/* ADC注入序列自检结果：0表示与adc.c不一致，run请求会被拒绝；由status的adc_cfg显示。 */
static volatile uint32_t motor_adc_cfg_ok = 0U;
/* 本拍实际用于Park/逆Park的控制角（rad），只供上位机与Live Watch观察。 */
static volatile float motor_control_angle_rad = 0.0f;
/* ObserverReady最近一次失败的判据位，0表示无失败项。 */
static volatile uint32_t motor_ready_fail_mask = 0U;
/* 启动失败原因（MOTOR_APP_FAIL_*），由status的fail字段显示，下一次启动时清零。 */
static volatile uint32_t motor_start_fail_reason = MOTOR_APP_FAIL_NONE;

/* ======================== 控制参考仲裁与电流环 ======================== */

/**
 * @brief 判断当前状态是否允许进入FOC实时控制。
 *
 * 这里是"主动控制状态"的唯一判定点：后续新增状态只需扩展本函数，
 * 不再修改ADC中断的主结构。
 */
static uint8_t MotorApp_IsControlState(FOC_Motor_State_t state) {
  return ((state == FOC_MOTOR_CLOSED_LOOP) ||
          (state == FOC_MOTOR_ALIGN) ||
          (state == FOC_MOTOR_OPEN_LOOP_IF) ||
          (state == FOC_MOTOR_OBSERVER_HANDOVER)) ? 1U : 0U;
}

/**
 * @brief 按当前状态仲裁本拍的控制角和dq电流参考。
 *
 * 唯一规则：状态只写参考，不写控制器内部状态。
 * - ALIGN：固定电角度，只给Id建立磁场。
 * - OPEN_LOOP_IF：虚拟电角度，只给Iq拖动。
 * - CLOSED_LOOP：观测器磁链角，参考直接取外部命令。
 */
static void MotorApp_ResolveControlReference(void) {
  switch (foc_motor_state) {
  case FOC_MOTOR_ALIGN:
    FOC_Control_SubmitReference(&motor_control, startup.align_angle,
                                MOTOR_APP_ALIGN_ID_A, 0.0f, 0U);
    break;

  case FOC_MOTOR_OPEN_LOOP_IF:
    FOC_Control_SubmitReference(&motor_control, startup.if_angle, 0.0f,
                                MOTOR_APP_IF_IQ_A, 0U);
    break;

  case FOC_MOTOR_OBSERVER_HANDOVER: {
    /* 接管：控制角=观测磁链角+按拍数线性渐消的初始偏置，电流参考保持I/f值。 */
    float blend = 0.0f;
    if (startup.handover_ticks_total > 0U) {
      blend = 1.0f - (float)startup.handover_ticks /
                         (float)startup.handover_ticks_total;
    }
    FOC_Control_SubmitReference(
        &motor_control,
        foc.observer.state.phase_raw + startup.handover_offset_rad * blend,
        0.0f, MOTOR_APP_IF_IQ_A, 0U);
    break;
  }

  case FOC_MOTOR_CLOSED_LOOP:
    /* 外部阶跃先复制到斜坡输入，实际限速由控制器内部的速度斜坡完成。 */
    motor_control.speed_ref_rpm = motor_control.speed_command_rpm;
    FOC_Control_SubmitReference(
        &motor_control, foc.observer.state.phase_raw, motor_control.id_ref,
        motor_control.iq_ref,
        (uint8_t)((motor_control.speed_loop_enable != 0U) ? 1U : 0U));
    break;

  default:
    /* IDLE在进入中断主体时已被拦截，不会到达这里。 */
    break;
  }
}

/**
 * @brief 公共电流环执行体：Park -> 电流PI -> 逆Park -> 逆Clarke -> SVPWM。
 *
 * 角度和电流参考全部取自控制器内的仲裁结果，所有运行状态共用这一条路径，
 * 状态之间的差别只体现在MotorApp_ResolveControlReference()给出的参考上。
 */
static void MotorApp_RunCurrentLoop(FOC_Control_t *control) {
  /* 控制角先做单步归一化，再交给CORDIC求正余弦。 */
  float theta_ctrl;
  uint32_t theta_q31;

  theta_ctrl = FOC_WrapToPiFast(control->reference.theta_ctrl);
  theta_q31 = (uint32_t)CORDIC_RadToQ31_WrappedFast(theta_ctrl);
  /* 诊断用：本拍实际控制角，供上位机比对观测角与虚拟角。 */
  motor_control_angle_rad = theta_ctrl;

  CORDIC_SinCos_FastF32((int32_t)theta_q31, &foc_sin_cos.sin,
                        &foc_sin_cos.cos);

  FOC_Park(&foc.state.i_alpha_beta, &foc_sin_cos, &foc.state.i_dq);

  FOC_Control_Run(control, foc.state.i_dq.d, foc.state.i_dq.q,
                  foc.observer.state.speed_rpm, foc.state.vbus,
                  &foc.state.u_dq.d, &foc.state.u_dq.q);

  FOC_InvPark(&foc.state.u_dq, &foc_sin_cos, &foc.state.u_alpha_beta);
  FOC_InvClarke(&foc.state.u_alpha_beta, &foc.state.u_abc);
  (void)FOC_SVPWM_Run(&foc.state.u_abc, foc.state.vbus, &foc.timer,
                      &foc.svpwm);
}

/* ======================== 启动时序状态机 ======================== */

/**
 * @brief I/F虚拟角积分：电角速度受限爬升，再积分成虚拟电角度。
 */
static void MotorApp_UpdateIfAngle(void) {
  if (startup.if_speed_rad_s < startup.if_speed_target_rad_s) {
    startup.if_speed_rad_s += MOTOR_APP_IF_ACCEL_RAD_S2 * foc.timer.Ts;
    if (startup.if_speed_rad_s > startup.if_speed_target_rad_s) {
      startup.if_speed_rad_s = startup.if_speed_target_rad_s;
    }
  }

  startup.if_angle += startup.if_speed_rad_s * startup.if_direction * foc.timer.Ts;
  startup.if_angle = FOC_WrapToPiFast(startup.if_angle);
}

/**
 * @brief 评估ObserverReady判据（接管前置条件）。
 *
 * 判据只用已有观测量，每条失败都写入motor_ready_fail_mask的独立位，便于现场定位：
 * 磁链幅值范围、初始化、转速下限、方向一致、角度与转速单拍跳变、I/F与观测角差、
 * 电流跟踪（|Id|与电流PI饱和）。
 * 其中"电流跟踪"按MOTOR_APP_OBSERVER_READY_SUSTAIN_MS去抖（瞬时失跟不算失败），
 * 其余条件必须本拍成立；全部成立且连续满足时间与累计电角度跨度后才置位ready。
 *
 * @note 本函数只做判断，不切换控制角；切换由接管状态完成。
 */
static void MotorApp_UpdateObserverReady(void) {
  /* angle_step/speed_step是相对上一拍的变化量，用于排除无法解释的跳变。 */
  float angle_step;
  float speed_step;
  /* load_angle是I/F虚拟角与观测磁链角之差，即允许存在的负载角。 */
  float load_angle;
  /* speed_rpm为本拍观测转速，正反转共用同一组判据。 */
  float speed_rpm;
  /* track_failed表示本拍出现"失跟或电压饱和"征兆。 */
  uint8_t track_failed;
  /* fail为本拍全部失败位的集合。 */
  uint32_t fail = 0U;

  speed_rpm = foc.observer.state.speed_rpm;

  /* 1) 磁链幅值在标定范围内（仅必要条件）。 */
  if ((foc.observer.state.psi_mag < foc.observer.config.psi_min) ||
      ((foc.observer.config.psi_max > 0.0f) &&
       (foc.observer.state.psi_mag > foc.observer.config.psi_max))) {
    fail |= MOTOR_APP_READY_FAIL_PSI;
  }

  /* 2) 观测器已初始化。 */
  if (foc.observer.state.initialized == 0U) {
    fail |= MOTOR_APP_READY_FAIL_INIT;
  }

  /* 3) 转速达到可观测下限：对应BEMF明显高于噪声与死区模型误差。 */
  if (fabsf(speed_rpm) < MOTOR_APP_OBSERVER_READY_MIN_RPM) {
    fail |= MOTOR_APP_READY_FAIL_SPEED;
  }

  /* 4) 观测方向必须与I/F拖动方向一致。 */
  if ((speed_rpm * startup.if_direction) <= 0.0f) {
    fail |= MOTOR_APP_READY_FAIL_DIR;
  }

  /* 5) 角度与转速的单拍变化有界。 */
  angle_step = FOC_WrapToPiFast(foc.observer.state.phase_raw -
                                startup.ready_last_phase_rad);
  speed_step = speed_rpm - startup.ready_last_speed_rpm;
  if (fabsf(angle_step) > MOTOR_APP_OBSERVER_READY_MAX_ANGLE_STEP) {
    fail |= MOTOR_APP_READY_FAIL_ANGLE_STEP;
  }
  if (fabsf(speed_step) > MOTOR_APP_OBSERVER_READY_MAX_SPEED_STEP) {
    fail |= MOTOR_APP_READY_FAIL_SPEED_STEP;
  }

  /* 6) I/F与观测角差在同步窗口内：允许非零负载角，不允许失控增长。 */
  load_angle = FOC_WrapToPiFast(startup.if_angle -
                                foc.observer.state.phase_raw);
  if (fabsf(load_angle) > MOTOR_APP_OBSERVER_READY_MAX_LOAD_ANGLE) {
    fail |= MOTOR_APP_READY_FAIL_SYNC;
  }

  /* 7) 电流跟踪：|Id|不持续偏大，且两个电流PI不持续饱和（去抖后判定）。 */
  track_failed = 0U;
  if ((fabsf(foc.state.i_dq.d) > MOTOR_APP_OBSERVER_READY_MAX_ID_A) ||
      (motor_control.id_pi.saturation != PI_SATURATION_NONE) ||
      (motor_control.iq_pi.saturation != PI_SATURATION_NONE)) {
    track_failed = 1U;
  }
  if (track_failed != 0U) {
    if (startup.ready_sustain_count <
        MOTOR_APP_MS_TO_TICKS(MOTOR_APP_OBSERVER_READY_SUSTAIN_MS)) {
      startup.ready_sustain_count++;
    }
  } else {
    startup.ready_sustain_count = 0U;
  }
  if (startup.ready_sustain_count >=
      MOTOR_APP_MS_TO_TICKS(MOTOR_APP_OBSERVER_READY_SUSTAIN_MS)) {
    fail |= MOTOR_APP_READY_FAIL_TRACK;
  }

  motor_ready_fail_mask = fail;

  /* 无论本拍是否失败都要更新历史值，下一拍的跳变判据以本拍为基准。 */
  startup.ready_last_phase_rad = foc.observer.state.phase_raw;
  startup.ready_last_speed_rpm = speed_rpm;

  if (fail != 0U) {
    startup.ready_count = 0U;
    startup.ready_span_rad = 0.0f;
    startup.observer_ready = 0U;
    return;
  }

  /* 条件成立：累计持续时间与累计电角度跨度，两者都满足才准许接管。 */
  if (startup.ready_count <
      MOTOR_APP_MS_TO_TICKS(MOTOR_APP_OBSERVER_READY_TIME_MS)) {
    startup.ready_count++;
  }
  startup.ready_span_rad += fabsf(angle_step);

  if ((startup.ready_count >=
       MOTOR_APP_MS_TO_TICKS(MOTOR_APP_OBSERVER_READY_TIME_MS)) &&
      (startup.ready_span_rad >= MOTOR_APP_OBSERVER_READY_SPAN_RAD)) {
    startup.observer_ready = 1U;
  }
}

/**
 * @brief 记录接管起点：初始角偏置与按允许角速度折算出的渐消拍数。
 *
 * @note 渐消时间由角差决定（上下限夹紧），不是固定值：角差越大渐消越慢，
 *       避免控制角相对观测角快速旋转导致电流矢量跳变。
 */
static void MotorApp_BeginObserverHandover(void) {
  /* offset为当前I/F虚拟角相对观测磁链角的偏置。 */
  float offset = FOC_WrapToPiFast(startup.if_angle -
                                  foc.observer.state.phase_raw);
  /* ticks为按最大角速度折算出的渐消拍数。 */
  uint32_t ticks;

  if (MOTOR_APP_HANDOVER_RATE_RAD_S > 0.0f) {
    ticks = (uint32_t)(fabsf(offset) /
                       (MOTOR_APP_HANDOVER_RATE_RAD_S * foc.timer.Ts));
  } else {
    ticks = 0U;
  }
  if (ticks < MOTOR_APP_MS_TO_TICKS(MOTOR_APP_HANDOVER_MIN_TIME_MS)) {
    ticks = MOTOR_APP_MS_TO_TICKS(MOTOR_APP_HANDOVER_MIN_TIME_MS);
  }
  if (ticks > MOTOR_APP_MS_TO_TICKS(MOTOR_APP_HANDOVER_MAX_TIME_MS)) {
    ticks = MOTOR_APP_MS_TO_TICKS(MOTOR_APP_HANDOVER_MAX_TIME_MS);
  }

  startup.handover_offset_rad = offset;
  startup.handover_ticks_total = ticks;
  startup.handover_ticks = 0U;
}

/**
 * @brief 闭环健康检查：角度失步或磁链出界持续存在时退流并报告。
 *
 * @note 判据只用|Id|与磁链幅值：闭环下dq坐标即转子坐标，Id持续偏大说明控制角
 *       已经跟不上转子。这是第一版安全网，不是完整的失步检测。
 */
static void MotorApp_CheckClosedLoopHealth(void) {
  /* unhealthy为1表示本拍检测到失步征兆。 */
  uint8_t unhealthy = 0U;

  if (fabsf(foc.state.i_dq.d) > MOTOR_APP_CLOSED_LOOP_MAX_ID_A) {
    unhealthy = 1U;
  }
  if ((foc.observer.state.psi_mag < foc.observer.config.psi_min) ||
      ((foc.observer.config.psi_max > 0.0f) &&
       (foc.observer.state.psi_mag > foc.observer.config.psi_max))) {
    unhealthy = 1U;
  }

  if (unhealthy != 0U) {
    startup.fault_count++;
  } else {
    startup.fault_count = 0U;
  }

  if (startup.fault_count >=
      MOTOR_APP_MS_TO_TICKS(MOTOR_APP_CLOSED_LOOP_FAULT_TIME_MS)) {
    motor_start_fail_reason = MOTOR_APP_FAIL_LOST_SYNC;
    motor_run_requested = 0U;
  }
}

/**
 * @brief 推进当前状态的内部量：观测器、虚拟角和定位计数。
 *
 * 本函数只更新状态自身的量，不产生控制参考，也不改变状态编号；
 * 状态切换在控制量写入之后由MotorApp_UpdateStateTransition()提交。
 */
static void MotorApp_AdvanceStateMachine(const Observer_Input_t *input) {
  switch (foc_motor_state) {
  case FOC_MOTOR_ALIGN:
    /* 定位段不运行观测器，只累加定位时间。 */
    startup.align_count++;
    break;

  case FOC_MOTOR_OPEN_LOOP_IF:
    /* I/F期间观测器后台运行，只用于可信判据；控制角仍是虚拟角。 */
    Observer_Run(&foc.observer, input);
    MotorApp_UpdateObserverReady();
    MotorApp_UpdateIfAngle();
    break;

  case FOC_MOTOR_OBSERVER_HANDOVER:
    /* 接管期间观测器必须继续运行：控制角以本拍磁链角为基准。 */
    Observer_Run(&foc.observer, input);
    startup.handover_ticks++;
    break;

  case FOC_MOTOR_CLOSED_LOOP:
    /* 闭环使用本拍刚更新的观测器状态，控制角与速度反馈同拍。 */
    Observer_Run(&foc.observer, input);
    MotorApp_CheckClosedLoopHealth();
    break;

  default:
    /* IDLE在进入中断主体时已被拦截，不会到达这里。 */
    break;
  }
}

/**
 * @brief 提交本拍结束时的状态切换。
 *
 * @note 必须在控制参考和PWM比较值都写入之后调用：切换拍仍然使用原状态的
 *       控制角和参考，新状态从下一拍生效，避免边界拍出现参考跳变。
 */
static void MotorApp_UpdateStateTransition(void) {
  switch (foc_motor_state) {
  case FOC_MOTOR_ALIGN:
    if (startup.align_count >= MOTOR_APP_MS_TO_TICKS(MOTOR_APP_ALIGN_TIME_MS)) {
      /* 定位角作为虚拟角初值，I/F从零速重新加速。 */
      startup.if_angle = startup.align_angle;
      startup.if_speed_rad_s = 0.0f;
      foc_motor_state = FOC_MOTOR_OPEN_LOOP_IF;
    }
    break;

  case FOC_MOTOR_OPEN_LOOP_IF:
    if (startup.observer_ready != 0U) {
      /* 观测器已可信：记录角偏置并进入接管，不再继续加速虚拟角。 */
      MotorApp_BeginObserverHandover();
      foc_motor_state = FOC_MOTOR_OBSERVER_HANDOVER;
    } else if (startup.if_speed_rad_s >= startup.if_speed_target_rad_s) {
      /* 已拖到检查速度仍不可信：有界验证窗口后判为启动失败并退流。 */
      startup.verify_count++;
      if (startup.verify_count >=
          MOTOR_APP_MS_TO_TICKS(MOTOR_APP_IF_VERIFY_TIME_MS)) {
        motor_start_fail_reason = MOTOR_APP_FAIL_OBSERVER_NOT_READY;
        motor_run_requested = 0U;
      }
    } else {
      startup.verify_count = 0U;
    }
    break;

  case FOC_MOTOR_OBSERVER_HANDOVER:
    if ((startup.handover_ticks_total > 0U) &&
        (startup.handover_ticks >= startup.handover_ticks_total)) {
      /* 偏置已渐消到0，控制角与闭环路径连续，可以进入闭环。 */
      foc_motor_state = FOC_MOTOR_CLOSED_LOOP;
    }
    break;

  default:
    break;
  }
}

/* ======================== 端电压融合 ======================== */

/**
 * @brief 每个控制周期更新观测器电压输入和实测权重。
 *
 * 三相端电压与TIM1同步采样，本拍已在中断内完成Clarke变换；这里只做融合。
 * 1200/900rpm构成迟滞区间，低速优先实测端电压，高速逐步切到PWM重构电压。
 * 包括切换在内，实际权重都按20ms渐变，不会立即跳到目标值。
 */
static void MotorApp_UpdateObserverVoltage(Observer_Input_t *input) {
  /* blend_step为本控制周期的权重增量。 */
  float blend_step;
  /* speed_abs_rpm用于按转速迟滞选择电压源，正反转共用阈值。 */
  float speed_abs_rpm;

  /* 三端电压均为对地电压；Clarke变换会消去三相共有的零序分量。 */
  input->measured_u_alpha = foc.state.u_alpha_beta_measured.alpha;
  input->measured_u_beta = foc.state.u_alpha_beta_measured.beta;

  /* 标量fabsf可直接生成FPU的VABS指令，无需调用面向数组的arm_abs_f32。 */
  speed_abs_rpm = fabsf(foc.observer.state.speed_rpm);

  if (speed_abs_rpm >= MOTOR_APP_CALCULATED_VOLTAGE_ENTER_RPM) {
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

/* ======================== 功率桥安全状态与启动 ======================== */

/**
 * @brief IDLE关闭功率输出，首次进入时清除闭环历史。
 *
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
  /* 观测器状态整体清零后重新初始化：电机和观测参数已在句柄内，这里只重置
   * 积分状态、PLL和初始磁链，因此传回句柄自身的参数指针是安全的。
   */
  foc.observer.state = (Observer_State_t){0};
  Observer_Init(&foc.observer, &foc.observer.motor, &foc.observer.config);
  voltage_source.measured_selected = 1U;
  voltage_source.measured_weight = 1.0f;
  motor_run_requested = 0U;
  motor_idle_reset_done = 1U;
}

/**
 * @brief 主循环响应run请求：初始化启动时序并打开功率桥。
 *
 * 目标速度和方向在本拍锁定：速度环内部参考同步到目标命令，避免启动瞬间经过
 * 调速斜坡；I/F拖动方向与目标符号一致。速度环在整个ALIGN/I/F阶段由仲裁器
 * 强制关闭，进入闭环后按键值使能。
 */
static void MotorApp_StartControlSequence(void) {
  /* if_target_rad_s是本次I/F的目标电角速度。 */
  float if_target_rad_s;
  /* pole_pairs来自观测器句柄保存的电机参数，避免此处再写死极对数。 */
  float pole_pairs;

  /* 开启六路PWM前暂停CH4，避免中途插入一次控制中断。 */
  if (HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }

  startup.align_angle = 0.0f;
  startup.align_count = 0U;
  startup.if_angle = startup.align_angle;
  startup.if_speed_rad_s = 0.0f;
  /* 清零判据、接管与失败计数，避免上一次运行的历史影响本次启动。 */
  startup.observer_ready = 0U;
  startup.ready_count = 0U;
  startup.ready_span_rad = 0.0f;
  startup.ready_last_phase_rad = 0.0f;
  startup.ready_last_speed_rpm = 0.0f;
  startup.ready_sustain_count = 0U;
  startup.ready_fail_mask = 0U;
  startup.verify_count = 0U;
  startup.fault_count = 0U;
  startup.handover_offset_rad = 0.0f;
  startup.handover_ticks = 0U;
  startup.handover_ticks_total = 0U;
  motor_ready_fail_mask = 0U;
  motor_start_fail_reason = MOTOR_APP_FAIL_NONE;

  /* 机械转速换算为电角速度：omega_e = rpm * 2pi/60 * pole_pairs。 */
  pole_pairs = (float)foc.observer.motor.pole_pairs;
  startup.if_direction =
      (motor_control.speed_command_rpm >= 0.0f) ? 1.0f : -1.0f;
  if_target_rad_s = fabsf(motor_control.speed_command_rpm) *
                    (CORDIC_TWO_PI_F / 60.0f) * pole_pairs;
  if (if_target_rad_s > MOTOR_APP_IF_MAX_SPEED_RAD_S) {
    if_target_rad_s = MOTOR_APP_IF_MAX_SPEED_RAD_S;
  }
  startup.if_speed_target_rad_s = if_target_rad_s;

  motor_control.speed_ref_rpm = motor_control.speed_command_rpm;
  /* 速度环的斜坡起点不在这里：速度PI使能那一拍由控制器用当前观测转速播种，
     使接管后从I/f转速平滑过渡到目标转速，而不是把参考直接设到目标。 */
  motor_control.speed_loop_enable = 1U;
  /* 模式切换历史对齐到"未使能"，使闭环第一拍必定执行一次无扰预加载。 */
  motor_control.speed_loop_enable_last = 0U;

  /* 每次启动均从实测端电压开始观测。 */
  voltage_source.measured_selected = 1U;
  voltage_source.measured_weight = 1.0f;

  foc_motor_state = FOC_MOTOR_ALIGN;
  motor_idle_reset_done = 0U;
  FOC_PWM_Start();
}

/* ======================== 调试控制台 ======================== */

/* 文本控制台的阻塞发送接口，由主循环命令处理调用。 */
static void MotorApp_DebugConsoleTx(const uint8_t *data, uint16_t length) {
  /* start记录等待发送器空闲的时间戳，用于限制阻塞时长。 */
  uint32_t start;

  /* 先禁止中断发起下一帧，等待当前DMA帧完全发完，再发送文本。 */
  motor_console_tx_active = 1U;
  __DMB();
  start = HAL_GetTick();
  while ((USART1->ISR & USART_ISR_TC) == 0U) {
    if ((HAL_GetTick() - start) >= 100U) {
      motor_console_tx_active = 0U;
      return;
    }
  }
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)data, length, 100U);
  motor_console_tx_active = 0U;
}

/**
 * @brief 一条纯文本状态回复，用于区分命令接收、校准等待和功率启动。
 *
 * @note 分行输出避免超过控制台192字节缓冲；各读数用于诊断，不保证同一拍快照。
 */
static void MotorApp_DebugStatus(int argc, char *argv[]) {
  (void)argc;
  (void)argv;

  DebugConsole_Printf(
      "STATUS run=%lu state=%u cal=%u idle_reset=%u adc_irq=%lu ARR=%lu CCR4=%lu MOE=%u rdy=%lu rdy_fail=0x%02lX fail=%lu\r\n",
      (unsigned long)motor_run_requested, (unsigned int)foc_motor_state,
      (unsigned int)foc.calibration.calibrated,
      (unsigned int)motor_idle_reset_done, (unsigned long)motor_adc_irq_count,
      (unsigned long)TIM1->ARR, (unsigned long)TIM1->CCR4,
      (unsigned int)((TIM1->BDTR & TIM_BDTR_MOE) != 0U),
      (unsigned long)startup.observer_ready,
      (unsigned long)motor_ready_fail_mask,
      (unsigned long)motor_start_fail_reason);
  DebugConsole_Printf("DRIVE Vbus=%.3f cmd=%.1f ref=%.1f rpm=%.1f speed_en=%lu\r\n",
      (double)foc.state.vbus, (double)motor_control.speed_command_rpm,
      (double)motor_control.speed_ref_active_rpm,
      (double)foc.observer.state.speed_rpm,
      (unsigned long)motor_control.reference.speed_loop_enable);
  DebugConsole_Printf("CURRENT Iq_ref=%.3f Id=%.3f Iq=%.3f Ud=%.3f Uq=%.3f Ulim=%.3f\r\n",
      (double)motor_control.iq_ref_active, (double)foc.state.i_dq.d,
      (double)foc.state.i_dq.q, (double)foc.state.u_dq.d,
      (double)foc.state.u_dq.q, (double)motor_control.voltage_limit);
  DebugConsole_Printf("PWM CCR=%lu,%lu,%lu CCER=0x%08lX CR1=0x%08lX\r\n",
      (unsigned long)TIM1->CCR1, (unsigned long)TIM1->CCR2,
      (unsigned long)TIM1->CCR3, (unsigned long)TIM1->CCER,
      (unsigned long)TIM1->CR1);
  DebugConsole_Printf("OBSERVER phase=%.3f flux=%.6f weight=%.3f U=%.2f V=%.2f W=%.2f ctrl=%.3f dtheta=%.3f\r\n",
      (double)foc.observer.state.phase_raw, (double)foc.observer.state.psi_mag,
      (double)voltage_source.measured_weight,
      (double)foc.state.u_abc_measured.a,
      (double)foc.state.u_abc_measured.b,
      (double)foc.state.u_abc_measured.c,
      (double)motor_control_angle_rad,
      (double)FOC_WrapToPiFast(motor_control_angle_rad -
                               foc.observer.state.phase_raw));
  /* ADC2注入组原始码诊断：JDR1为V相电流(Ib)，JDR2为V相端电压。
   * JSQR的JL字段决定注入序列长度，用于区分"V相没有被转换(JDR2恒为0)"
   * 和"V相实测确实为0V"两种情况。adc_cfg为上电回读自检结果，0时拒绝启动。
   */
  DebugConsole_Printf("ADC JDR1=%lu JDR2=%lu JSQR=0x%08lX adc_cfg=%lu\r\n",
      (unsigned long)ADC2->JDR1, (unsigned long)ADC2->JDR2,
      (unsigned long)ADC2->JSQR, (unsigned long)motor_adc_cfg_ok);
}

/**
 * @brief 将命令名绑定到现有控制结构，避免另外维护一份参数副本。
 * 注册接口最后一个参数为只读标志；范围只约束控制台写入，
 * 不等同于控制器输出限幅。例如速度PI的Iq限幅仍由controller.h定义。
 *
 * @note id/iq/speed_en注册的是外部命令字段，实际的每拍参考由应用层仲裁后
 *       写入motor_control.reference；status回复显示的是仲裁后的生效值。
 */
static HAL_StatusTypeDef MotorApp_RegisterDebugVariables(void) {
  /* success累积每个注册调用的结果，任意一项失败都会报告初始化失败。 */
  uint8_t success = 1U;

  success &= DebugConsole_RegisterF32("id", &motor_control.id_ref,
                                      -8.0f, 8.0f, false);
  /* 直接Iq模式同步放宽到10 A，保持与速度环输出范围一致，便于调试两种模式。 */
  success &= DebugConsole_RegisterF32("iq", &motor_control.iq_ref,
                                      -10.0f, 10.0f, false);
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
  /* 启动期诊断：ADC注入自检结果与本拍控制角。 */
  success &= DebugConsole_RegisterU32("adc_cfg", &motor_adc_cfg_ok, 0U, 1U,
                                      true);
  success &= DebugConsole_RegisterF32("ctrl_rad", &motor_control_angle_rad,
                                      -4.0f, 4.0f, true);
  /* 接管判据与启动失败的观察点。 */
  success &= DebugConsole_RegisterU32("rdy_fail", &motor_ready_fail_mask, 0U,
                                      0xFFU, true);
  success &= DebugConsole_RegisterU32("fail", &motor_start_fail_reason, 0U, 2U,
                                      true);

  return (success != 0U) ? HAL_OK : HAL_ERROR;
}

/* ======================== 波形输出 ======================== */

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
  /* f0至f5依次对应三相电流、转速、电角度和母线电压。 */
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

/* ======================== 模拟前端与功率级初始化 ======================== */

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

/**
 * @brief 回读注入序列寄存器，确认两路电流与三路端电压真的在注入序列里。
 *
 * HAL_ADCEx_InjectedConfigChannel()在ScanConvMode=DISABLE时会静默丢弃
 * InjectedNbrOfConversion、只把Rank1写进JSQR，而它仍返回HAL_OK，且本工程关闭了
 * USE_FULL_ASSERT，所以配置被截断时没有任何错误信号。V相端电压曾因此从未被转换。
 * 这里只能靠回读JSQR发现：校验失败只置标志并由run门控拒绝启动，不进Error_Handler
 * 死循环，便于现场通过串口定位。
 *
 * @return 1=与MOTOR_APP_ADCx_INJ_* 期望一致；0=不一致。
 */
static uint8_t MotorApp_CheckAdcInjectedConfig(void) {
  /* 两个ADC的注入序列寄存器快照。 */
  uint32_t jsqr1 = ADC1->JSQR;
  uint32_t jsqr2 = ADC2->JSQR;
  /* JL字段为转换次数-1，是"序列被截断"最直接的判据。 */
  uint32_t jl1 = (jsqr1 & ADC_JSQR_JL) >> ADC_JSQR_JL_Pos;
  uint32_t jl2 = (jsqr2 & ADC_JSQR_JL) >> ADC_JSQR_JL_Pos;
  uint8_t ok = 1U;

  if ((jl1 != MOTOR_APP_ADC1_INJ_JL) ||
      (((jsqr1 & ADC_JSQR_JSQ1) >> ADC_JSQR_JSQ1_Pos) != MOTOR_APP_ADC1_INJ_JSQ1) ||
      (((jsqr1 & ADC_JSQR_JSQ2) >> ADC_JSQR_JSQ2_Pos) != MOTOR_APP_ADC1_INJ_JSQ2) ||
      (((jsqr1 & ADC_JSQR_JSQ3) >> ADC_JSQR_JSQ3_Pos) != MOTOR_APP_ADC1_INJ_JSQ3) ||
      (((jsqr1 & ADC_JSQR_JSQ4) >> ADC_JSQR_JSQ4_Pos) != MOTOR_APP_ADC1_INJ_JSQ4)) {
    ok = 0U;
  }

  if ((jl2 != MOTOR_APP_ADC2_INJ_JL) ||
      (((jsqr2 & ADC_JSQR_JSQ1) >> ADC_JSQR_JSQ1_Pos) != MOTOR_APP_ADC2_INJ_JSQ1) ||
      (((jsqr2 & ADC_JSQR_JSQ2) >> ADC_JSQR_JSQ2_Pos) != MOTOR_APP_ADC2_INJ_JSQ2)) {
    ok = 0U;
  }

  if (ok == 0U) {
    /* 打印实际JSQR，便于直接对照adc.c判断是哪一段被改写。 */
    DebugConsole_Printf("ADC INJ FAIL ADC1 JSQR=0x%08lX JL=%lu ADC2 JSQR=0x%08lX JL=%lu\r\n",
        (unsigned long)jsqr1, (unsigned long)jl1,
        (unsigned long)jsqr2, (unsigned long)jl2);
    DebugConsole_Printf("ADC INJ expect ADC1 JL=3 JSQ=3,12,11,14 ; ADC2 JL=1 JSQ=3,17 ; run disabled\r\n");
  }

  return ok;
}

/**
 * @brief 强制将三相功率级的6路PWM控制引脚置为安全低电平
 *
 * @note 该函数应在系统启动早期、TIM1和相关GPIO被配置为复用PWM功能之前调用，
 *       防止MCU上电/复位过程中PWM引脚处于不确定状态而误导通MOSFET。
 *
 *       当前三相PWM引脚：
 *         PA8  -> TIM1_CH1      PB13 -> TIM1_CH1N
 *         PA9  -> TIM1_CH2      PB14 -> TIM1_CH2N
 *         PA10 -> TIM1_CH3      PB15 -> TIM1_CH3N
 *
 *       执行完成后6路引脚均为普通推挽输出并保持低电平，后续由CubeMX生成的
 *       GPIO/TIM初始化重新配置为复用功能。
 */
void MotorApp_ForcePowerStageSafe(void) {
  /* gpio复用同一结构体依次配置高低桥臂引脚。 */
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

/* ======================== 对外接口 ======================== */

/** @brief 返回应用层唯一的FOC控制器实例，供协议层更新目标。 */
FOC_Control_t *MotorApp_GetControl(void) {
  return &motor_control;
}

/** @brief 写入运行请求，实际启停仍由MotorApp控制节拍执行。 */
void MotorApp_RequestRun(uint8_t run) {
  motor_run_requested = (run != 0U) ? 1U : 0U;
}

/**
 * @brief 初始化电机控制应用层
 *
 * @return HAL_OK    初始化成功
 * @return HAL_ERROR 任一步骤初始化失败
 *
 * @note 前置条件：main已在MX_TIM1_Init之前调用FOC_Data_Init，完成板级参数初始化。
 *       本函数依次完成DWT延时、CORDIC配置、波形输出、控制器、功率桥安全状态、
 *       调试串口、模拟前端校准、母线初值和ADC注入组启动。
 *
 *       函数返回时：ADC注入采样和TIM1 CH4触发已经工作；HAL启动CH4会置位MOE，
 *       但CH1~3及其互补通道尚未使能，因此三相功率桥不会在本函数中开始输出。
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
    foc.state.temperature_c = BoardAdc_GetMeasurements()->temperature_c;
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
   * 注入序列已由MX_ADC1_Init/MX_ADC2_Init写定，这里回读校验。失败只记录标志，
   * 由MotorApp_Process拒绝run请求；不进Error_Handler，保持串口可诊断。
   */
  motor_adc_cfg_ok = MotorApp_CheckAdcInjectedConfig();

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
 * @note 该函数在main()的while(1)中循环调用，只处理不要求严格实时性的任务。
 *       规则组ADC轮询和串口命令解析可能产生等待或不确定执行时间，
 *       因此不放入25kHz的电流环控制中断，避免影响FOC控制周期。
 */
void MotorApp_Process(void) {
  /* 只发送一次READY，避免主循环高速运行时重复占用串口。 */
  static uint8_t ready_reported = 0U;
  /* 只报告一次"因ADC配置无效而拒绝启动"，避免阻塞时刷屏。 */
  static uint8_t adc_cfg_reported = 0U;

  if (ready_reported == 0U) {
    ready_reported = 1U;
    DebugConsole_Printf("READY: set run 1 / set run 0 / status\r\n");
  }

  /* 优先解析命令，避免规则组的轮询等待增加启停请求延迟。 */
  DebugConsole_Process();

  /*
   * 轮询ADC规则组，仅采集母线电压和MCU温度。
   * U/V/W三相端电压已进入TIM1同步Injected序列，不再从主循环规则组读取。
   */
  if (BoardAdc_Update() == HAL_OK) {
    foc.state.vbus = BoardAdc_GetMeasurements()->vbus_voltage;
    foc.state.temperature_c = BoardAdc_GetMeasurements()->temperature_c;
  }

  /* ADC注入配置不匹配时拒绝启动：采样通道不可信时闭环没有意义。 */
  if ((motor_run_requested != 0U) && (motor_adc_cfg_ok == 0U) &&
      (adc_cfg_reported == 0U)) {
    adc_cfg_reported = 1U;
    DebugConsole_Printf("run ignored: ADC injected config invalid\r\n");
  }

  /* 校准完成、IDLE复位完成且ADC注入配置正确才能启动，重复run 1不会重新初始化运行电机。
   * 校准期间收到run 1则等待校准结束；run 0可以取消该请求。
   */
  if ((motor_run_requested != 0U) &&
      (motor_adc_cfg_ok != 0U) &&
      (foc.calibration.calibrated != 0U) &&
      (motor_idle_reset_done != 0U) &&
      (foc_motor_state == FOC_MOTOR_IDLE)) {
    MotorApp_StartControlSequence();
  }
}

/**
 * @brief 注入转换完成后的实时入口，只有ADC1回调执行完整控制流程。
 * ADC1依次采Ia、Ic、U端电压、W端电压，ADC2依次采Ib、V端电压；
 * 两个ADC由同一个TIM1 TRGO触发。ADC2不打开注入中断，控制入口仅由
 * ADC1四个Injected Rank全部完成后的JEOS产生，此时两路ADC结果均已就绪。
 */
void MotorApp_OnInjectedConversion(ADC_HandleTypeDef *hadc) {
  /* 校准计数器只在ADC注入中断上下文中递增，达到样本数后锁定零偏。 */
  static uint16_t calibration_count = 0U;
  /* observer_input收集本拍电流、母线、占空比和端电压输入。 */
  Observer_Input_t observer_input = {0};
  /* 三个ADC原始码分别对应U、V、W相电流采样通道。 */
  uint16_t adc_a;
  uint16_t adc_b;
  uint16_t adc_c;
  /* 三相端电压Injected原始码：ADC1 JDR3/JDR4为U/W，ADC2 JDR2为V。 */
  uint16_t phase_u_raw;
  uint16_t phase_v_raw;
  uint16_t phase_w_raw;

  if ((hadc == NULL) || (hadc->Instance != ADC1)) {
    return;
  }
  motor_adc_irq_count++;

  /* 串口解析完run 0后，下一次采样中断先退出闭环并关断功率输出。 */
  if (motor_run_requested == 0U) {
    foc_motor_state = FOC_MOTOR_IDLE;
  }

  /*
   * 安全状态关闭功率桥，主动控制状态进入FOC执行路径。
   * 当前可运行状态为ALIGN、OPEN_LOOP_IF和CLOSED_LOOP，其余一律回到IDLE。
   */
  if (MotorApp_IsControlState(foc_motor_state) == 0U) {
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

  /*
   * 端电压与电流来自同一次TIM1触发：
   * ADC2的V相在第二个Rank完成，ADC1随后完成U/W两个Rank并产生JEOS。
   * 三路均使用板上100kΩ/5.1kΩ分压比例恢复为端子对地实际电压。
   */
  phase_u_raw = (uint16_t)ADC1->JDR3;
  phase_v_raw = (uint16_t)ADC2->JDR2;
  phase_w_raw = (uint16_t)ADC1->JDR4;
  foc.state.u_abc_measured.a =
      (float)phase_u_raw * BOARD_ADC_COUNT_TO_VOLTAGE;
  foc.state.u_abc_measured.b =
      (float)phase_v_raw * BOARD_ADC_COUNT_TO_VOLTAGE;
  foc.state.u_abc_measured.c =
      (float)phase_w_raw * BOARD_ADC_COUNT_TO_VOLTAGE;
  FOC_Clarke(&foc.state.u_abc_measured, &foc.state.u_alpha_beta_measured);

  /* 先用上次计算的占空比与本拍电流观测，再计算新的电压命令。 */
  observer_input.duty_a = foc.svpwm.duty_a;
  observer_input.duty_b = foc.svpwm.duty_b;
  observer_input.duty_c = foc.svpwm.duty_c;
  observer_input.vbus = foc.state.vbus;
  observer_input.i_alpha = foc.state.i_alpha_beta.alpha;
  observer_input.i_beta = foc.state.i_alpha_beta.beta;
  MotorApp_UpdateObserverVoltage(&observer_input);

  /*
   * 只有主动控制状态才推进状态机、执行电流环并写入功率PWM比较值。
   * IDLE下必须完全跳过：否则电流PI会在关桥期间积分饱和，下一次run进入ALIGN时
   * 从饱和积分起步，造成启动电流冲击。
   */
  if (MotorApp_IsControlState(foc_motor_state) != 0U) {
    /*
     * 控制顺序固定为：推进状态内部量 -> 仲裁参考 -> 执行电流环 -> 写PWM
     * -> 提交状态切换。切换放在最后，保证边界拍仍使用原状态的控制量。
     */
    MotorApp_AdvanceStateMachine(&observer_input);
    MotorApp_ResolveControlReference();
    MotorApp_RunCurrentLoop(&motor_control);

    TIM1->CCR1 = foc.svpwm.ccr_a;
    TIM1->CCR2 = foc.svpwm.ccr_b;
    TIM1->CCR3 = foc.svpwm.ccr_c;

    MotorApp_UpdateStateTransition();
  }

  /* 高频更新估算值，供低频CAN反馈使用；IDLE时函数会同步清零。 */
  FOC_UpdateBusCurrentEstimate(&foc);

  /* VOFA通道：Iu(A)、Iv(A)、Iw(A)、机械转速(rpm)、观测电角度(deg)、母线(V)。
   * 所有主动控制状态都发送：ALIGN / I-f / 接管期间的相电流与角度同样需要观察。
   */
  if ((just_float_enabled != 0U) &&
      (motor_console_tx_active == 0U) &&
      (MotorApp_IsControlState(foc_motor_state) != 0U) &&
      ((USART1->ISR & USART_ISR_TC) != 0U)) {
    (void)MotorApp_SendJustFloat(
        foc.state.i_abc.a, foc.state.i_abc.b, foc.state.i_abc.c,
        foc.observer.state.speed_rpm,
        foc.observer.state.phase_raw * RAD_TO_DEG_F, foc.state.vbus);
  }
}
