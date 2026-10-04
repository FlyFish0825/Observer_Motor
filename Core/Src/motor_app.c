/**
 * @file motor_app.c
 * @brief 无感启动应用层 + 25 kHz 实时控制入口。
 *
 * 架构：向上被 main() 主循环与 ADC 注入中断调用，向下调用纯算法层
 * （controller.c 电流/速度环、observer.c 磁链观测器、foc_math.c 的
 * Clarke/Park/SVPWM/CORDIC），并直接读写 TIM1、ADC1/ADC2、USART1。
 * CubeMX 重新生成外设代码时不会覆盖本文件，"状态机 + 外设时序"的
 * 胶水逻辑集中在这里。
 *
 * 两个执行域（务必区分）：
 *  - 中断域（硬实时 25 kHz，单拍 40 us）：ADC1 JEOS ->
 *    MotorApp_OnInjectedConversion()。采样、观测器、状态机、电流环、
 *    PWM 比较值写入都在这条路径上，耗时用 DWT->CYCCNT 统计（status 的
 *    TIMING 行输出）。
 *  - 主循环域（软实时）：串口命令解析、规则组轮询（母线/温度）、阻塞
 *    打印只在这里做，绝不放回中断。
 * 两域之间只靠文件作用域 volatile 标志与共享结构体交接；需要成组一致
 * 的地方用临界区（__disable_irq）或 BoardAdc 的 seqlock 保护。
 *
 * 启动状态机：IDLE -> ALIGN（固定角建磁场）-> OPEN_LOOP_IF（虚拟角拖动）
 * -> OBSERVER_HANDOVER（控制角渐切换到观测角）-> CLOSED_LOOP（双闭环）。
 * 状态编号对外公开（CAN/串口），不得改动数值；新增状态只需扩展
 * MotorApp_IsControlState()。上电前 1000 拍（约 40 ms）只做电流零偏校准，
 * 期间桥保持关闭。运行中换向先闭环制动，实际速度进入 ±800 rpm 后
 * 才由 I/F 虚拟角延续斜率穿零；低速段不把观测器当作控制角来源。
 */
#include "motor_app.h"

#include "adc.h"
#include "board_adc.h"
#include "bsp_dwt.h"
#include "controller.h"
#include "debug_console.h"
#include "foc_math.h"
#include "main.h"
#include "motor_calibration.h"
#include "motor_protocol.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"

#include "arm_math.h"
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* ======================== 应用层参数 ======================== */

/* 实时控制节拍 = PWM 载波频率（中央对齐，ARR=3399 -> 170M/6800 = 25 kHz），
 * 也是电流环与状态机计数的"拍"频率，Ts = 40 us 存于 foc.timer.Ts。
 * 改 TIM1 的 ARR/时钟树必须同步改这里，否则所有时间阈值按错误节拍计时。 */
#define MOTOR_APP_CONTROL_HZ 25000U

/* 毫秒 -> 控制拍数（先乘后除避免截断）：时间阈值统一折算成中断计数值。 */
#define MOTOR_APP_MS_TO_TICKS(ms) \
  (((uint32_t)(ms) * MOTOR_APP_CONTROL_HZ) / 1000U)

/* 端电压 RC（100k/5.1k/68nF，fc 约 482Hz）会带来幅值衰减与相位滞后：
 * 低速用实测端电压，高速切占空比重构电压；1200/900 rpm 迟滞（宽 300 rpm）
 * 防止切换点抖动，权重按 20 ms 渐变避免电压突跳冲击磁链积分器（逻辑
 * 已内联在 MotorApp_OnInjectedConversion 中）。 */
#define MOTOR_APP_CALCULATED_VOLTAGE_ENTER_RPM 1200.0f
#define MOTOR_APP_MEASURED_VOLTAGE_RETURN_RPM   900.0f
#define MOTOR_APP_VOLTAGE_BLEND_TIME_S            0.020f

/* ALIGN：固定角（0 rad = alpha 轴）注入 Id 把转子拽到已知位置，给观测器
 * 提供初始角度基准；Iq=0 不产生转矩。时长/电流为台架整定值，带载越重
 * 定位时间越长。读者：ResolveControlReference / UpdateStateTransition。 */
#define MOTOR_APP_ALIGN_TIME_MS 30U
#define MOTOR_APP_ALIGN_ID_A 1.0f

/* I/F 开环拖动：只给 Iq（幅值 if_iq_magnitude，符号由 if_direction 施加），
 * 虚拟电角度按受限加速度爬升。IF_IQ 调大抗载但发热、易饱和；调小重载可能
 * 失步（观测器判据报 TRACK 或验证超时停机）。加速度折算机械约 819 rpm/s
 * （7 极对），只取正值，方向由 if_direction 统一施加，正反转共用。 */
#define MOTOR_APP_IF_IQ_A 1.5f
#define MOTOR_APP_IF_ACCEL_RAD_S2 600.0f

/* |目标|<=800 rpm 时保持虚拟角强制开环；观测器只作辅助估计，不参与控制角。 */
#define MOTOR_APP_FORCED_OPEN_LOOP_MAX_RPM 800.0f

/* 接管目标转速：先拖到 min(|命令|, 1000 rpm) 完成接管，再由速度环爬到
 * 用户目标——接管点固定可复现，中低速观测更可靠。 */
#define MOTOR_APP_HANDOFF_TARGET_RPM 1000.0f

/* 虚拟速度到位后的有界验证窗口：窗口内观测器始终不可信则停机
 * （FAIL_OBSERVER_NOT_READY）。 */
#define MOTOR_APP_IF_VERIFY_TIME_MS 1500U

/* 从 ALIGN 到进闭环的总时间上限（兜住任何判据卡死，防止长时间开环发热）。 */
#define MOTOR_APP_START_TOTAL_TIME_MS 4000U

/* ---- ObserverReady 判据参数（读者：MotorApp_UpdateObserverReady）---- */

/* 判据须连续成立的最短时长（去抖：任一位失败立即清零重计）。 */
#define MOTOR_APP_OBSERVER_READY_TIME_MS 30U

/* 最低观测转速绝对值：反电动势不足以支撑角度估计则失败。 */
#define MOTOR_APP_OBSERVER_READY_MIN_RPM 800.0f

/* 窗口内必须累计覆盖的净电角度（2*pi = 整一圈；抖动会正负抵消）。 */
#define MOTOR_APP_OBSERVER_READY_SPAN_RAD CORDIC_TWO_PI_F

/* 单拍角/转速跳变上限（正常单拍角增量约 0.032 rad），超限判 atan2 跨 ±pi
 * 或 PLL 被扰动带飞。 */
#define MOTOR_APP_OBSERVER_READY_MAX_ANGLE_STEP 0.35f
#define MOTOR_APP_OBSERVER_READY_MAX_SPEED_STEP 100.0f

/* 绝对负载角判据开关：0=关闭（I/F 固有负载角在重载下会误杀；"相对漂移 +
 * 整圈净覆盖 + 电流跟踪"已能排除假同步），1=恢复 85 度门限。编译期常量。 */
#define MOTOR_APP_OBSERVER_READY_ANGLE_CHECK_ENABLE 0U
#define MOTOR_APP_OBSERVER_READY_MAX_LOAD_ANGLE 1.483529864f /* 85 deg */

/* 观测转速与虚拟转速允许偏差（约目标的 4%）：观测器必须锁在同一旋转磁场。
 * 这里使用 Observer 的两级 30 Hz 速度滤波值，避免 PLL 原始速度的单拍尖峰
 * 把已经稳定的 Ready 窗口反复清零；原始速度仍由最低转速、方向和单拍跳变门控
 * 约束。验证窗口在 I/F 速度爬升约 1.2 s 后才开始，滤波器不会带来启动初始滞后。 */
#define MOTOR_APP_OBSERVER_READY_SPEED_ERROR_RPM 40.0f

/* 窗口内相对锚点的漂移上限（容忍固定负载角，只查慢漂）。
 *
 * 这个窗口同时允许观测转速与虚拟转速相差 40 rpm。对当前 7 极对电机，
 * 该误差在 30 ms 内最多积成约 0.88 rad；若仍使用 10°（0.1745 rad），
 * 即使观测器已经稳定在约 983 rpm、虚拟速度为 1000 rpm，也会在 Ready
 * 计数完成前必然清零。取 1.0 rad 保留少量余量，仍会拦住更明显的失步，
 * 绝对角度安全门仍由 MOTOR_APP_OBSERVER_READY_ANGLE_CHECK_ENABLE 独立控制。 */
#define MOTOR_APP_OBSERVER_READY_DRIFT_RAD 1.0f /* 当前 7 极对/30 ms/±40 rpm 的上界约 0.88 rad */

/* 电流跟踪门限：I/F 只给 Iq，|Id| 过大说明角度错位；Iq 偏差大说明环跟
 * 不上；任一 PI 饱和也算失败。 */
#define MOTOR_APP_OBSERVER_READY_MAX_ID_A 0.8f
#define MOTOR_APP_OBSERVER_READY_MAX_IQ_ERROR_A 0.5f

/* 电流跟踪连续失败去抖时长（2 ms = 50 拍）。 */
#define MOTOR_APP_OBSERVER_READY_SUSTAIN_MS 2U

/* 可选 I/F 轻载电流削减（默认关，set if_trim 1 打开）：只在同步后的稳定
 * 支路缓慢下调（2 A/s，下限 0.30 A），负载角须在 55~85 度之间，
 * 绝不放宽任何角度判据。 */
#define MOTOR_APP_IF_CURRENT_FLOOR_A 0.30f
#define MOTOR_APP_IF_CURRENT_TRIM_A_PER_S 2.0f
#define MOTOR_APP_IF_TRIM_ANGLE_RAD 0.959931089f /* 55 deg */
#define MOTOR_APP_IF_TRIM_MAX_ANGLE_RAD 1.483529864f /* 85 deg */

/* 接管偏置渐变速率：控制角从虚拟角平滑切到观测角（三次 smoothstep 峰值
 * 斜率 1.5，总拍数 = 1.5*|offset|/(RATE*Ts)+1 再夹到 20~150 ms）：下限保证
 * PI 积分器搬运不突变，上限避免长时间处于混合坐标系累积误差。 */
#define MOTOR_APP_HANDOVER_RATE_RAD_S 20.0f
#define MOTOR_APP_HANDOVER_MIN_TIME_MS 20U
#define MOTOR_APP_HANDOVER_MAX_TIME_MS 150U

/* 接管/闭环跟踪健康门限。当前台架策略不因高速电流误差自动停机；保留
 * 参数供后续需要时重新打开软件健康闸门。 */
#define MOTOR_APP_CLOSED_LOOP_HEALTH_ENABLE 0U
#define MOTOR_APP_CLOSED_LOOP_MAX_ID_A 1.5f
#define MOTOR_APP_CLOSED_LOOP_MAX_IQ_ERROR_A 1.5f
#define MOTOR_APP_CLOSED_LOOP_FAULT_TIME_MS 20U

/* 接管后的参考保持只有固定 50 ms，不再等最终转速稳定才放开运行斜率。 */
#define MOTOR_APP_START_IQ_SLEW_A_S 20.0f
#define MOTOR_APP_SPEED_SETTLE_MS 50U
/* 运行中只平滑转矩阶跃，不限制最终速度或可用电压。 */
#define MOTOR_APP_RUN_IQ_SLEW_A_S 100.0f
/* 只在可信闭环速度区估计机械加速度：5 ms 差分、10 ms 一阶滤波。 */
#define MOTOR_APP_ACCEL_SAMPLE_MS 5U
#define MOTOR_APP_ACCEL_FILTER_S 0.010f
#define MOTOR_APP_ACCEL_VALID_SAMPLES 3U

/* ADC 注入序列期望值，与 adc.c 的 MX_ADCx_Init 保持一致（JL=转换次数-1，
 * JSQx=第 x 个 Rank 的通道号）。只用于上电回读自检：HAL 在 ScanConvMode=
 * DISABLE 时会静默截断注入序列（V 相端电压曾因此从未被采样、JDR2 恒 0），
 * 改动 adc.c 的注入通道必须同步这里，不一致时自检拒绝启动。 */
#define MOTOR_APP_ADC1_INJ_JL 3U
#define MOTOR_APP_ADC1_INJ_JSQ1 3U  /* Ia  ADC1_IN3  */
#define MOTOR_APP_ADC1_INJ_JSQ2 12U /* Ic  ADC1_IN12 */
#define MOTOR_APP_ADC1_INJ_JSQ3 11U /* U端 ADC1_IN11 */
#define MOTOR_APP_ADC1_INJ_JSQ4 14U /* W端 ADC1_IN14 */
#define MOTOR_APP_ADC2_INJ_JL 1U
#define MOTOR_APP_ADC2_INJ_JSQ1 3U  /* Ib  ADC2_IN3  */
#define MOTOR_APP_ADC2_INJ_JSQ2 17U /* V端 ADC2_IN17 */

/* 取 JSQR 中某 Rank 的通道号字段：field 与 ADC_JSQR_x / ADC_JSQR_x_Pos 同名。 */
#define MOTOR_APP_JSQR_FIELD(jsqr, field) \
  (((jsqr) & ADC_JSQR_##field) >> ADC_JSQR_##field##_Pos)

/* ======================== 应用层类型 ======================== */

/* 端电压来源（迟滞判决的存储值：低速用实测、高速用占空比重构）。 */
typedef enum {
  MOTOR_APP_VOLTAGE_CALCULATED = 0, /* 占空比 x Vbus 重构电压 */
  MOTOR_APP_VOLTAGE_MEASURED = 1,   /* 板上实测端电压 */
} MotorApp_VoltageSrc_t;

/* 启动失败原因码（写 motor_start_fail_reason 与故障快照）。NONE 兼作
 * "无故障/新启动清零"；一旦 EnterFault 闭锁，必须显式 run 0 才能重启；
 * 编号可扩展但不要复用已用编号。 */
typedef enum {
  MOTOR_APP_FAIL_NONE = 0UL,               /* 无故障 */
  MOTOR_APP_FAIL_OBSERVER_NOT_READY = 1UL, /* I/F 验证窗口内观测器始终不可信 */
  MOTOR_APP_FAIL_LOST_SYNC = 2UL,          /* 接管/闭环跟踪/磁链/掉速连续异常 */
  MOTOR_APP_FAIL_START_COMMAND = 3UL,      /* 目标转速或启动参数非法 */
  MOTOR_APP_FAIL_DIRECTION_CHANGE = 4UL,   /* 保留码：当前换向走 I/F 过零，不再主动触发 */
  MOTOR_APP_FAIL_START_TIMEOUT = 5UL,      /* 未进闭环总超时 */
  MOTOR_APP_FAIL_NONFINITE = 6UL,          /* 关键浮点量出现 NaN/Inf */
  MOTOR_APP_FAIL_HANDOVER_CURRENT = 7UL,   /* 接管电流异常/转矩反向 */
  MOTOR_APP_FAIL_BUS_INVALID = 8UL,        /* 母线电压非法 */
} MotorApp_Fail_t;

/* ObserverReady 判据失败位（按位或累积，0=全部通过；串口显示范围 0~0xFFF，
 * 新增判据用空闲位并保持 12 位以内）。 */
typedef enum {
  MOTOR_APP_READY_FAIL_PSI = 1UL << 0,         /* 磁链幅值越界(psi_max<=0 不查上限) */
  MOTOR_APP_READY_FAIL_INIT = 1UL << 1,        /* 观测器未初始化 */
  MOTOR_APP_READY_FAIL_SPEED = 1UL << 2,       /* 转速低于下限 */
  MOTOR_APP_READY_FAIL_DIR = 1UL << 3,         /* 转向与拖动方向不符 */
  MOTOR_APP_READY_FAIL_ANGLE_STEP = 1UL << 4,  /* 单拍观测角跳变超限 */
  MOTOR_APP_READY_FAIL_SPEED_STEP = 1UL << 5,  /* 单拍观测转速跳变超限 */
  MOTOR_APP_READY_FAIL_SYNC = 1UL << 6,        /* 绝对负载角超限(判据开关) */
  MOTOR_APP_READY_FAIL_TRACK = 1UL << 7,       /* 电流持续跟踪失败 */
  MOTOR_APP_READY_FAIL_SPEED_MATCH = 1UL << 8, /* 观测/虚拟转速偏差超限 */
  MOTOR_APP_READY_FAIL_DRIFT = 1UL << 9,       /* 窗口内相对锚点漂移超限 */
  MOTOR_APP_READY_FAIL_RAMP = 1UL << 10,       /* 尚未进入验证窗口 */
  MOTOR_APP_READY_FAIL_NONFINITE = 1UL << 11,  /* 判据输入出现 NaN/Inf */
} MotorApp_ReadyFail_t;

/* 波形通道选择（set wave_mode；诊断通道 25 分频降到 1 kHz）。 */
typedef enum {
  MOTOR_APP_WAVE_DEFAULT = 0, /* 三相电流/滤波转速/电角度/母线/原始转速 */
  MOTOR_APP_WAVE_STARTUP = 1, /* 启动时序 7 通道 */
  MOTOR_APP_WAVE_CURRENT = 2, /* 电流跟踪 7 通道 */
  MOTOR_APP_WAVE_VOLTAGE = 3, /* 控制速度/参考/Iq/电压饱和诊断 */
  MOTOR_APP_WAVE_TIMING = 4,  /* DWT 周期/时间/预算占用诊断 */
} MotorApp_WaveMode_t;

/* 端电压来源选择与融合权重：中断域每拍更新、主循环只读显示；
 * "目标已切换"与"权重已收敛"之间有一阶延迟，只用于显示、属设计允许。 */
typedef struct {
  volatile uint32_t measured_selected; /* 当前来源，取 MotorApp_VoltageSrc_t 值 */
  volatile float measured_weight;      /* 实测权重 0~1，每拍变化 ±Ts/0.020 */
} MotorApp_VoltageSource_t;

/* 启动流程全部中间状态：生命周期 = 一次启动（StartControlSequence 整体
 * 清零后由中断域推进）；无 volatile——主循环只读做诊断显示，允许读到
 * 某一拍的中间状态。字段按职能分组，末尾三个为接管专用量。 */
typedef struct {
  uint32_t align_count;        /* 定位拍数，达 750(30ms) 退出 ALIGN（固定角
                                  恒 0 rad = alpha 轴，与 Observer_Init 的
                                  初始磁链方向对应） */
  float if_angle;              /* 虚拟电角度(rad)，"旋转磁场指令角"而非
                                  转子真实角，归一化到 [-pi,pi) */
  float if_speed_rad_s;        /* 虚拟电角速度幅值(rad/s)，符号见 if_direction */
  float if_speed_target_rad_s; /* 目标电角速度幅值 */
  float if_accel_rad_s2;       /* 本拍虚拟电角加速度幅值；运行段由实测斜率播种 */
  float if_direction;          /* 拖动方向 ±1，同时作用于角积分/Iq/判据 */
  float reversal_target_direction; /* 换向目标方向 ±1；换向过零前保持旧方向 */
  uint8_t reversal_braking;    /* 1=沿旧方向强制减速，0=已翻转并重新加速 */
  uint8_t forced_open_loop;     /* 1=低速/换向强制 I/F，不等待 ObserverReady */
  uint8_t runtime_if;          /* 1=运行中低速段，不重新套用静止启动斜率 */
  uint8_t closed_loop_braking; /* 1=保持观测闭环制动，等待实际降到低速 */
  float if_id_reference;       /* 闭环转 I/F 时承接旧 Id，再渐消至零 */
  float if_iq_reference;       /* 运行 I/F 的有符号转矩参考，独立于最终转向 */
  float if_ramp_rpm_s;         /* 本段继承的机械斜率幅值，不随远端目标放大 */
  uint8_t if_measured_ramp;     /* 1=斜率/转矩来自闭环有效减速；0=起步斜率回退 */
  float measured_accel_rpm_s;  /* 最后可信区的有符号加速度估计 */
  float accel_last_rpm;
  uint32_t accel_count;
  uint32_t accel_samples;
  uint8_t accel_initialized;
  uint32_t segment;            /* 每次重新进入运行 I/F 递增，关联诊断快照 */

  float ready_last_phase_rad;   /* 上一拍观测角：算单拍角增量 */
  float ready_last_speed_rpm;   /* 上一拍观测转速：算单拍增量 */
  float ready_span_rad;         /* 窗口内带符号净角度覆盖(夹>=0)，
                                   来回抖动正负抵消，只计单向覆盖 */
  uint32_t ready_count;         /* 判据连续成立拍数(去抖)，任一失败清零；
                                   ==0 时兼作"窗口第一拍"重设锚点 */
  uint32_t ready_max_count;     /* 本次启动达到过的最长连续通过拍数 */
  uint32_t ready_sustain_count; /* 电流跟踪连续失败拍数(2ms 去抖) */
  uint8_t observer_ready;       /* 综合结论：连续达标 且 覆盖整一圈 */

  float if_iq_magnitude;        /* I/F Iq 幅值(A，恒正)；if_trim 可缓慢下调 */
  uint32_t if_hold_count;       /* 速度到位后的有界保持计数(只增不减，
                                   与每拍重置的 ready_count 不同) */
  uint32_t total_count;         /* 未进闭环的累计拍数(总超时用) */
  float ready_anchor_delta;     /* 窗口第一拍 delta：相对漂移判据的锚点 */
  float ready_speed_error;      /* 诊断：speed_match - VirtualRpm()，rpm */

  float handover_id_obs;        /* 接管起点：上一拍 dq 参考旋转到观测系的
                                   分量；d 随 blend 渐消、q(转矩)恒定 */
  float handover_iq_obs;
  float handover_last_offset;   /* 上一拍已卸掉的偏置：相邻帧坐标系旋转量，
                                   供 PI 积分器随坐标搬运 */
  uint32_t speed_settle_count;  /* 闭环淡入计数：Id 淡入比例+斜坡放开时刻 */
  uint8_t speed_startup_active; /* 1=启动过渡中（接管前就置 1，故最低转速
                                   检查在接管阶段也生效）；0=常态运行 */
  uint8_t if_verify_active;     /* 1=速度到位进入有界验证窗口 */
  uint32_t fault_count;         /* 闭环健康连续异常拍数(500 拍跳闸) */

  float handover_offset_rad;    /* 接管初始角偏置(rad)，随 smoothstep 渐消 */
  uint32_t handover_ticks;      /* 接管已运行拍数(封顶 total) */
  uint32_t handover_ticks_total;/* 接管总拍数(>0，防止 0 拍接管) */
} MotorApp_Startup_t;

/* JustFloat 波形帧：7 个小端 float + 帧尾 0x7F800000(+Inf 位模式)共 32 字节，
 * 一次 DMA 传输；帧须 4 字节对齐，改内容前必须确认上一帧已发完(TC=1)，
 * 否则会撕裂正在发送的帧。 */
typedef struct {
  float data[7];
  uint32_t tail;
} MotorApp_JustFloatFrame_t;

/* ======================== 应用层状态变量 ======================== */
/* 文件级静态均有多个函数跨域使用（中断域写 / 主循环与控制台读），
 * 无法收进函数内部；函数内唯一的跨拍保持量是 ISR 的校准计数器。 */

/* ---- 控制核心：控制器实例、启动时序、端电压融合 ---- */

/* 全工程唯一的串级控制器：中断域每拍运行；主循环/CAN 域只写其中 volatile
 * 外部命令字段（&motor_control.id_ref 等已注册为可写命令）。 */
static FOC_Control_t motor_control;

/* 启动时序中间状态：上电全 0 安全（if_direction=0 时 VirtualRpm() 返回 0），
 * 由 StartControlSequence 清零重建、中断域每拍推进。 */
static MotorApp_Startup_t startup = {0};

/* 端电压融合：启动都从静止/低速开始，实测精度优于重构，故初值选实测；
 * 每次启动与回 IDLE 时重置回该值。 */
static MotorApp_VoltageSource_t voltage_source = {
    .measured_selected = MOTOR_APP_VOLTAGE_MEASURED,
    .measured_weight = 1.0f,
};

/* ---- 运行闸门与故障闭锁 ---- */

static volatile uint32_t motor_run_command = 0U;    /* 用户意图：set run / CAN */
static volatile uint32_t motor_run_requested = 0U;  /* ISR 闸门：故障时强制 0 */
static volatile uint32_t motor_fault_latched = 0U;  /* 故障闭锁：须显式 run 0 解除 */
static volatile uint32_t motor_start_fail_reason = MOTOR_APP_FAIL_NONE; /* 首个故障码 */
static volatile uint8_t motor_idle_reset_done = 0U; /* IDLE 重量级复位去重 */
static volatile uint8_t motor_calibration_just_started = 0U;

/* ---- 采样与实时诊断（中断域写，主循环只读） ---- */

static volatile uint32_t motor_adc_irq_count = 0U;        /* 注入中断拍计数 */
static volatile uint32_t motor_adc_cfg_ok = 0U;           /* 注入序列自检结果 */
static volatile uint32_t motor_adc_check_enable = 1U;     /* 自检门控：0 现场绕过 */
static volatile float motor_control_angle_rad = 0.0f;     /* 本拍控制角(rad) */
static volatile uint32_t motor_ready_fail_mask = 0U;      /* ObserverReady 失败位 */
static volatile uint32_t motor_ready_fail_history = 0U;   /* 本次验证窗口失败位历史 */
static volatile uint32_t motor_callback_cycles_last = 0U; /* 回调耗时(CPU 周期) */
static volatile uint32_t motor_callback_cycles_max = 0U;  /* 回调耗时最大值 */
static volatile uint32_t motor_dwt_hclk_hz = 0U;          /* DWT 周期换算频率 */

/* ---- 波形输出（与文本串口共用 USART1 + DMA） ---- */

static MotorApp_JustFloatFrame_t just_float_frame __attribute__((aligned(4))); /* DMA 帧缓冲 */
static volatile uint32_t just_float_enabled = 0U;     /* 波形总开关，上电默认关 */
static volatile uint32_t motor_wave_mode = MOTOR_APP_WAVE_DEFAULT; /* 通道选择 */
static volatile uint8_t motor_console_tx_active = 0U; /* 文本发送占用 USART1 标志 */

/* ---- 调试选项 ---- */

static volatile uint32_t motor_if_trim_enable = 0U; /* I/F 轻载削减开关，默认关 */

/* ---- 诊断快照：故障/接管/闭环各一份，startdiag 回放 ---- */

/* 一次"故障/接管/闭环"时刻的整拍快照：故障瞬间实时量已不可复现，必须
 * 当拍固化。发布协议：清 valid -> 写载荷 -> __DMB -> 置 valid；读者在
 * 临界区整体拷贝后再判断 valid。 */
typedef struct {
  uint32_t valid;         /* 1=已捕获且可用，读前必须先看它 */
  uint32_t tick;          /* 捕获时刻拍序号 */
  uint32_t state;         /* 捕获时刻状态编号（EnterFault 后可能是已切换值） */
  uint32_t ready_fail;    /* 捕获时刻判据失败位 */
  MotorApp_Fail_t reason; /* 捕获原因/故障码，正常节点为 NONE */
  float direction;        /* I/F 拖动方向 ±1 */
  float virtual_rpm;      /* 虚拟（指令）转速幅值，rpm，恒 >= 0 */
  float command_rpm;      /* 故障/接管瞬间用户目标，rpm；用于识别命令瞬态 */
  float observed_rpm;     /* 观测器 PLL 转速，rpm，带符号 */
  float observed_rpm_f;   /* 观测器两级低通转速，rpm，带符号 */
  float reference_rpm;    /* 生效速度参考 speed_ref_active_rpm，rpm */
  float delta;            /* wrap(控制角-观测角)，坐标系错位量，rad */
  float id_ref;           /* 本拍 d 轴电流参考，A */
  float iq_ref;           /* 本拍生效 q 轴参考 iq_ref_active，A */
  float id;               /* 实测 d 轴电流，A */
  float iq;               /* 实测 q 轴电流，A */
  float vbus;             /* 母线电压，V */
  uint32_t segment;
  float control_rpm;
  float accel_rpm_s;
  float if_accel_rad_s2;
  float speed_integral;
  uint8_t measured_ramp;
} MotorApp_Diagnostic_t;

static MotorApp_Diagnostic_t motor_fault_snapshot = {0};    /* 首次 EnterFault */
static MotorApp_Diagnostic_t motor_handover_snapshot = {0}; /* 进入接管 */
static MotorApp_Diagnostic_t motor_closed_snapshot = {0};   /* 进入闭环 */
static MotorApp_Diagnostic_t motor_low_enter_snapshot = {0}; /* 最近一次运行 I/F 入口 */
static void MotorApp_CaptureDiagnostic(MotorApp_Diagnostic_t *dst,
                                       MotorApp_Fail_t reason);

/* I/F 虚拟电角速度换算机械转速 = speed*dir*60/(2*pi*pp)，已含方向与极对数
 * 换算，正反转不需要单独分支；极对数为 0 时返回 0 防除零（纯防御）。 */
static float MotorApp_VirtualRpm(void) {
  if (foc.observer.motor.pole_pairs == 0U) return 0.0f;

  return startup.if_speed_rad_s * startup.if_direction * 60.0f /
      (CORDIC_TWO_PI_F * (float)foc.observer.motor.pole_pairs);
}

/* 机械 rpm -> 虚拟电角速度。命令目标只在 I/F 阶段使用，闭环目标仍由
 * speed_command_rpm 直接交给速度环。 */
static float MotorApp_RpmToElectricalRad(float rpm) {
  if (foc.observer.motor.pole_pairs == 0U) return 0.0f;
  return fabsf(rpm) * CORDIC_TWO_PI_F *
      (float)foc.observer.motor.pole_pairs / 60.0f;
}

/* 本段目标只限到接管点，绝不限制接管后的用户最终目标。重置仅发生在
 * 目标改变/进入新 I/F 段时；反向制动状态不再被普通目标刷新覆盖。 */
static void MotorApp_SetOpenLoopTarget(float command) {
  float target_rpm = fabsf(command);
  if (target_rpm > MOTOR_APP_HANDOFF_TARGET_RPM)
    target_rpm = MOTOR_APP_HANDOFF_TARGET_RPM;
  startup.reversal_target_direction = (command < 0.0f) ? -1.0f : 1.0f;
  startup.if_speed_target_rad_s = MotorApp_RpmToElectricalRad(target_rpm);
  startup.reversal_braking =
      (MotorApp_VirtualRpm() * startup.reversal_target_direction < 0.0f);
  startup.forced_open_loop = startup.reversal_braking ||
      (fabsf(command) <= MOTOR_APP_FORCED_OPEN_LOOP_MAX_RPM);
  startup.if_verify_active = 0U;
  startup.if_hold_count = 0U;
  startup.ready_count = 0U;
  startup.ready_max_count = 0U;
  startup.ready_sustain_count = 0U;
  startup.ready_span_rad = 0.0f;
  startup.observer_ready = 0U;
  startup.total_count = 0U;
  motor_ready_fail_mask = 0U;
  motor_ready_fail_history = 0U;
}

/* 有符号斜坡让一次减速穿过零点后继续反向加速，没有速度跳变/零点停顿。
 * max_step==0 与控制器 speed_slew==0 一致，表示保持当前参考。 */
static float MotorApp_Slew(float current, float target, float max_step) {
  float delta = target - current;
  if (delta > max_step) delta = max_step;
  if (delta < -max_step) delta = -max_step;
  return current + delta;
}

/* 不对盲区速度求导，也不使用虚拟轨迹作为“实测”加速度。进入 I/F 后
 * 冻结最后可信估计；重新闭环时从新的速度窗口开始，避免跨状态差分。 */
static void MotorApp_UpdateMeasuredAcceleration(void) {
  float speed = foc.observer.state.speed_rpm_f;
  if ((foc_motor_state != FOC_MOTOR_CLOSED_LOOP) ||
      !isfinite(speed) ||
      (fabsf(speed) <= MOTOR_APP_FORCED_OPEN_LOOP_MAX_RPM) ||
      (fabsf(foc.observer.state.speed_rpm) <= MOTOR_APP_FORCED_OPEN_LOOP_MAX_RPM))
    return;
  if (startup.accel_initialized == 0U) {
    startup.accel_last_rpm = speed;
    startup.accel_count = 0U;
    startup.accel_samples = 0U;
    startup.measured_accel_rpm_s = 0.0f;
    startup.accel_initialized = 1U;
    return;
  }
  if (++startup.accel_count >= MOTOR_APP_MS_TO_TICKS(MOTOR_APP_ACCEL_SAMPLE_MS)) {
    float dt = (float)startup.accel_count * foc.timer.Ts;
    float acceleration = (speed - startup.accel_last_rpm) / dt;
    if (startup.accel_samples == 0U)
      startup.measured_accel_rpm_s = acceleration;
    else
      startup.measured_accel_rpm_s += dt / (MOTOR_APP_ACCEL_FILTER_S + dt) *
          (acceleration - startup.measured_accel_rpm_s);
    startup.accel_last_rpm = speed;
    startup.accel_count = 0U;
    if (startup.accel_samples < MOTOR_APP_ACCEL_VALID_SAMPLES)
      startup.accel_samples++;
  }
}

/* 仅在实际速度进入低速区后调用。进入点沿用当前控制角、电流参考；
 * 虚拟轨迹继承实际减速度及对应转矩，不能把配置上限当成实际减速度。 */
static void MotorApp_BeginForcedOpenLoop(float command) {
  float speed = foc.observer.state.speed_rpm_f;
  float target = copysignf(fminf(fabsf(command), MOTOR_APP_HANDOFF_TARGET_RPM), command);
  float motion = target - speed;
  float requested_rate = motor_control.speed_slew_rpm_per_s;
  startup.if_angle = foc.observer.state.phase_raw;
  if (foc_motor_state == FOC_MOTOR_OBSERVER_HANDOVER)
    startup.if_angle = FOC_WrapToPiFast(startup.if_angle +
                                      startup.handover_last_offset);
  startup.if_direction = (speed < 0.0f) ? -1.0f : 1.0f;
  startup.if_speed_rad_s = MotorApp_RpmToElectricalRad(speed);
  /* 缺少有效减速窗口时使用已验证的冷启动斜率；不从“估计为零”推断停转。 */
  startup.if_ramp_rpm_s = MOTOR_APP_IF_ACCEL_RAD_S2 * 60.0f /
      (CORDIC_TWO_PI_F * (float)foc.observer.motor.pole_pairs);
  startup.if_measured_ramp =
      (startup.accel_samples >= MOTOR_APP_ACCEL_VALID_SAMPLES) &&
      isfinite(startup.measured_accel_rpm_s) &&
      (fabsf(startup.measured_accel_rpm_s) > 1.0f) &&
      (startup.measured_accel_rpm_s * motion > 0.0f) &&
      (motor_control.iq_ref_active * motion > 0.0f);
  startup.if_iq_magnitude = MOTOR_APP_IF_IQ_A;
  /* 先承接当前正在产生减速转矩的幅值；即使加速度样本尚未齐全，也不能
   * 在切入 I/F 的边界把负向制动电流突然缩成 1.5 A。 */
  if ((motor_control.iq_ref_active * motion > 0.0f) &&
      (fabsf(motor_control.iq_ref_active) > startup.if_iq_magnitude))
    startup.if_iq_magnitude = fabsf(motor_control.iq_ref_active);
  if (startup.if_measured_ramp != 0U) {
    startup.if_ramp_rpm_s = fabsf(startup.measured_accel_rpm_s);
  }
  if (!isfinite(requested_rate) || (requested_rate < 0.0f)) requested_rate = 0.0f;
  /* 已经从闭环实测到的减速度就是低速轨迹的继承量，不再被旧的默认
   * speed_slew 二次削弱；只有没有实测窗口时，冷启动回退斜率才受命令斜率
   * 限制。这样 +2000 -> -8000 穿过低速盲区时不会突然改成另一套速度率。 */
  startup.if_accel_rad_s2 = MotorApp_RpmToElectricalRad(
      (startup.if_measured_ramp != 0U) ? startup.if_ramp_rpm_s :
      fminf(startup.if_ramp_rpm_s, requested_rate));
  startup.if_iq_reference = motor_control.iq_ref_active;
  startup.if_id_reference = motor_control.reference.id_ref;
  startup.runtime_if = 1U;
  startup.closed_loop_braking = 0U;
  startup.speed_startup_active = 0U;
  MotorApp_SetOpenLoopTarget(command);
  startup.segment++;
  motor_handover_snapshot.valid = 0U;
  motor_closed_snapshot.valid = 0U;
  MotorApp_CaptureDiagnostic(&motor_low_enter_snapshot, MOTOR_APP_FAIL_NONE);
  startup.accel_initialized = 0U;
  foc_motor_state = FOC_MOTOR_OPEN_LOOP_IF;
  /* 不改 iq_ref_active，不复位电流 PI；1->0 模式边沿自动承接上拍转矩。 */
}

/* CORDIC 求任意角 sin/cos：先归一化到 [-pi,pi) 再转 Q1.31；纯硬件固定
 * 周期（不用 sinf/cosf，避免浮点库的调用开销与不确定执行时间）。 */
static void MotorApp_SinCos(float angle, FOC_SIN_COS_t *sc) {
  int32_t q31 = CORDIC_RadToQ31_WrappedFast(FOC_WrapToPiFast(angle));
  CORDIC_SinCos_FastF32(q31, &sc->sin, &sc->cos);
}

/* 把当拍运行量整体拷进诊断快照（只在 25 kHz 中断域调用，无需临界区）。
 * 纯只读采样，不改变控制量。 */
static void MotorApp_CaptureDiagnostic(MotorApp_Diagnostic_t *dst,
                                       MotorApp_Fail_t reason) {
  dst->valid = 0U;
  dst->tick = motor_adc_irq_count;
  dst->state = (uint32_t)foc_motor_state;
  dst->ready_fail = motor_ready_fail_mask | motor_ready_fail_history;
  dst->reason = reason;
  dst->direction = startup.if_direction;

  /* 极对数为 0 时不做换算（防除零），直接记 0 rpm。 */
  dst->virtual_rpm = (foc.observer.motor.pole_pairs != 0U)
      ? MotorApp_VirtualRpm() : 0.0f;
  dst->command_rpm = motor_control.speed_command_rpm;
  dst->observed_rpm = foc.observer.state.speed_rpm;
  dst->observed_rpm_f = foc.observer.state.speed_rpm_f;
  dst->reference_rpm = motor_control.speed_ref_active_rpm;

  /* delta = 本拍控制角 - 观测角，归一化到 [-pi,pi)，正负表示领先/滞后。 */
  dst->delta = FOC_WrapToPiFast(motor_control_angle_rad -
                              foc.observer.state.phase_raw);
  dst->id_ref = motor_control.reference.id_ref;
  dst->iq_ref = motor_control.iq_ref_active;
  dst->id = foc.state.i_dq.d;
  dst->iq = foc.state.i_dq.q;
  dst->vbus = foc.state.vbus;
  dst->segment = startup.segment;
  dst->control_rpm = foc.observer.state.speed_rpm_f;
  dst->accel_rpm_s = startup.measured_accel_rpm_s;
  dst->if_accel_rad_s2 = startup.if_accel_rad_s2;
  dst->speed_integral = motor_control.speed_pi.integral;
  dst->measured_ramp = startup.if_measured_ramp;

  __DMB();
  dst->valid = 1U;
}

/* 进入故障停机：第一件事清 MOE 立即撤销桥输出（必须在任何后续 PWM 写
 * 之前，否则会多输出一拍错误电压）；只在未闭锁时记录原因与快照（保留
 * 首个故障便于定位根因）；之后闭锁、清 ISR 闸门、回 IDLE、触发完整复位。
 * 解除闭锁唯一途径：显式 run 0。 */
static void MotorApp_EnterFault(MotorApp_Fail_t reason) {
  CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);

  if (motor_fault_latched == 0U) {
    MotorApp_CaptureDiagnostic(&motor_fault_snapshot, reason);
    motor_start_fail_reason = reason;
  }

  motor_fault_latched = 1U;
  motor_run_requested = 0U;
  foc_motor_state = FOC_MOTOR_IDLE;
  motor_idle_reset_done = 0U;
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  TIM1->CCR3 = 0U;
}

/* 把用户意图转成 ISR 运行闸门（主循环与 CAN 中断都会调用，故整段在
 * 临界区内成组更新）。已闭锁：闸门强制 0，仅显式 run 0 才解除；未闭锁：
 * 闸门归一跟随意图。本函数不启动电机，启动由 MotorApp_Process 的门控做。 */
static void MotorApp_ApplyRunCommand(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  if (motor_fault_latched != 0U) {
    motor_run_requested = 0U;
    if (motor_run_command == 0U) motor_fault_latched = 0U;
  } else {
    motor_run_requested = (motor_run_command != 0U) ? 1U : 0U;
  }

  __set_PRIMASK(primask);
}

/* ======================== 控制参考仲裁与电流环 ======================== */

/* "主动控制状态"的唯一判定点（白名单：未列出的状态默认走关桥路径，更
 * 安全）：ALIGN / OPEN_LOOP_IF / OBSERVER_HANDOVER / CLOSED_LOOP 需要推进
 * 观测器/状态机并写 PWM。新增状态只需扩展本函数，不必改中断主结构。 */
static uint8_t MotorApp_IsControlState(FOC_Motor_State_t state) {
  return ((state == FOC_MOTOR_CLOSED_LOOP) ||
          (state == FOC_MOTOR_ALIGN) ||
          (state == FOC_MOTOR_OPEN_LOOP_IF) ||
          (state == FOC_MOTOR_OBSERVER_HANDOVER)) ? 1U : 0U;
}

/* 每拍提交一次控制参考（按状态决定控制角与 dq 电流）：
 *   ALIGN        固定角 + Id、Iq=0（只建磁场不产转矩）
 *   OPEN_LOOP_IF 虚拟角 + Iq=dir*幅值、Id=0
 *   HANDOVER     观测角+渐消偏置，PI 积分器随坐标系旋转搬运（见内注）
 *   CLOSED_LOOP  观测角 + 命令/速度环，启动过渡期加淡入与斜坡限幅
 * 必须在 AdvanceStateMachine 之后、RunCurrentLoop 之前调用。 */
static void MotorApp_ResolveControlReference(void) {
  switch (foc_motor_state) {
  case FOC_MOTOR_ALIGN:
    /* 固定角 0 rad（alpha 轴）定位：speed_loop_enable=0 表示直接使用
     * iq_ref=0，速度环不参与。 */
    FOC_Control_SubmitReference(&motor_control, 0.0f,
                                MOTOR_APP_ALIGN_ID_A, 0.0f, 0U);
    break;

  case FOC_MOTOR_OPEN_LOOP_IF:
    /* 换向前后的 q 目标始终指向新方向，穿零不再改 iq_ref_active。
     * 控制器承接旧参考并按 A/s 过渡；虚拟速度符号与电流符号分开管理。 */
    FOC_Control_SubmitReference(&motor_control, startup.if_angle,
        startup.if_id_reference,
        (startup.runtime_if != 0U) ? startup.if_iq_reference :
        startup.reversal_target_direction * startup.if_iq_magnitude, 0U);
    if (startup.runtime_if != 0U)
      motor_control.reference.iq_slew_limit = MOTOR_APP_RUN_IQ_SLEW_A_S;
    break;

  case FOC_MOTOR_OBSERVER_HANDOVER: {
    FOC_SIN_COS_t sc;
    /* 渐变进度 0 -> 1，夹到 1 防越界（防御性限幅）。 */
    float x = (float)startup.handover_ticks /
              (float)startup.handover_ticks_total;
    if (x > 1.0f) x = 1.0f;

    /* blend = 1 - 三次 smoothstep：x=0 保留全部初始偏置，x=1 偏置卸完；
     * 端点斜率为 0，切换瞬间无突变。d 轴分量随之渐消。 */
    float blend = 1.0f - x * x * (3.0f - 2.0f * x);
    float offset = startup.handover_offset_rad * blend;
    float d_obs = startup.handover_id_obs * blend;
    /* 速度模式下 I/F 的恒定 q 转矩不能贯穿整个角度接管：上一拍日志显示
     * 1000 rpm 接管到闭环首拍已冲到 2600 rpm。让 q 转矩按 blend^2 平滑收敛
     * 到零，闭环速度环从低转矩状态接手；电流模式仍保持原 q 参考。 */
    float q_scale = (motor_control.speed_loop_enable != 0U)
        ? (blend * blend) : 1.0f;
    float q_obs = startup.handover_iq_obs * q_scale;

    /* PI 积分状态从上一拍坐标系旋转到本拍坐标系（角度=两帧偏置差）：
     * 必须在新旧坐标系间搬运积分器，否则积分项按错误角度继续累积，
     * 表现为接管期间的电流冲击。 */
    MotorApp_SinCos(startup.handover_last_offset - offset, &sc);
    FOC_Control_RotateCurrentIntegrals(&motor_control, sc.sin, sc.cos);
    startup.handover_last_offset = offset;

    /* 观测系 (d,q) 旋转回控制系：theta_ctrl = phase_raw + offset，
     * id/iq 为 R(-offset) 旋转的两个分量。 */
    MotorApp_SinCos(offset, &sc);
    FOC_Control_SubmitReference(&motor_control,
        foc.observer.state.phase_raw + offset,
        d_obs * sc.cos + q_obs * sc.sin,
       -d_obs * sc.sin + q_obs * sc.cos, 0U);
    break;
  }

  case FOC_MOTOR_CLOSED_LOOP: {
    float id_reference = motor_control.id_ref;

    /* 用户命令 -> 控制器内部斜坡参考（生成 speed_ref_active_rpm 给速度 PI）。 */
    /* 反转时闭环只负责刹到低速；远端 -2000/-8000 目标不参与这一段。
     * 内部目标 0 不改用户命令，因此不会触发 set speed 0 的停机语义。 */
    motor_control.speed_ref_rpm =
        (motor_control.speed_command_rpm * startup.if_direction < 0.0f)
        ? 0.0f : motor_control.speed_command_rpm;

    /* 启动过渡：Id 参考按 settle 进度线性淡入，避免接管刚完成时 Id 阶跃
     * 通过电流环产生电压阶跃。 */
    if (startup.speed_startup_active != 0U) {
      float f = (float)startup.speed_settle_count /
          (float)MOTOR_APP_MS_TO_TICKS(MOTOR_APP_SPEED_SETTLE_MS);
      if (f > 1.0f) f = 1.0f;
      id_reference *= f;
    }

    /* speed_loop_enable 由用户配置：0=电流模式（直接用 iq_ref），
     * 非 0=速度模式；接管完成前速度环被控制器内部抑制。 */
    FOC_Control_SubmitReference(&motor_control,
        foc.observer.state.phase_raw, id_reference, motor_control.iq_ref,
        (uint8_t)((motor_control.speed_loop_enable != 0U) ||
                  (startup.closed_loop_braking != 0U)));

    motor_control.reference.iq_slew_limit = MOTOR_APP_RUN_IQ_SLEW_A_S;
    /* 只在首次接管的 50 ms 内保持参考；换向请求立即允许闭环制动。 */
    if ((startup.speed_startup_active != 0U) &&
        (startup.closed_loop_braking == 0U)) {
      motor_control.reference.speed_slew_limit = 0.0f;
      motor_control.reference.iq_slew_limit = MOTOR_APP_START_IQ_SLEW_A_S;
    }
    break;
  }

  default:
    /* 安全状态不提交参考（调用前已用 IsControlState 过滤，不应到达）。 */
    break;
  }
}

/* 控制速度与显示速度分离；I/F 时连速度输入都来自虚拟状态，辅助观测
 * 输出即使失效也不能把 NaN 带进控制器。控制反馈直接用观测器 50 Hz
 * 双极点滤波值 speed_rpm_f（与 Ready 一致性门、上报同源），应用层不再
 * 叠加单极点滤波。 */
static float MotorApp_ControlSpeedFeedback(void) {
  if ((foc_motor_state == FOC_MOTOR_CLOSED_LOOP) ||
      (foc_motor_state == FOC_MOTOR_OBSERVER_HANDOVER)) {
    return foc.observer.state.speed_rpm_f;
  }
  return MotorApp_VirtualRpm();
}

/* 公共电流环执行体：Park -> 电流PI（含分频速度环）-> 逆Park -> 逆Clarke
 * -> SVPWM。所有状态共用这一条路径，差别只在提交的参考上。
 * 错误不返回码而是直接 EnterFault 关桥（u_dq 非有限 -> NONFINITE；
 * SVPWM 定标失败 -> BUS_INVALID），因此返回后调用者必须重查状态。 */
static void MotorApp_RunCurrentLoop(FOC_Control_t *control) {
  float theta_ctrl;
  uint32_t theta_q31;

  /* 单步归一化到 [-pi,pi) 后转 Q1.31（对运行范围内的角度足够）；
   * motor_control_angle_rad 供诊断比对观测角与虚拟角。 */
  theta_ctrl = FOC_WrapToPiFast(control->reference.theta_ctrl);
  theta_q31 = (uint32_t)CORDIC_RadToQ31_WrappedFast(theta_ctrl);
  motor_control_angle_rad = theta_ctrl;

  /* CORDIC 硬件求 sin/cos（先读 cos 再读 sin 由封装保证顺序）。 */
  CORDIC_SinCos_FastF32((int32_t)theta_q31, &foc_sin_cos.sin,
                        &foc_sin_cos.cos);

  /* Park：静止系 alpha-beta -> 旋转系 dq。 */
  FOC_Park(&foc.state.i_alpha_beta, &foc_sin_cos, &foc.state.i_dq);

  /* 速度环反馈用观测器 50 Hz 双极点滤波值（与显示/Ready 同源，50 Hz
   * 双极点直流群延迟约 10.6 ms）。强制 I/F 时只送虚拟速度，不依赖低速
   * 观测器。控制器经 control->speed_feedback 读取，不再走形参。 */
  control->speed_feedback = MotorApp_ControlSpeedFeedback();
  FOC_Control_Run(control, foc.state.i_dq.d, foc.state.i_dq.q,
                  foc.state.vbus,
                  &foc.state.u_dq.d, &foc.state.u_dq.q);

  /* NaN/Inf 一旦进 SVPWM 会算出垃圾比较值，必须在写寄存器前拦下。 */
  if (!isfinite(foc.state.u_dq.d) || !isfinite(foc.state.u_dq.q)) {
    MotorApp_EnterFault(MOTOR_APP_FAIL_NONFINITE);
    return;
  }

  /* 逆 Park + 逆 Clarke：dq 电压 -> alpha-beta -> 三相电压。 */
  FOC_InvPark(&foc.state.u_dq, &foc_sin_cos, &foc.state.u_alpha_beta);
  FOC_InvClarke(&foc.state.u_alpha_beta, &foc.state.u_abc);

  /* SVPWM：三相电压 -> 比较值与占空比；失败说明母线过小/非法。 */
  if (FOC_SVPWM_Run(&foc.state.u_abc, foc.state.vbus, &foc.timer,
                    &foc.svpwm) != HAL_OK) {
    MotorApp_EnterFault(MOTOR_APP_FAIL_BUS_INVALID);
  }
}

/* ======================== 启动时序状态机 ======================== */

/* ObserverReady 综合判据：决定能否从 I/F 切到闭环。按位或累积失败位，
 * 任一位失败即整体重新计时；全部通过则累加连续拍数与净角度覆盖，二者
 * 同时达标（连续 750 拍 + 整一圈电角度）才置 observer_ready。
 * 设计意图："连续 N 拍 + 整圈覆盖"排除偶然对齐；"窗口内相对漂移"容忍
 * I/F 固有负载角；电流跟踪用独立计数器去抖且把 PI 饱和视为失败。 */
static void MotorApp_UpdateObserverReady(void) {
  float speed = foc.observer.state.speed_rpm;
  /* Ready 的连续性判据使用 PLL 积分角，而不是每拍 atan2 得到的原始磁链角。
   * phase_raw 会把 PWM/端电压采样噪声直接带入单拍回退和整圈累计；在
   * 1000 rpm 时每拍真实前进约 0.029 rad，0.01 rad 的回退容差很容易被
   * 采样抖动击穿。PLL 角由同一个观测速度积分得到，仍保留速度跳变、方向、
   * 磁链幅值和电流跟踪门槛；phase_raw 继续留给最终坐标接管。 */
  float phase = foc.observer.state.pll_phase;
  float psi = foc.observer.state.psi_mag;
  /* 单拍角增量，归一化后 [-pi,pi)，避免边界处出现 2*pi 假跳变。 */
  float angle_step = FOC_WrapToPiFast(phase - startup.ready_last_phase_rad);
  float speed_step = speed - startup.ready_last_speed_rpm;
  /* 负载角：虚拟角超前观测角为正。 */
  float delta = FOC_WrapToPiFast(startup.if_angle - phase);
  /* 观测转速与指令转速的偏差；VirtualRpm 已含方向。 */
  float speed_match = foc.observer.state.speed_rpm_f;
  float speed_error = speed_match - MotorApp_VirtualRpm();
  uint32_t fail = 0U;

  startup.ready_speed_error = speed_error;

  /* 非有限值优先拦下：NaN 参与比较全为假，会让后续所有门控失效。 */
  if (!isfinite(speed) || !isfinite(speed_match) || !isfinite(phase) ||
      !isfinite(delta) || !isfinite(speed_error)) {
    fail |= MOTOR_APP_READY_FAIL_NONFINITE;
  }

  /* 磁链幅值须在可信区间；psi_max<=0 表示不查上限。 */
  if ((psi < foc.observer.config.psi_min) ||
      ((foc.observer.config.psi_max > 0.0f) &&
       (psi > foc.observer.config.psi_max))) fail |= MOTOR_APP_READY_FAIL_PSI;

  /* 观测器未初始化：内部磁链/PLL 状态无意义。 */
  if (foc.observer.state.initialized == 0U) fail |= MOTOR_APP_READY_FAIL_INIT;

  /* 反电动势不足：转速绝对值过低则角度估计不可信。 */
  if (fabsf(speed) < MOTOR_APP_OBSERVER_READY_MIN_RPM)
    fail |= MOTOR_APP_READY_FAIL_SPEED;

  /* 方向：转速符号须与拖动方向一致，且单拍角增量不得为反向（-0.01 rad
   * 容差允许噪声级回退；乘 if_direction 的写法让正反转共用同一段代码）。 */
  if ((speed * startup.if_direction <= 0.0f) ||
      (angle_step * startup.if_direction < -0.01f))
    fail |= MOTOR_APP_READY_FAIL_DIR;

  /* 单拍跳变：正常每拍最多约 0.032 rad，超 0.35 说明 atan2 跨 ±pi 或异常。 */
  if (fabsf(angle_step) > MOTOR_APP_OBSERVER_READY_MAX_ANGLE_STEP)
    fail |= MOTOR_APP_READY_FAIL_ANGLE_STEP;

  if (fabsf(speed_step) > MOTOR_APP_OBSERVER_READY_MAX_SPEED_STEP)
    fail |= MOTOR_APP_READY_FAIL_SPEED_STEP;

  /* 绝对负载角判据（默认编译期关闭，原因见宏注释）。 */
  if ((MOTOR_APP_OBSERVER_READY_ANGLE_CHECK_ENABLE != 0U) &&
      (fabsf(delta) > MOTOR_APP_OBSERVER_READY_MAX_LOAD_ANGLE))
    fail |= MOTOR_APP_READY_FAIL_SYNC;

  /* 速度匹配：用滤波速度抑制 PLL 原始速度的单拍尖峰；原始 speed 仍受上面的
   * 最低转速、方向和 speed_step 门控，持续失步会在滤波值上体现出来。 */
  if (fabsf(speed_error) > MOTOR_APP_OBSERVER_READY_SPEED_ERROR_RPM)
    fail |= MOTOR_APP_READY_FAIL_SPEED_MATCH;

  /* 未进验证窗口（速度还没爬到位）不允许判就绪：刚起步低速段可能碰巧
   * 通过其它判据。 */
  if (startup.if_verify_active == 0U) fail |= MOTOR_APP_READY_FAIL_RAMP;

  /* 漂移：窗口第一拍（ready_count==0）记锚点，其后查相对锚点的变化量
   * （容忍固定负载角，只查窗口内慢漂）。 */
  if (startup.ready_count == 0U) startup.ready_anchor_delta = delta;
  if (fabsf(FOC_WrapToPiFast(delta - startup.ready_anchor_delta)) >
      MOTOR_APP_OBSERVER_READY_DRIFT_RAD) fail |= MOTOR_APP_READY_FAIL_DRIFT;

  /* 电流跟踪：|Id| 过大=角度错位；Iq 偏差过大=环跟不上或饱和；
   * 任一 PI 饱和=电压已用尽，角度信息同样不可信。 */
  uint8_t bad_tracking =
      (fabsf(foc.state.i_dq.d) > MOTOR_APP_OBSERVER_READY_MAX_ID_A) ||
      (fabsf(foc.state.i_dq.q - motor_control.iq_ref_active) >
       MOTOR_APP_OBSERVER_READY_MAX_IQ_ERROR_A) ||
      (motor_control.id_pi.saturation != PI_SATURATION_NONE) ||
      (motor_control.iq_pi.saturation != PI_SATURATION_NONE);

  /* 去抖：连续坏 50 拍(2ms)才置 FAIL_TRACK，有一拍好即清零。 */
  if (bad_tracking != 0U) {
    if (startup.ready_sustain_count <
        MOTOR_APP_MS_TO_TICKS(MOTOR_APP_OBSERVER_READY_SUSTAIN_MS))
      startup.ready_sustain_count++;
  } else startup.ready_sustain_count = 0U;

  if (startup.ready_sustain_count >=
      MOTOR_APP_MS_TO_TICKS(MOTOR_APP_OBSERVER_READY_SUSTAIN_MS))
    fail |= MOTOR_APP_READY_FAIL_TRACK;

  /* 可选 I/F 电流削减：仅在"稳定支路"缓慢下调。stable_delta = -dir*delta
   * 把负载角换算成恒正的稳定滞后量。五个条件必须同时满足：
   * a) if_trim 打开  b) 已进验证窗口  c) 已保持 20ms 以上（避开瞬态）
   * d) 除 SYNC/DRIFT 外无失败位（这两位重载下本就会亮，但不许掩盖其它位）
   * e) 稳定负载角在 55~85 度（确实拖得住，有余量减流）。 */
  float stable_delta = -startup.if_direction * delta;
  if ((motor_if_trim_enable != 0U) &&
      (startup.if_verify_active != 0U) &&
      (startup.if_hold_count > MOTOR_APP_MS_TO_TICKS(20U)) &&
      ((fail & ~(MOTOR_APP_READY_FAIL_SYNC | MOTOR_APP_READY_FAIL_DRIFT)) == 0U) &&
      (stable_delta > MOTOR_APP_IF_TRIM_ANGLE_RAD) &&
      (stable_delta < MOTOR_APP_IF_TRIM_MAX_ANGLE_RAD)) {
    startup.if_iq_magnitude -= MOTOR_APP_IF_CURRENT_TRIM_A_PER_S * foc.timer.Ts;
    if (startup.if_iq_magnitude < MOTOR_APP_IF_CURRENT_FLOOR_A)
      startup.if_iq_magnitude = MOTOR_APP_IF_CURRENT_FLOOR_A;
  }

  /* 只在有界验证窗口内保存失败历史；加速阶段的 READY_FAIL_RAMP 不混入历史，
   * 这样故障快照能回答"窗口里哪一条反复失败"，同时保留 motor_ready_fail_mask
   * 作为最后一拍的即时位图。 */
  if (startup.if_verify_active != 0U) motor_ready_fail_history |= fail;

  /* 发布本拍结论：全局量供跨域观察，startup 内副本供本文件/快照使用。 */
  motor_ready_fail_mask = fail;
  startup.ready_last_phase_rad = phase;
  startup.ready_last_speed_rpm = speed;

  if (fail != 0U) {
    /* 任一判据失败：整体重新计时（连续计数与角度覆盖都归零）。 */
    startup.ready_count = 0U;
    startup.ready_span_rad = 0.0f;
    startup.observer_ready = 0U;
    return;
  }

  /* 全部通过：累加连续拍数（封顶 750）与带符号净覆盖（夹 >=0，
   * 来回抖动正负抵消，只有真正单向转过的角度才计入）。 */
  if (startup.ready_count < MOTOR_APP_MS_TO_TICKS(MOTOR_APP_OBSERVER_READY_TIME_MS))
    startup.ready_count++;
  if (startup.ready_count > startup.ready_max_count)
    startup.ready_max_count = startup.ready_count;
  startup.ready_span_rad += startup.if_direction * angle_step;
  if (startup.ready_span_rad < 0.0f) startup.ready_span_rad = 0.0f;

  /* 最终结论 = 持续够久 且 净覆盖整一圈。 */
  startup.observer_ready =
      (startup.ready_count >= MOTOR_APP_MS_TO_TICKS(MOTOR_APP_OBSERVER_READY_TIME_MS)) &&
      (startup.ready_span_rad >= MOTOR_APP_OBSERVER_READY_SPAN_RAD);
}

/* 接管准备（observer_ready 置位的那一拍调用一次，不是每拍）：
 * 1) 偏置 = wrap(控制角-观测角)；把"最后施加的 dq 参考"（而非新采样，
 *    避免噪声；ready 已确认实测电流在跟踪它）旋转到观测坐标系——观测系
 *    q 轴电流必须与拖动方向同号，反号说明观测器锁到反方向（角度差接近
 *    180 度），接管会反转转矩，必须放弃报故障；
 * 2) 渐变总拍数由偏置角速率上限反算（smoothstep 峰值斜率 1.5），
 *    夹到 20~150 ms；+1 防止 0 拍接管（后面要做除数）。 */
static void MotorApp_BeginObserverHandover(void) {
  FOC_SIN_COS_t sc;
  /* 初始偏置 = 当前控制角 - 观测角，归一化到 [-pi,pi)。 */
  float offset = FOC_WrapToPiFast(motor_control_angle_rad -
                                  foc.observer.state.phase_raw);
  MotorApp_SinCos(offset, &sc);

  /* 最后施加的参考旋转到观测坐标系：与 ResolveControlReference 里的逆
   * 旋转严格互逆，保证接管前后 alpha-beta 参考连续、起点无扰。 */
  float d = motor_control.reference.id_ref;
  float q = motor_control.iq_ref_active;
  startup.handover_id_obs = d * sc.cos - q * sc.sin;
  startup.handover_iq_obs = d * sc.sin + q * sc.cos;

  /* 安全门：非有限，或观测系 q 轴转矩与拖动方向反号 -> 放弃接管。 */
  if (!isfinite(startup.handover_id_obs) ||
      !isfinite(startup.handover_iq_obs) ||
      (startup.handover_iq_obs * startup.if_direction <= 0.0f)) {
    MotorApp_EnterFault(MOTOR_APP_FAIL_HANDOVER_CURRENT);
    return;
  }

  uint32_t ticks = (uint32_t)(1.5f * fabsf(offset) /
      (MOTOR_APP_HANDOVER_RATE_RAD_S * foc.timer.Ts)) + 1U;
  /* 下限 500 拍(20ms)：积分器搬运需要足够拍数；上限 3750 拍(150ms)：
   * 混合坐标系拖太久会累积误差。 */
  if (ticks < MOTOR_APP_MS_TO_TICKS(MOTOR_APP_HANDOVER_MIN_TIME_MS))
    ticks = MOTOR_APP_MS_TO_TICKS(MOTOR_APP_HANDOVER_MIN_TIME_MS);
  if (ticks > MOTOR_APP_MS_TO_TICKS(MOTOR_APP_HANDOVER_MAX_TIME_MS))
    ticks = MOTOR_APP_MS_TO_TICKS(MOTOR_APP_HANDOVER_MAX_TIME_MS);

  startup.handover_offset_rad = offset;
  startup.handover_last_offset = offset; /* 本拍尚未卸掉任何偏置 */
  startup.handover_ticks_total = ticks;
  startup.handover_ticks = 0U;
  startup.fault_count = 0U; /* 不把 I/F 阶段的异常带进接管 */
  MotorApp_CaptureDiagnostic(&motor_handover_snapshot, MOTOR_APP_FAIL_NONE);
}

/* 检查观测器全部对外状态与 PLL 积分（在观测器里复用为频率积分器）：
 * 任一 NaN/Inf 会在 atan2/Park 中扩散，必须在控制计算前拦下。 */
static uint8_t MotorApp_ObserverIsFinite(void) {
  return isfinite(foc.observer.state.phase_raw) &&
      isfinite(foc.observer.state.speed_rpm) &&
      isfinite(foc.observer.state.speed_rpm_f) &&
      isfinite(foc.observer.state.psi_mag) &&
      isfinite(foc.observer.state.x_alpha) &&
      isfinite(foc.observer.state.x_beta) &&
      isfinite(foc.observer.state.pll_phase) &&
      isfinite(foc.observer.pll.integral);
}

/* 辅助观测器数值失效时，按虚拟状态重新播种估计，不触碰强拖角或电流 PI。
 * 重新播种不是 Ready；之后仍必须重新通过完整接管窗口。 */
static void MotorApp_RecoverAuxObserver(void) {
  Observer_MotorParam_t motor = foc.observer.motor;
  Observer_Config_t config = foc.observer.config;
  FOC_SIN_COS_t sc;
  float omega = startup.if_direction * startup.if_speed_rad_s;
  MotorApp_SinCos(startup.if_angle, &sc);
  foc.observer.state = (Observer_State_t){0};
  Observer_Init(&foc.observer, &motor, &config);
  foc.observer.state.psi_alpha = motor.flux_linkage * sc.cos;
  foc.observer.state.psi_beta = motor.flux_linkage * sc.sin;
  foc.observer.state.x_alpha = foc.observer.state.psi_alpha +
      motor.Ls * foc.state.i_alpha_beta.alpha;
  foc.observer.state.x_beta = foc.observer.state.psi_beta +
      motor.Ls * foc.state.i_alpha_beta.beta;
  foc.observer.state.phase_raw = startup.if_angle;
  foc.observer.state.pll_phase = startup.if_angle;
  foc.observer.state.pll_omega_e = omega;
  foc.observer.state.omega_m = omega / (float)motor.pole_pairs;
  foc.observer.state.speed_rpm = MotorApp_VirtualRpm();
  foc.observer.state.speed_rpm_f = foc.observer.state.speed_rpm;
  foc.observer.state.speed_filter.y1 = foc.observer.state.speed_rpm;
  PI_Controller_PreloadOutput(&foc.observer.pll, omega, 0.0f, 0.0f);
  startup.ready_count = 0U;
  startup.ready_span_rad = 0.0f;
  startup.observer_ready = 0U;
}

/* 推进状态内部量（观测器/虚拟角/计数），不产生参考也不改状态编号——
 * 状态切换在控制量写入之后由 UpdateStateTransition 提交。顺序：
 *   命令校验 -> 命令层约束 -> 总超时 -> ALIGN 计数 -> 观测器（前后各查
 *   一次有限性：防污染输入、防积分器发散）-> 分状态推进。
 * 观测器统一在此执行：接管/闭环/ready 判据都需要它，避免重复。
 * 可能经 EnterFault 提前终止（此时状态已被改回 IDLE，调用者必须重查）。 */
static void MotorApp_AdvanceStateMachine(const Observer_Input_t *input) {
  float command = motor_control.speed_command_rpm;

  /* 命令必须是有限值：NaN 参与比较全为假，后面所有门控都会失效。 */
  if (!isfinite(command)) {
    MotorApp_EnterFault(MOTOR_APP_FAIL_NONFINITE);
    return;
  }

  /* 命令归零在任何主动状态都表示立即停机。 */
  if (command == 0.0f) {
    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
    motor_run_command = 0U;
    motor_run_requested = 0U;
    foc_motor_state = FOC_MOTOR_IDLE;
    return;
  }

  if ((foc_motor_state == FOC_MOTOR_ALIGN) ||
      (foc_motor_state == FOC_MOTOR_OPEN_LOOP_IF)) {
    float target = fminf(fabsf(command), MOTOR_APP_HANDOFF_TARGET_RPM);
    float direction = (command < 0.0f) ? -1.0f : 1.0f;
    if (foc_motor_state == FOC_MOTOR_ALIGN)
      startup.if_direction = direction;
    if ((direction != startup.reversal_target_direction) ||
        (fabsf(MotorApp_RpmToElectricalRad(target) -
               startup.if_speed_target_rad_s) > 0.01f))
      MotorApp_SetOpenLoopTarget(command);
  }

  /* 观测器不可用的低速强制开环允许长期运行，不受启动验证总超时影响。 */
  if (((foc_motor_state == FOC_MOTOR_ALIGN) ||
       (foc_motor_state == FOC_MOTOR_OBSERVER_HANDOVER) ||
       ((foc_motor_state == FOC_MOTOR_OPEN_LOOP_IF) &&
        (startup.if_verify_active != 0U))) &&
      (startup.forced_open_loop == 0U)) {
    startup.total_count++;
    if (startup.total_count >=
        MOTOR_APP_MS_TO_TICKS(MOTOR_APP_START_TOTAL_TIME_MS)) {
      MotorApp_EnterFault(MOTOR_APP_FAIL_START_TIMEOUT);
      return;
    }
  }

  if (foc_motor_state == FOC_MOTOR_ALIGN) {
    /* ALIGN 不跑观测器（转子位置未知，结果无意义），只累加定位拍数，
     * 由 UpdateStateTransition 判退出。 */
    startup.align_count++;
    return;
  }

  /* 强制 I/F 时观测器只作辅助估计：若低速反电动势使其暂时失效，控制仍
   * 使用虚拟角/虚拟速度继续运行；离开强制段后再恢复严格有限性检查。 */
  if (!MotorApp_ObserverIsFinite()) {
    if (foc_motor_state != FOC_MOTOR_OPEN_LOOP_IF) {
      MotorApp_EnterFault(MOTOR_APP_FAIL_NONFINITE);
      return;
    }
    MotorApp_RecoverAuxObserver();
  } else {
    Observer_Run(&foc.observer, input);
    if (!MotorApp_ObserverIsFinite()) {
      if (foc_motor_state != FOC_MOTOR_OPEN_LOOP_IF) {
        MotorApp_EnterFault(MOTOR_APP_FAIL_NONFINITE);
        return;
      }
      MotorApp_RecoverAuxObserver();
    }
  }

  /* 目标反向/降至低速时，先继续观测闭环制动。只有实际观测速度和控制
   * 滤波速度都进低速区才交给虚拟角；不能用已先行过零的速度参考代替。 */
  if ((foc_motor_state == FOC_MOTOR_CLOSED_LOOP) ||
      (foc_motor_state == FOC_MOTOR_OBSERVER_HANDOVER)) {
    MotorApp_UpdateMeasuredAcceleration();
    startup.closed_loop_braking =
        (command * startup.if_direction < 0.0f) ||
        (fabsf(command) <= MOTOR_APP_FORCED_OPEN_LOOP_MAX_RPM);
    if ((startup.closed_loop_braking != 0U) &&
        (fabsf(foc.observer.state.speed_rpm) <= MOTOR_APP_FORCED_OPEN_LOOP_MAX_RPM) &&
        (fabsf(foc.observer.state.speed_rpm_f) <= MOTOR_APP_FORCED_OPEN_LOOP_MAX_RPM)) {
      MotorApp_BeginForcedOpenLoop(command);
      return; /* 本拍角度与电流沿用原值，下一拍推进虚拟角。 */
    }
  }

  switch (foc_motor_state) {
  case FOC_MOTOR_OPEN_LOOP_IF: {
    float previous = startup.if_speed_rad_s * startup.if_direction;
    float target = startup.if_speed_target_rad_s * startup.reversal_target_direction;
    float next;
    if (startup.runtime_if != 0U) {
      float requested_rate = motor_control.speed_slew_rpm_per_s;
      if (!isfinite(requested_rate) || (requested_rate < 0.0f)) requested_rate = 0.0f;
      startup.if_accel_rad_s2 = MotorApp_RpmToElectricalRad(
          (startup.if_measured_ramp != 0U) ? startup.if_ramp_rpm_s :
          fminf(startup.if_ramp_rpm_s, requested_rate));
      /* 减速转矩看轨迹变化方向，不看最终速度符号。+790 -> +500 也要
       * 沿用负 Iq；穿零没有特殊翻转，换目标时仍由控制器限制电流变化率。 */
      if (target != previous)
        startup.if_iq_reference = copysignf(startup.if_iq_magnitude, target - previous);
    }
    next = MotorApp_Slew(previous, target, startup.if_accel_rad_s2 * foc.timer.Ts);
    startup.if_speed_rad_s = fabsf(next);
    if (next != 0.0f) startup.if_direction = (next < 0.0f) ? -1.0f : 1.0f;
    startup.reversal_braking = (next * startup.reversal_target_direction < 0.0f);
    startup.forced_open_loop = startup.reversal_braking ||
        (fabsf(command) <= MOTOR_APP_FORCED_OPEN_LOOP_MAX_RPM);
    /* 梯形积分保持穿零前后角连续；零点不改角度、不翻转电流。 */
    startup.if_angle = FOC_WrapToPiFast(startup.if_angle +
        0.5f * (previous + next) * foc.timer.Ts);
    startup.if_id_reference = MotorApp_Slew(startup.if_id_reference, 0.0f,
        MOTOR_APP_RUN_IQ_SLEW_A_S * foc.timer.Ts);
    if ((startup.runtime_if != 0U) && (next == target)) {
      startup.if_iq_magnitude = MotorApp_Slew(startup.if_iq_magnitude,
          MOTOR_APP_IF_IQ_A, MOTOR_APP_START_IQ_SLEW_A_S * foc.timer.Ts);
      startup.if_iq_reference = startup.reversal_target_direction * startup.if_iq_magnitude;
    }
    if (startup.forced_open_loop == 0U) {
      if (startup.if_verify_active != 0U) startup.if_hold_count++;
      MotorApp_UpdateObserverReady();
    }
    break;
  }

  case FOC_MOTOR_OBSERVER_HANDOVER:
    /* 接管进度只增不减，封顶 total；到顶由 UpdateStateTransition 切闭环。 */
    if (startup.handover_ticks < startup.handover_ticks_total)
      startup.handover_ticks++;
    break;

  case FOC_MOTOR_CLOSED_LOOP:
    /* 闭环淡入计数（封顶 1250 拍）：同时决定 Id 淡入比例与斜坡放开时刻。 */
    if (startup.speed_settle_count <
        MOTOR_APP_MS_TO_TICKS(MOTOR_APP_SPEED_SETTLE_MS))
      startup.speed_settle_count++;
    break;

  default:
    break;
  }
}

/* 提交拍末状态切换（必须在参考与 PWM 比较值都写入之后调用：切换拍仍用
 * 原状态控制量，新状态下一拍生效，避免边界拍参考跳变）。各状态退出条件：
 *   ALIGN        定时到 -> 虚拟角从定位角起算、速度从 0 爬升，转 I/F
 *   OPEN_LOOP_IF observer_ready -> 接管准备（可能故障，须复查闭锁再切换）；
 *                否则速度到位后进入有界验证窗口，超时判观测器不可信
 *   HANDOVER     渐变走完 -> 清淡入计数、置启动过渡标志，转闭环
 *   CLOSED_LOOP  固定 50 ms 接管保持结束后恢复常规速度斜率 */
static void MotorApp_UpdateStateTransition(void) {
  switch (foc_motor_state) {
  case FOC_MOTOR_ALIGN:
    if (startup.align_count >= MOTOR_APP_MS_TO_TICKS(MOTOR_APP_ALIGN_TIME_MS)) {
      /* 角度连续交接：I/F 从定位角(0 rad = alpha 轴)开始积分，速度从 0
       * 开始爬升。 */
      startup.if_angle = 0.0f;
      startup.if_speed_rad_s = 0.0f;
      foc_motor_state = FOC_MOTOR_OPEN_LOOP_IF;
    }
    break;

  case FOC_MOTOR_OPEN_LOOP_IF:
    if (startup.forced_open_loop != 0U) {
      /* 低速强制开环：虚拟角到达目标后保持，不等待观测器接管，也不触发
       * 恒速验证超时；端电压/Observer 仍可通过 status/wave_mode 观察。 */
      startup.observer_ready = 0U;
      startup.if_verify_active = 0U;
      if ((startup.reversal_braking == 0U) &&
          (fabsf(startup.if_speed_rad_s - startup.if_speed_target_rad_s) < 0.01f))
      {
        if (startup.runtime_if == 0U) {
          startup.if_ramp_rpm_s = MOTOR_APP_IF_ACCEL_RAD_S2 * 60.0f /
              (CORDIC_TWO_PI_F * (float)foc.observer.motor.pole_pairs);
          startup.if_iq_reference = startup.if_direction * startup.if_iq_magnitude;
        }
        startup.runtime_if = 1U;
      }
    } else if (startup.observer_ready != 0U) {
      /* 接管准备内部可能已进故障，切换前必须复查闭锁，否则会把已关桥的
       * 状态切进接管、下一拍继续提交参考。 */
      MotorApp_BeginObserverHandover();
      if (motor_fault_latched == 0U)
        foc_motor_state = FOC_MOTOR_OBSERVER_HANDOVER;
    } else if (fabsf(startup.if_speed_rad_s - startup.if_speed_target_rad_s) < 0.01f) {
      /* 速度到位但观测器未就绪：进入有界验证窗口（首次进入清三个计数）。 */
      if (startup.if_verify_active == 0U) {
        startup.if_verify_active = 1U;
        startup.if_hold_count = 0U;
        startup.ready_count = 0U;
        startup.ready_span_rad = 0.0f;
      }
      /* 有界保持：只在这里判定（而非每拍重置），保证超时一定能触发；
       * 37500 拍(1500ms) 后仍不就绪即停机。 */
      if (startup.if_hold_count >=
          MOTOR_APP_MS_TO_TICKS(MOTOR_APP_IF_VERIFY_TIME_MS))
        MotorApp_EnterFault(MOTOR_APP_FAIL_OBSERVER_NOT_READY);
    }
    break;

  case FOC_MOTOR_OBSERVER_HANDOVER:
    /* ticks_total>0 前置判断防止"0 拍接管"（此时 x 的分母为 0）。 */
    if ((startup.handover_ticks_total > 0U) &&
        (startup.handover_ticks >= startup.handover_ticks_total)) {
      startup.speed_settle_count = 0U;
      startup.speed_startup_active = 1U;
      startup.fault_count = 0U;
      foc_motor_state = FOC_MOTOR_CLOSED_LOOP;
    }
    break;

  case FOC_MOTOR_CLOSED_LOOP:
    /* 初次接管保持窗口到时即结束，不再等最终命令/电流误差连续稳定。
     * 否则调高目标或电压饱和时会一直卡在 800 rpm/s。 */
    if (startup.speed_settle_count >= MOTOR_APP_MS_TO_TICKS(MOTOR_APP_SPEED_SETTLE_MS))
      startup.speed_startup_active = 0U;
    break;

  default:
    break;
  }
}

/* ======================== 功率桥安全状态与启动 ======================== */

/* 从静止启动：校验目标 -> 关中断窗口内整体重建 startup 并开桥（避免挂起
 * 的 ADC 回调读到半初始化状态）。失败经 EnterFault 记录原因后返回；
 * 成功返回时状态为 ALIGN 且 PWM 已使能。I/F 目标在高速启动时取
 * min(|命令|, 1000 rpm) 供观测器接管；接管后速度环继续追随用户目标。 */
static void MotorApp_StartControlSequence(void) {
  float command = motor_control.speed_command_rpm;    /* 用户目标转速 */
  float pairs = (float)foc.observer.motor.pole_pairs; /* 极对数 */

  /* 前置校验：命令有限、控制周期有效、极对数有效。低速目标允许由强制
   * I/F 长期运行，因此这里不再用最低启动转速拒绝命令。 */
  if (!isfinite(command) || !isfinite(foc.timer.Ts) ||
      (foc.timer.Ts <= 0.0f) || (pairs <= 0.0f)) {
    MotorApp_EnterFault(MOTOR_APP_FAIL_START_COMMAND);
    return;
  }

  /* 母线校验：0.1 V 是"未上电/采样失效"的粗判，SVPWM 靠它定标。 */
  if (!isfinite(foc.state.vbus) || (foc.state.vbus <= 0.1f)) {
    MotorApp_EnterFault(MOTOR_APP_FAIL_BUS_INVALID);
    return;
  }

  /* 临界区：startup 重建 + 状态改 ALIGN + 使能 PWM 必须原子。 */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  /* 关中断后复查（可能在进入临界区前被 run 0 或故障撤销）。 */
  if ((motor_fault_latched != 0U) || (motor_run_requested == 0U) ||
      (motor_run_command == 0U)) {
    __set_PRIMASK(primask);
    return;
  }

  /* 停 CH4（ADC 注入触发源）期间不产生新的注入中断，保证下面的状态
   * 初始化不被打断；计数器仍在跑，PWM 载波不中断。 */
  if (HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4) != HAL_OK) {
    __set_PRIMASK(primask);
    Error_Handler();
    return;
  }

  /* 整体重建启动状态；回调耗时统计重新开始（观察本次启动的实时余量）。 */
  startup = (MotorApp_Startup_t){0};
  motor_callback_cycles_last = 0U;
  motor_callback_cycles_max = 0U;

  /* 方向由命令符号一次确定，之后同时作用于虚拟角与 Iq；
   * 先置启动过渡标志（它使能闭环健康的"启动期最低转速"检查）。 */
  startup.if_direction = (command > 0.0f) ? 1.0f : -1.0f;
  startup.reversal_target_direction = startup.if_direction;
  startup.if_iq_magnitude = MOTOR_APP_IF_IQ_A;
  startup.if_accel_rad_s2 = MOTOR_APP_IF_ACCEL_RAD_S2;
  startup.segment = 1U;
  startup.speed_startup_active = 1U;

  /* I/F 目标：低于 800 rpm 保持强制开环；更高目标先拖到 1000 rpm，
   * 接管后由速度环继续追随用户目标。 */
  MotorApp_SetOpenLoopTarget(command);

  /* 播种速度斜坡内部参考（接管后不必从 0 爬起）；只重置速度环状态机
   * 记录，保留用户配置的模式，接管完成前速度环被抑制。 */
  motor_control.speed_ref_rpm = command;
  motor_control.speed_loop_enable_last = 0U;

  /* 清诊断量：失败掩码、原因码、三份快照。 */
  motor_ready_fail_mask = 0U;
  motor_ready_fail_history = 0U;
  motor_start_fail_reason = MOTOR_APP_FAIL_NONE;
  motor_fault_snapshot.valid = 0U;
  motor_handover_snapshot.valid = 0U;
  motor_closed_snapshot.valid = 0U;
  motor_low_enter_snapshot.valid = 0U;

  /* 电压源回低速默认（实测优先）；状态机入口 ALIGN；开桥开始输出。 */
  voltage_source.measured_selected = MOTOR_APP_VOLTAGE_MEASURED;
  voltage_source.measured_weight = 1.0f;
  foc_motor_state = FOC_MOTOR_ALIGN;
  motor_idle_reset_done = 0U;
  FOC_PWM_Start();

  /* 恢复中断屏蔽到进入前的值（不无条件开中断，保持可嵌套语义）。 */
  __set_PRIMASK(primask);
}

/* ======================== 调试控制台 ======================== */

/* DebugConsole 底层发送回调：阻塞式发文本到 USART1。与 25 kHz 波形 DMA
 * 共用串口的互斥协议：置占用标志 + __DMB -> 限时 100ms 等 TC（等上一帧
 * DMA 发完；超时宁可不打印也不能死等，否则拖慢主循环甚至看门狗复位）
 * -> HAL 阻塞发送 -> 清标志。 */
static void MotorApp_DebugConsoleTx(const uint8_t *data, uint16_t length) {
  uint32_t start;

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

/* 对外显示的启动相位码（只用于上位机/波形看"启动到哪一步"，不写回电机
 * 状态字段，不与公开的 FOC_Motor_State_t 编号冲突）：
 *   30/31 = 高速 I/F 加速段 / 验证段；32/33 = 低速强制 I/F / 换向制动；
 *   10 = 闭环接管保持；12 = 高速闭环制动；
 *   其余状态直接用原始编号。读者：status 的 phase=、波形模式 1/2 第 1 通道。 */
static uint32_t MotorApp_StartupPhaseCode(void) {
  if ((foc_motor_state == FOC_MOTOR_CLOSED_LOOP) &&
      (startup.closed_loop_braking != 0U)) return 12U;
  if (foc_motor_state == FOC_MOTOR_OPEN_LOOP_IF) {
    if (startup.forced_open_loop != 0U)
      return (startup.reversal_braking != 0U) ? 33U : 32U;
    return (startup.if_verify_active != 0U) ? 31U : 30U;
  }

  if ((foc_motor_state == FOC_MOTOR_CLOSED_LOOP) &&
      (startup.speed_startup_active != 0U))
    return 10U;

  return (uint32_t)foc_motor_state;
}

/* 打印一份诊断快照（startdiag 的三段）。临界区只覆盖结构体拷贝（打印
 * 很慢不关中断）；拷到栈上保证多行数据来自同一拍快照。 */
static void MotorApp_PrintDiagnostic(const char *name,
                                     const MotorApp_Diagnostic_t *source) {
  MotorApp_Diagnostic_t x;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  x = *source;
  __set_PRIMASK(primask);

  if (x.valid == 0U) {
    DebugConsole_Printf("%s: no snapshot\r\n", name);
    return;
  }

  /* 数值转 unsigned long/double：newlib 的 printf 不支持 32 位 int/float 直传。 */
  DebugConsole_Printf("%s tick=%lu state=%lu reason=%lu mask=0x%03lX dir=%.0f\r\n",
      name, (unsigned long)x.tick, (unsigned long)x.state,
      (unsigned long)x.reason, (unsigned long)x.ready_fail, (double)x.direction);
  DebugConsole_Printf("  n_cmd=%.1f n_if=%.1f n_obs=%.1f n_obs_f=%.1f n_ref=%.1f delta=%.4f Vbus=%.3f\r\n",
      (double)x.command_rpm, (double)x.virtual_rpm,
      (double)x.observed_rpm, (double)x.observed_rpm_f,
      (double)x.reference_rpm, (double)x.delta, (double)x.vbus);
  DebugConsole_Printf("  Id_ref=%.3f Iq_ref=%.3f Id=%.3f Iq=%.3f\r\n",
      (double)x.id_ref, (double)x.iq_ref, (double)x.id, (double)x.iq);
  DebugConsole_Printf("  segment=%lu rpm_fb=%.1f a_obs=%.1f if_accel=%.1f measured=%u speed_i=%.3f\r\n",
      (unsigned long)x.segment, (double)x.control_rpm, (double)x.accel_rpm_s,
      (double)x.if_accel_rad_s2, (unsigned)x.measured_ramp, (double)x.speed_integral);
}

/** @brief startdiag 命令：按 接管 -> 闭环 -> 故障 顺序回放三份快照。 */
static void MotorApp_DebugStartup(int argc, char *argv[]) {
  (void)argc; (void)argv;
  MotorApp_PrintDiagnostic("HANDOVER", &motor_handover_snapshot);
  MotorApp_PrintDiagnostic("CLOSED", &motor_closed_snapshot);
  MotorApp_PrintDiagnostic("FAULT", &motor_fault_snapshot);
}

/* status 命令：打印整组状态（只读，各行读数不保证同一拍快照）：
 *   STATUS   闸门/状态/校准/复位/拍数/ARR/CCR4/MOE/rdy/失败掩码/故障码
 *            （ARR/CCR4/MOE 直读寄存器，确认 PWM 与 ADC 触发真的在跑）
 *   DRIVE    母线、命令/生效参考、观测转速原始与滤波（二者非独立测量）
 *   CURRENT  Iq 参考、实测 Id/Iq、Ud/Uq、电压限幅
 *   PWM      三相比较值与 CCER/CR1（确认通道使能与极性配置）
 *   OBSERVER 观测角/磁链/融合权重/三相端电压/控制角/dtheta（控制角与
 *            观测角之差：接管后应稳定在负载角附近，持续增长=没对齐）
 *   ADC      ADC2 的 JDR1/JDR2/JSQR 与自检结果（区分"V 相没被转换，
 *            JDR2 恒 0"与"V 相实测确实 0V"两种情况）
 *   START    意图/闭锁/相位码/虚拟转速/I-F 电流/转速偏差/验证保持计数
 *   TIMING   DWT 回调周期数/时间/预算占用（不含 HAL 中断入口开销） */
static void MotorApp_DebugStatus(int argc, char *argv[]) {
  uint32_t hclk_hz;
  double callback_us;
  double callback_max_us;
  double period_us;

  (void)argc;
  (void)argv;
  hclk_hz = HAL_RCC_GetHCLKFreq();
  callback_us = (hclk_hz != 0U)
      ? ((double)motor_callback_cycles_last * 1000000.0 / (double)hclk_hz)
      : 0.0;
  callback_max_us = (hclk_hz != 0U)
      ? ((double)motor_callback_cycles_max * 1000000.0 / (double)hclk_hz)
      : 0.0;
  period_us = 1000000.0 / (double)MOTOR_APP_CONTROL_HZ;
  MotorApp_PrintDiagnostic("LOW_ENTER", &motor_low_enter_snapshot);

  DebugConsole_Printf(
      "STATUS run=%lu state=%u cal=%u idle_reset=%u adc_irq=%lu ARR=%lu CCR4=%lu MOE=%u rdy=%lu rdy_fail=0x%03lX rdy_hist=0x%03lX rdy_count=%lu rdy_max=%lu span=%.3f fail=%lu\r\n",
      (unsigned long)motor_run_requested, (unsigned int)foc_motor_state,
      (unsigned int)foc.calibration.calibrated,
      (unsigned int)motor_idle_reset_done, (unsigned long)motor_adc_irq_count,
      (unsigned long)TIM1->ARR, (unsigned long)TIM1->CCR4,
      (unsigned int)((TIM1->BDTR & TIM_BDTR_MOE) != 0U),
      (unsigned long)startup.observer_ready,
      (unsigned long)motor_ready_fail_mask,
      (unsigned long)motor_ready_fail_history,
      (unsigned long)startup.ready_count,
      (unsigned long)startup.ready_max_count,
      (double)startup.ready_span_rad,
      (unsigned long)motor_start_fail_reason);
  DebugConsole_Printf("DRIVE Vbus=%.3f cmd=%.1f ref=%.1f rpm=%.1f rpm_f=%.1f speed_en=%lu\r\n",
      (double)foc.state.vbus, (double)motor_control.speed_command_rpm,
      (double)motor_control.speed_ref_active_rpm,
      (double)foc.observer.state.speed_rpm,
      (double)foc.observer.state.speed_rpm_f,
      (unsigned long)motor_control.reference.speed_loop_enable);
  DebugConsole_Printf("CURRENT Iq_ref=%.3f Id=%.3f Iq=%.3f Ud=%.3f Uq=%.3f Ulim=%.3f\r\n",
      (double)motor_control.iq_ref_active, (double)foc.state.i_dq.d,
      (double)foc.state.i_dq.q, (double)foc.state.u_dq.d,
      (double)foc.state.u_dq.q, (double)motor_control.voltage_limit);
  DebugConsole_Printf("CONTROL rpm_fb=%.1f slew=%.1f if_accel=%.1f sat_d=%d sat_q=%d speed_aw=%u brake=%u\r\n",
      (double)foc.observer.state.speed_rpm_f,
      (double)motor_control.speed_slew_rpm_per_s,
      (double)startup.if_accel_rad_s2,
      (int)motor_control.id_pi.saturation, (int)motor_control.iq_pi.saturation,
      (unsigned)motor_control.speed_voltage_limited,
      (unsigned)startup.closed_loop_braking);
  DebugConsole_Printf("MOTION segment=%lu a_obs=%.1f if_rate=%.1f measured=%u iq_if=%.3f speed_i=%.3f\r\n",
      (unsigned long)startup.segment, (double)startup.measured_accel_rpm_s,
      (double)startup.if_ramp_rpm_s, (unsigned)startup.if_measured_ramp,
      (double)startup.if_iq_reference, (double)motor_control.speed_pi.integral);
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
  DebugConsole_Printf("ADC JDR1=%lu JDR2=%lu JSQR=0x%08lX adc_cfg=%lu\r\n",
      (unsigned long)ADC2->JDR1, (unsigned long)ADC2->JDR2,
      (unsigned long)ADC2->JSQR, (unsigned long)motor_adc_cfg_ok);
  /* 自检不通过时把"为什么起不来"直接写进 status：上电那条报容易漏看。 */
  if (motor_adc_cfg_ok == 0U) {
    DebugConsole_Printf("ADC INJ MISMATCH expect ADC1 JL=3 JSQ=3,12,11,14 ; ADC2 JL=1 JSQ=3,17 ; adc_chk=%lu\r\n",
        (unsigned long)motor_adc_check_enable);
  }
  DebugConsole_Printf("START cmd_run=%lu latched=%lu phase=%lu if_rpm=%.1f if_iq=%.3f dn=%.1f hold=%lu\r\n",
      (unsigned long)motor_run_command, (unsigned long)motor_fault_latched,
      (unsigned long)MotorApp_StartupPhaseCode(), (double)MotorApp_VirtualRpm(),
      (double)(startup.reversal_target_direction * startup.if_iq_magnitude),
      (double)startup.ready_speed_error, (unsigned long)startup.if_hold_count);
  DebugConsole_Printf("TIMING dwt_cycles=%lu max=%lu time=%.3fus max_time=%.3fus budget=%.3fus load=%.1f%% max_load=%.1f%% hclk=%lu (HAL IRQ overhead excluded)\r\n",
      (unsigned long)motor_callback_cycles_last,
      (unsigned long)motor_callback_cycles_max,
      callback_us, callback_max_us, period_us,
      (period_us > 0.0) ? (callback_us * 100.0 / period_us) : 0.0,
      (period_us > 0.0) ? (callback_max_us * 100.0 / period_us) : 0.0,
      (unsigned long)hclk_hz);
  if (motor_fault_latched != 0U)
    DebugConsole_Printf("FAULT latched: inspect startdiag; set run 0 before another start\r\n");
}

/* Rs单一启动入口：串口、CAN和按键均共用同一套前置条件。 */
bool MotorApp_StartRsCalibration(void) {
  if ((foc_motor_state != FOC_MOTOR_IDLE) ||
      MotorCalibration_LsBusy() ||
      (foc.calibration.calibrated == 0U) ||
      (foc.state.vbus < 5.0f) || (foc.state.vbus > 50.0f) ||
      ((TIM1->BDTR & TIM_BDTR_MOE) != 0U)) {
    return false;
  }

  FOC_PWM_Stop();
  /* 辨识完成后不得沿用尚未执行的run 1自动启动FOC；上位机需再次明确下发run。 */
  motor_run_command = 0U;
  MotorApp_ApplyRunCommand();
  motor_calibration_just_started = 1U;
  MotorCalibration_Start();
  foc_motor_state = FOC_MOTOR_CALIBRATION;
  TIM1->CCR4 = foc.timer.adc_trigger;
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
    MotorCalibration_Stop();
    motor_cal.state = CAL_ERROR;
    return false;
  }
  return true;
}

void MotorApp_MarkCalibrationStarted(void) {
  motor_calibration_just_started = 1U;
}

static void MotorDebug_Rs(int argc, char *argv[]) {
  MotorProtocol_CalibrationCancel();
  if ((argc == 2) && (strcmp(argv[1], "stop") == 0)) {
    MotorCalibration_Stop();
    DebugConsole_Printf("Rs stopped\r\n");
    return;
  }
  if ((argc != 1) || !MotorApp_StartRsCalibration()) {
    DebugConsole_Printf("Rs rejected: use rs | rs stop; check idle, ADC, Vbus\r\n");
    return;
  }
  DebugConsole_Printf("Rs started\r\n");
}

static void MotorDebug_Ls(int argc, char *argv[]) {
  MotorProtocol_CalibrationCancel();
  if ((argc == 2) && (strcmp(argv[1], "stop") == 0)) {
    MotorCalibration_Stop();
    DebugConsole_Printf("Ls stopped\r\n");
    return;
  }
  if (MotorCalibration_LsBusy()) {
    DebugConsole_Printf("Ls rejected: calibration already active\r\n");
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
    char *end = NULL;
    float rs = strtof(argv[pos], &end);
    if ((end == NULL) || (*end != '\0') || !(rs >= 0.1f && rs <= 2.0f) ||
        (foc_motor_state != FOC_MOTOR_IDLE)) {
      DebugConsole_Printf("Usage: ls [all|ab|bc|ca] [Rs_ohm] | ls stop\r\n");
      return;
    }
    motor_cal.rs = rs;
    DebugConsole_Printf("Ls using supplied Rs=%.6fohm\r\n", (double)rs);
  }
  motor_run_command = 0U;
  MotorApp_ApplyRunCommand();
  motor_calibration_just_started = 1U;
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

static void MotorDebug_Cal(int argc, char *argv[]) {
  if ((argc != 2) || (strcmp(argv[1], "show") != 0)) {
    DebugConsole_Printf("Usage: cal show\r\n");
    return;
  }
  DebugConsole_Printf("Rs=%.6fohm Ls AB/BC/CA=%.3f/%.3f/%.3fuH\r\n",
      (double)motor_cal.rs, (double)(motor_cal.ls_ab * 1e6f),
      (double)(motor_cal.ls_bc * 1e6f), (double)(motor_cal.ls_ca * 1e6f));
}

/* 把命令名直接绑定到现有控制结构（避免另维护一份参数副本）。范围只
 * 约束控制台写入，不等于控制器内部限幅；只读项仅供 get / Live Watch。
 * 注册对象必须是 static 变量或外设寄存器（固件运行期内始终有效）。 */
static HAL_StatusTypeDef MotorApp_RegisterDebugVariables(void) {
  uint8_t success = 1U;

  /* 外部命令与环路增益：单位见 controller.h。 */
  success &= DebugConsole_RegisterF32("id", &motor_control.id_ref,
                                      -8.0f, 8.0f, false);
  success &= DebugConsole_RegisterF32("iq", &motor_control.iq_ref,
                                      -10.0f, 10.0f, false);
  /* speed 改动由状态机选择闭环制动或低速强制拖动。 */
  success &= DebugConsole_RegisterF32("speed", &motor_control.speed_command_rpm,
                                      -15000.0f, 15000.0f, false);
  success &= DebugConsole_RegisterF32("speed_kp", &motor_control.speed_pi.kp,
                                      0.0f, 1.0f, false);
  success &= DebugConsole_RegisterF32("speed_ki", &motor_control.speed_pi.ki,
                                      0.0f, 100.0f, false);
  /* speed_slew：运行速度参考斜坡率（rpm/s），0 表示保持参考。 */
  success &= DebugConsole_RegisterF32(
      "speed_slew", &motor_control.speed_slew_rpm_per_s,
      0.0f, 30000.0f, false);
  /* speed_en：0=电流模式（直接用 iq），1=速度模式（速度环输出 Iq）。 */
  success &= DebugConsole_RegisterBool("speed_en",
                                       &motor_control.speed_loop_enable,
                                       false);
  success &= DebugConsole_RegisterBool("just_float", &just_float_enabled,
                                       false);
  /* run：用户意图；写 0 兼作故障闭锁的解除动作。 */
  success &= DebugConsole_RegisterBool("run", &motor_run_command, false);
  success &= DebugConsole_RegisterCommand("status", MotorApp_DebugStatus,
                                           "status: show motor startup state");

  /* vbus 可手动改写（危险：影响 SVPWM 定标与电压限幅）。 */
  success &= DebugConsole_RegisterF32("vbus", &foc.state.vbus,
                                      0.0f, 70.0f, false);
  /* 只读观察点：三相端电压、电压源选择/权重、注入自检、本拍控制角、
   * 就绪失败掩码与故障码。 */
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
  success &= DebugConsole_RegisterU32("adc_cfg", &motor_adc_cfg_ok, 0U, 1U,
                                      true);
  /* adc_chk：自检门控开关，可写；置 0 可在现场临时绕过"注入配置不匹配
   * 拒绝启动"的门控（采样通道是否可信需自行判断）。 */
  success &= DebugConsole_RegisterU32("adc_chk", &motor_adc_check_enable, 0U,
                                      1U, false);
  success &= DebugConsole_RegisterF32("ctrl_rad", &motor_control_angle_rad,
                                      -4.0f, 4.0f, true);
  success &= DebugConsole_RegisterU32("rdy_fail", &motor_ready_fail_mask, 0U,
                                      0xFFFU, true);
  success &= DebugConsole_RegisterU32("fail", &motor_start_fail_reason, 0U, 8U,
                                      true);

  /* 可写：波形通道选择与 I/F 削减开关。 */
  success &= DebugConsole_RegisterU32("wave_mode", &motor_wave_mode, 0U, 4U, false);
  success &= DebugConsole_RegisterBool("if_trim", &motor_if_trim_enable, false);
  success &= DebugConsole_RegisterCommand("startdiag", MotorApp_DebugStartup,
                                           "startdiag: latched handover/closed/fault snapshots");
  success &= DebugConsole_RegisterCommand("rs", MotorDebug_Rs,
                                           "rs: identify AB/BC/CA resistance; rs stop: abort");
  success &= DebugConsole_RegisterCommand("ls", MotorDebug_Ls,
                                           "ls: AB->BC->CA; ls [all|ab|bc|ca] [Rs_ohm]; ls stop: abort");
  success &= DebugConsole_RegisterCommand("cal", MotorDebug_Cal,
                                           "cal show (RAM calibration only)");

  return (success != 0U) ? HAL_OK : HAL_ERROR;
}

/* ======================== 波形输出 ======================== */

/* 尝试启动一帧波形发送：TC=1（上一帧整帧发完）才允许改共用帧，否则丢弃
 * 本拍并返回 -1（忙则丢弃、不等待——一帧约 180us 远大于 40us 控制周期，
 * 等待会破坏实时性）。通道含义由调用者决定（见 OnInjectedConversion）。 */
static int MotorApp_SendJustFloat(float f0, float f1, float f2,
                                  float f3, float f4, float f5, float f6) {
  if ((USART1->ISR & USART_ISR_TC) == 0U) {
    return -1;
  }

  just_float_frame.data[0] = f0;
  just_float_frame.data[1] = f1;
  just_float_frame.data[2] = f2;
  just_float_frame.data[3] = f3;
  just_float_frame.data[4] = f4;
  just_float_frame.data[5] = f5;
  just_float_frame.data[6] = f6;

  /* 重装长度前先停 DMA 并确认停稳，避免指针用旧长度造成半帧/越界搬运。 */
  __HAL_DMA_DISABLE(&hdma_usart1_tx);
  while ((hdma_usart1_tx.Instance->CCR & DMA_CCR_EN) != 0U) {
  }

  __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx,
                       __HAL_DMA_GET_GI_FLAG_INDEX(&hdma_usart1_tx));
  /* 32 字节 = 7*4 + 4。 */
  hdma_usart1_tx.Instance->CNDTR = sizeof(MotorApp_JustFloatFrame_t);
  /* 清 TC 标志，否则刚清完又可能立刻满足"完成"条件。 */
  USART1->ICR = USART_ICR_TCCF;

  /* __DMB 保证 DMA 使能发生在写 data[] 与 CNDTR 之后。 */
  __DMB();
  __HAL_DMA_ENABLE(&hdma_usart1_tx);

  return 0;
}

/* ======================== 模拟前端与功率级初始化 ======================== */

/* 上电最早期（任何 HAL/MX 初始化之前）把六路 PWM 引脚置为推挽输出低：
 * 复位后引脚默认浮空输入，栅极驱动悬空可能被噪声耦合为高电平误导通
 * MOSFET；之后由 CubeMX 的 GPIO/TIM 初始化改回复用。不做错误检查——
 * 上电安全动作必须无条件执行。
 *   PA8/9/10 -> TIM1_CH1/2/3，PB13/14/15 -> TIM1_CH1N/2N/3N。 */
void MotorApp_ForcePowerStageSafe(void) {
  GPIO_InitTypeDef gpio = {0};

  /* 先开 GPIO 时钟，否则写/配置无效。 */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* 先写数据寄存器再改模式，避免"刚切输出就输出高"的瞬间。 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
                    GPIO_PIN_RESET);

  /* 推挽输出、无上下拉（外部栅极驱动自带偏置）、低速档即可。 */
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOB, &gpio);
}

/* ======================== 对外接口 ======================== */

/* 返回应用层唯一的 FOC 控制器实例（永不 NULL）：供协议层写 volatile
 * 外部命令字段；不得直接改 reference 或 PI 运行状态（属于中断域）。 */
FOC_Control_t *MotorApp_GetControl(void) {
  return &motor_control;
}

/* 写入运行请求（用户意图），并立即 ApplyRunCommand——让 CAN 调用方
 * 无需等主循环就看到闸门跟随意图，显式 run 0 也能在中断上下文立刻
 * 解除闭锁。只改意图，不越过 MotorApp_Process 的启动前置条件。 */
void MotorApp_RequestRun(uint8_t run) {
  motor_run_command = (run != 0U) ? 1U : 0U;
  MotorApp_ApplyRunCommand();
}

/* 应用层初始化（main 的 MX_*_Init 之后、while(1) 之前调用一次；前置
 * 条件：FOC_Data_Init 已在 MX_TIM1_Init 之前执行）。关键顺序：
 *   DWT 延时 -> CORDIC -> 波形 DMA 预配置 -> 控制器增益 -> 关桥清比较值
 *   -> 调试控制台与寄存器注册 -> 模拟前端校准 -> 母线初值
 *   -> 启动 ADC2 注入（无中断）-> 启动 ADC1 注入中断（默认 JEOC，随后
 *   换成 JEOS，保证每周期只进一次控制回调）-> 注入序列回读自检
 *   -> 设置 CH4 比较值并启动 CH4（会置 MOE，但 CH1~3 未使能，桥不输出）。
 * 返回 HAL_OK 后注入中断开始产生，零偏校准自动进行。 */
HAL_StatusTypeDef MotorApp_Init(void) {
  OPAMP_HandleTypeDef *const opamps[3] = {&hopamp1, &hopamp2, &hopamp3}; /* 三相电流前端运放 */
  uint32_t jsqr1, jsqr2; /* 注入序列回读：两路 ADC 的 JSQR 原始值 */
  uint32_t jl1, jl2;     /* 注入序列回读：JL = 转换次数-1，截断最直接判据 */
  uint8_t adc_inj_ok;    /* 注入序列自检结果 */
  uint32_t i;            /* 校准循环索引 */

  /* DWT：既是微秒级延时的时钟源，也是回调耗时统计的计数器。 */
  if (DWT_Delay_Init() == 0U) return HAL_ERROR;
  motor_dwt_hclk_hz = HAL_RCC_GetHCLKFreq();

  CORDIC_SinCos_RegisterConfig();

  /* 波形 DMA 一次性配置：固定两端地址（CPAR=&TDR，CMAR=&帧），长度每次
   * 发送前重装；不用 HAL 的 DMA 发送——每拍重发同一段固定内存，HAL 状态机
   * 开销大且与"忙则丢弃"策略不匹配。帧尾 0x7F800000 = +Inf 位模式，
   * 上位机据此切分 JustFloat 帧。 */
  just_float_frame.tail = 0x7F800000UL;
  just_float_enabled = 0U;

  /* 先断开 USART 的 DMA 请求，并确认 DMA 通道真正停止（EN 清零是异步的）。 */
  CLEAR_BIT(USART1->CR3, USART_CR3_DMAT);
  __HAL_DMA_DISABLE(&hdma_usart1_tx);
  while ((hdma_usart1_tx.Instance->CCR & DMA_CCR_EN) != 0U) {
  }
  /* 清全局中断标志，避免残留标志在使能后立刻触发假完成。 */
  __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx,
                       __HAL_DMA_GET_GI_FLAG_INDEX(&hdma_usart1_tx));
  hdma_usart1_tx.Instance->CPAR = (uint32_t)&USART1->TDR;
  hdma_usart1_tx.Instance->CMAR = (uint32_t)&just_float_frame;
  hdma_usart1_tx.Instance->CNDTR = 0U;
  SET_BIT(USART1->CR3, USART_CR3_DMAT);

  /* 用实际控制周期 Ts 初始化电流/速度 PI 的离散积分步长。 */
  FOC_Control_Init(&motor_control, foc.timer.Ts);

  /* 保持桥关闭：四个比较值清零（CH4 稍后写触发位置），关 MOE——即使
   * CH1~3 之后被误使能也不会输出。 */
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  TIM1->CCR3 = 0U;
  TIM1->CCR4 = 0U;
  __HAL_TIM_MOE_DISABLE(&htim1);

  /* 调试控制台：绑定 USART1 与阻塞发送回调；失败直接返回（后续诊断
   * 都依赖它）。 */
  if (DebugConsole_Init(&huart1, MotorApp_DebugConsoleTx) != HAL_OK) return HAL_ERROR;
  if (MotorApp_RegisterDebugVariables() != HAL_OK) return HAL_ERROR;

  /* 模拟前端校准（功率桥关闭状态）：SelfCalibrate 必须在 Start 之前做，
   * 先校准三个运放再统一启动；不代替 25 kHz 中断里的三相零偏平均。 */
  for (i = 0U; i < 3U; i++) {
    if (HAL_OPAMP_SelfCalibrate(opamps[i]) != HAL_OK) return HAL_ERROR;
  }
  for (i = 0U; i < 3U; i++) {
    if (HAL_OPAMP_Start(opamps[i]) != HAL_OK) return HAL_ERROR;
  }

  /* 等运放输出与偏置网络（1.65 V 中点）稳定。 */
  DWT_Delay_Ms(10U);
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) return HAL_ERROR;
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) return HAL_ERROR;
  /* 等 ADC 校准结果生效（HAL 在转换前自动加载校准系数）。 */
  DWT_Delay_Ms(10U);

  /* 电流采样中断开始前先取母线/温度初值；失败保留零初值（后续 vbus
   * 校验会拦住启动）。 */
  if (BoardAdc_Update() == HAL_OK) {
    foc.state.vbus = BoardAdc_GetMeasurements()->vbus_voltage;
    foc.state.temperature_c = BoardAdc_GetMeasurements()->temperature_c;
  }

  /* ADC2 只有一个注入通道，先启动不开中断：结果在 ADC1 完整序列结束时
   * 统一读取，共享的 ADC1_2 IRQ 只由 ADC1 产生控制中断。 */
  if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK) return HAL_ERROR;

  /* 启动 ADC1 注入并开中断。从这里开始 25 kHz 中断产生；状态为 IDLE，
   * 中断只走关桥+校准路径。 */
  if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) return HAL_ERROR;

  /* 注入序列回读自检：HAL 在 ScanConvMode=DISABLE 时会静默截断注入序列
   * 且仍返回 HAL_OK（V 相端电压曾因此从未被采样、JDR2 恒 0），只能靠
   * 回读发现；失败只置标志、拒绝 run，不进 Error_Handler（保持串口可诊断）。 */
  jsqr1 = ADC1->JSQR;
  jsqr2 = ADC2->JSQR;
  jl1 = (jsqr1 & ADC_JSQR_JL) >> ADC_JSQR_JL_Pos;
  jl2 = (jsqr2 & ADC_JSQR_JL) >> ADC_JSQR_JL_Pos;
  adc_inj_ok = 1U;

  /* ADC1 四个 Rank（Ia/Ic/U端/W端）与 ADC2 两个 Rank（Ib/V端）逐字段核对。 */
  if ((jl1 != MOTOR_APP_ADC1_INJ_JL) ||
      (MOTOR_APP_JSQR_FIELD(jsqr1, JSQ1) != MOTOR_APP_ADC1_INJ_JSQ1) ||
      (MOTOR_APP_JSQR_FIELD(jsqr1, JSQ2) != MOTOR_APP_ADC1_INJ_JSQ2) ||
      (MOTOR_APP_JSQR_FIELD(jsqr1, JSQ3) != MOTOR_APP_ADC1_INJ_JSQ3) ||
      (MOTOR_APP_JSQR_FIELD(jsqr1, JSQ4) != MOTOR_APP_ADC1_INJ_JSQ4)) {
    adc_inj_ok = 0U;
  }
  if ((jl2 != MOTOR_APP_ADC2_INJ_JL) ||
      (MOTOR_APP_JSQR_FIELD(jsqr2, JSQ1) != MOTOR_APP_ADC2_INJ_JSQ1) ||
      (MOTOR_APP_JSQR_FIELD(jsqr2, JSQ2) != MOTOR_APP_ADC2_INJ_JSQ2)) {
    adc_inj_ok = 0U;
  }
  if (adc_inj_ok == 0U) {
    /* 打印实际 JSQR，便于直接对照 adc.c 判断是哪一段被改写。 */
    DebugConsole_Printf("ADC INJ FAIL ADC1 JSQR=0x%08lX JL=%lu ADC2 JSQR=0x%08lX JL=%lu\r\n",
        (unsigned long)jsqr1, (unsigned long)jl1,
        (unsigned long)jsqr2, (unsigned long)jl2);
    DebugConsole_Printf("ADC INJ expect ADC1 JL=3 JSQ=3,12,11,14 ; ADC2 JL=1 JSQ=3,17 ; run disabled\r\n");
  }
  motor_adc_cfg_ok = adc_inj_ok;

  /* EOCSelection 须保留 ADC_EOC_SINGLE_CONV 供规则组逐 Rank 轮询（HAL
   * 据此默认开 JEOC）；这里把注入中断换成 JEOS，确保 ADC1 的 U/W 两个
   * Rank 全部完成后每周期只进一次控制回调。 */
  __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOC);
  __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOS);

  /* CH4 比较值 = adc_trigger：决定注入触发在 PWM 周期内的位置（通常落在
   * 下桥臂全开、电流可采的窗口）。必须用 HAL_TIM_PWM_Start（HAL 才会
   * 记录通道状态）；这步会置 MOE，但 CH1~3 未使能，桥依旧无输出。 */
  TIM1->CCR4 = foc.timer.adc_trigger;
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) return HAL_ERROR;

  return HAL_OK;
}

/* 主循环处理（只做非实时任务）：串口命令解析（先于 ADC 轮询，避免轮询
 * 等待拖慢启停响应）-> 同步运行闸门 -> 规则组轮询母线/温度 -> 注入配置
 * 不匹配时提示一次并拒绝启动（set adc_chk 0 绕过）-> 六项启动门控全部
 * 满足才 StartControlSequence：run 请求 + 未闭锁 + 注入配置通过 + 校准
 * 完成 + IDLE 复位完成 + 状态为 IDLE（重复 run 1 不会重启电机；校准期的
 * run 1 会一直等到校准结束）。 */
void MotorApp_Process(void) {
  /* 只发送/提示一次的标志，避免刷屏。 */
  static uint8_t ready_reported = 0U;
  static uint8_t adc_cfg_reported = 0U;
  static uint8_t previous_run_command = 0U;
  static uint8_t calibration_seen = 0U;

  if (ready_reported == 0U) {
    ready_reported = 1U;
    DebugConsole_Printf("READY: set run 1 / set run 0 / status\r\n");
  }

  DebugConsole_Process();
  MotorApp_ApplyRunCommand();

  /* 串口/CAN的 run 0 下降沿也必须撤销辨识占用；不把上电时默认的
   * run=0误认为用户刚刚停止，避免辨识启动后被同一主循环立即取消。 */
  if ((motor_calibration_just_started == 0U) &&
      (calibration_seen != 0U) && (previous_run_command != 0U) &&
      (motor_run_command == 0U) &&
      ((foc_motor_state == FOC_MOTOR_CALIBRATION) || MotorCalibration_LsBusy())) {
    MotorProtocol_CalibrationCancel();
    MotorCalibration_Stop();
  }
  previous_run_command = motor_run_command;
  motor_calibration_just_started = 0U;
  calibration_seen = ((foc_motor_state == FOC_MOTOR_CALIBRATION) ||
                       MotorCalibration_LsBusy()) ? 1U : 0U;

  /* 参数辨识的 Ls 脉冲会临时占用规则组 ADC；此时不能由主循环重新启动
   * 普通 Vbus/温度采样，否则会抢占 DMA 和触发源。 */
  if (!MotorCalibration_LsBusy()) {
    if (BoardAdc_Update() == HAL_OK) {
      foc.state.vbus = BoardAdc_GetMeasurements()->vbus_voltage;
      foc.state.temperature_c = BoardAdc_GetMeasurements()->temperature_c;
    }
  }

  /* 闸门归零时复位提示标志，使下一次 run 请求能重新提示一次。 */
  if (motor_run_requested == 0U) {
    adc_cfg_reported = 0U;
  }
  if ((motor_run_requested != 0U) && (motor_adc_cfg_ok == 0U) &&
      (motor_adc_check_enable != 0U) && (adc_cfg_reported == 0U)) {
    adc_cfg_reported = 1U;
    DebugConsole_Printf("run ignored: ADC injected config mismatch, see status adc_cfg (set adc_chk 0 to bypass)\r\n");
  }

  if ((motor_run_requested != 0U) &&
      (motor_fault_latched == 0U) &&
      ((motor_adc_cfg_ok != 0U) || (motor_adc_check_enable == 0U)) &&
      (foc.calibration.calibrated != 0U) &&
      (motor_idle_reset_done != 0U) &&
      (foc_motor_state == FOC_MOTOR_IDLE)) {
    MotorApp_StartControlSequence();
  }

  /* Rs/Ls拟合、换相和外设恢复全部在主循环完成，避免阻塞25 kHz中断。 */
  MotorCalibration_Process();
}

/**
 * @brief 注入转换完成后的实时入口（25 kHz，单拍预算 40 us ≈ 6800 周期）。
 *
 * 只处理 ADC1：ADC2 共享 ADC1_2 中断向量但未开注入中断，显式排除防止
 * 同拍重复执行。ADC1 依次采 Ia/Ic/U端/W端，ADC2 采 Ib/V端；入口由 ADC1
 * 四个 Rank 全部完成后的 JEOS 产生，此时两路结果均已就绪。
 * 每拍流程（顺序不可调换）：停机/状态闸门 -> 零偏校准（未完成则本拍到此
 * 为止：不写 PWM、不统计耗时） -> 采样 Clarke -> 组装观测器输入 -> 主动
 * 控制拍（守卫/状态机/电流环/写 PWM/状态切换） -> 母线电流估算 -> 波形
 * 提交 -> 耗时统计。
 */
void MotorApp_OnInjectedConversion(ADC_HandleTypeDef *hadc) {
  static uint16_t calibration_count = 0U; /* 零偏校准拍数：唯一跨拍保持量，仅中断域读写 */
  Observer_Input_t observer_input = {0};  /* 本拍观测器输入：逐字段填满后交给状态机 */
  uint32_t callback_cycles_start;         /* DWT 起点：末尾差值即回调耗时 */
  float speed_abs_rpm;                    /* 观测转速绝对值：电压源迟滞判决 */
  float blend_step;                       /* 电压融合权重单拍增量 = Ts/20ms */

  if ((hadc == NULL) || (hadc->Instance != ADC1)) return;

  /* 拍计数（诊断时间戳 + 波形 25 分频用）。 */
  motor_adc_irq_count++;
  callback_cycles_start = DWT->CYCCNT;

  /* run 0：只改状态，真正关桥统一交给下面的 ResetIdle（停止路径与其它
   * 安全状态共用同一条代码）。 */
  if ((motor_run_requested == 0U) &&
      (foc_motor_state != FOC_MOTOR_CALIBRATION)) {
    foc_motor_state = FOC_MOTOR_IDLE;
  }

  /* 参数辨识独占功率级和ADC规则组：Rs阶段仍由TIM1注入回调累计三相
   * 电流，Ls阶段则只允许DMA/TIM3路径工作，绝不能落入FOC状态机。 */
  if (foc_motor_state == FOC_MOTOR_CALIBRATION) {
    if (motor_cal.state == CAL_IDLE) {
      motor_callback_cycles_last = DWT->CYCCNT - callback_cycles_start;
      if (motor_callback_cycles_last > motor_callback_cycles_max)
        motor_callback_cycles_max = motor_callback_cycles_last;
      return;
    }
    FOC_Get_Iabc(&foc, (uint16_t)ADC1->JDR1, (uint16_t)ADC2->JDR1,
                 (uint16_t)ADC1->JDR2);
    MotorCalibration_Run(foc.state.i_abc.a, foc.state.i_abc.b,
                          foc.state.i_abc.c, foc.state.vbus);
    motor_callback_cycles_last = DWT->CYCCNT - callback_cycles_start;
    if (motor_callback_cycles_last > motor_callback_cycles_max)
      motor_callback_cycles_max = motor_callback_cycles_last;
    return;
  }

  /* 安全状态关桥 + IDLE 复位；主动控制状态清 idle_reset_done（允许下次
   * 回 IDLE 重新完整复位一次）。 */
  if (MotorApp_IsControlState(foc_motor_state) == 0U) {
    foc_motor_state = FOC_MOTOR_IDLE;

    /* 每拍无条件先清 MOE 撤销输出；重量级复位（停 HAL 通道/清 PI/清
     * 观测器）只在首次进入 IDLE 做一次（motor_idle_reset_done 去重——
     * 25 kHz 里每拍做这些多寄存器写会挤占实时预算且无意义）。刻意保留：
     * CH4（ADC 触发）、零偏校准结果、用户命令与增益。 */
    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
    if (motor_idle_reset_done == 0U) {
      /* 停六路（主+互补），HAL 通道状态回 READY，下次才能重新 Start。 */
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
      HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
      HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
      TIM1->CCR1 = 0U;
      TIM1->CCR2 = 0U;
      TIM1->CCR3 = 0U;

      /* 清 PI 运行状态（保留 Kp/Ki/限幅）：否则下次启动带积分残留造成
       * 启动电流冲击；再清电压链路缓存与 SVPWM 输出，避免用陈旧值。 */
      FOC_Control_Reset(&motor_control);
      foc.state.u_dq = (FOC_DQ_t){0};
      foc.state.u_alpha_beta = (FOC_AlphaBeta_t){0};
      foc.state.u_abc = (FOC_ABC_t){0};
      foc.svpwm = (FOC_SVPWM_Output_t){0};

      /* 观测器整体清零后重新初始化：磁链重置到 alpha 轴正方向、PLL 复位
       * ——正是 ALIGN 固定角 0 rad 所期望的初始基准。 */
      foc.observer.state = (Observer_State_t){0};
      Observer_Init(&foc.observer, &foc.observer.motor, &foc.observer.config);
      voltage_source.measured_selected = MOTOR_APP_VOLTAGE_MEASURED;
      voltage_source.measured_weight = 1.0f;
      motor_run_requested = 0U;
      motor_idle_reset_done = 1U;
    }
  } else {
    motor_idle_reset_done = 0U;
  }

  /* 零偏校准期（前 1000 拍约 40ms；桥未驱动、无相电流是前提）：只累加
   * 原始码，均值化后锁存；本拍直接返回——不跑观测器、不写 PWM，即使
   * 此时收到 run 请求（状态已是 ALIGN）也不会输出。
   * 通道：ADC1 JDR1=Ia、JDR2=Ic；ADC2 JDR1=Ib（JEOS 刚触发，读到的
   * 必是最新结果）。 */
  if (foc.calibration.calibrated == 0U) {
    calibration_count++;
    foc.calibration.ia_offset +=
        HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    foc.calibration.ib_offset +=
        HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    foc.calibration.ic_offset +=
        HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

    if (calibration_count >= CURRENT_OFFSET_SAMPLE_NUM) {
      /* 均值化锁存（单位原始码），之后 FOC_Get_Iabc 先减偏移再换算安培；
       * 仅标记就绪，保持 IDLE，启动由串口 run 命令决定。 */
      foc.calibration.ia_offset /= (float)CURRENT_OFFSET_SAMPLE_NUM;
      foc.calibration.ib_offset /= (float)CURRENT_OFFSET_SAMPLE_NUM;
      foc.calibration.ic_offset /= (float)CURRENT_OFFSET_SAMPLE_NUM;
      foc.calibration.calibrated = 1U;
    }
    /* 校准分支也更新一次 DWT 计时，避免 status 在上电校准阶段显示上一次
     * 旧值；此处测到的是本拍采样/累加/校准逻辑耗时。 */
    motor_callback_cycles_last = DWT->CYCCNT - callback_cycles_start;
    if (motor_callback_cycles_last > motor_callback_cycles_max)
      motor_callback_cycles_max = motor_callback_cycles_last;
    return;
  }

  /* ---- 采样：三相电流 + 三相端电压 -> 物理量 -> Clarke ---- */

  /* 电流通道：ADC1 JDR1 = Ia，ADC2 JDR1 = Ib，ADC1 JDR2 = Ic。 */
  FOC_Get_Iabc(&foc, (uint16_t)ADC1->JDR1, (uint16_t)ADC2->JDR1,
               (uint16_t)ADC1->JDR2);
  FOC_Clarke(&foc.state.i_abc, &foc.state.i_alpha_beta);

  /* 端电压与电流同一次 TIM1 触发（ADC1 JDR3=U、JDR4=W，ADC2 JDR2=V）：
   * 直接乘分压换算系数（端电压无零点偏置，故不用 FOC_Get_*）；
   * Clarke 消去 Vbus/2 等三相共模。 */
  foc.state.u_abc_measured.a =
      (float)(uint16_t)ADC1->JDR3 * BOARD_ADC_COUNT_TO_VOLTAGE;
  foc.state.u_abc_measured.b =
      (float)(uint16_t)ADC2->JDR2 * BOARD_ADC_COUNT_TO_VOLTAGE;
  foc.state.u_abc_measured.c =
      (float)(uint16_t)ADC1->JDR4 * BOARD_ADC_COUNT_TO_VOLTAGE;
  FOC_Clarke(&foc.state.u_abc_measured, &foc.state.u_alpha_beta_measured);

  /* ---- 组装观测器输入：必须用"上一拍实际施加的占空比"（与采样时刻
   * 电压一致）；先观测、后计算本拍新电压，否则引入一拍前馈错位 ---- */
  observer_input.duty_a = foc.svpwm.duty_a;
  observer_input.duty_b = foc.svpwm.duty_b;
  observer_input.duty_c = foc.svpwm.duty_c;
  observer_input.vbus = foc.state.vbus;
  observer_input.i_alpha = foc.state.i_alpha_beta.alpha;
  observer_input.i_beta = foc.state.i_alpha_beta.beta;

  /* ---- 端电压融合：实测/重构权重按迟滞选目标、20 ms 渐变（在采样
   * Clarke 之后、状态机之前，保证观测器拿到本拍电压）。端电压是对地
   * 电压，Clarke 已消共模，无需减 Vbus/2。---- */
  observer_input.measured_u_alpha = foc.state.u_alpha_beta_measured.alpha;
  observer_input.measured_u_beta = foc.state.u_alpha_beta_measured.beta;

  /* 迟滞：>=1200 rpm 切重构电压，<=900 切回实测；中间区间保持原选择
   * 不抖动，正反转共用同一组阈值。 */
  speed_abs_rpm = (foc_motor_state == FOC_MOTOR_OPEN_LOOP_IF)
      ? fabsf(MotorApp_VirtualRpm()) : fabsf(foc.observer.state.speed_rpm);
  if (speed_abs_rpm >= MOTOR_APP_CALCULATED_VOLTAGE_ENTER_RPM) {
    voltage_source.measured_selected = MOTOR_APP_VOLTAGE_CALCULATED;
  } else if (speed_abs_rpm <= MOTOR_APP_MEASURED_VOLTAGE_RETURN_RPM) {
    voltage_source.measured_selected = MOTOR_APP_VOLTAGE_MEASURED;
  }

  /* 权重增量 = Ts/20ms（控制周期被改大到超过 20 ms 则一拍到位），朝目标
   * 方向逼近并夹紧 [0,1]——等价于给观测器电压输入加一阶斜坡，切换不
   * 产生磁链冲击。 */
  blend_step = foc.timer.Ts / MOTOR_APP_VOLTAGE_BLEND_TIME_S;
  if (blend_step > 1.0f) blend_step = 1.0f;
  voltage_source.measured_weight +=
      (voltage_source.measured_selected == MOTOR_APP_VOLTAGE_MEASURED)
          ? blend_step : -blend_step;
  if (voltage_source.measured_weight > 1.0f) voltage_source.measured_weight = 1.0f;
  if (voltage_source.measured_weight < 0.0f) voltage_source.measured_weight = 0.0f;
  /* 观测器内部按 u = w*measured + (1-w)*duty*Vbus 融合。 */
  observer_input.measured_voltage_weight = voltage_source.measured_weight;

  /* ---- 主动控制拍：IDLE 必须完全跳过——否则电流 PI 会在关桥期间积分
   * 饱和，下次 run 从饱和积分起步造成启动电流冲击 ---- */
  if (MotorApp_IsControlState(foc_motor_state) != 0U) {
    /* 入口守卫：任一控制输入非有限（NaN 会污染整条链路）或母线过低
     * （SVPWM 无法定标）直接判故障。 */
    if (!isfinite(foc.state.vbus) ||
        !isfinite(foc.state.i_alpha_beta.alpha) || !isfinite(foc.state.i_alpha_beta.beta) ||
        !isfinite(foc.state.u_alpha_beta_measured.alpha) ||
        !isfinite(foc.state.u_alpha_beta_measured.beta)) {
      MotorApp_EnterFault(MOTOR_APP_FAIL_NONFINITE);
    } else if (foc.state.vbus <= 0.1f) {
      MotorApp_EnterFault(MOTOR_APP_FAIL_BUS_INVALID);
    } else {
      MotorApp_AdvanceStateMachine(&observer_input);
    }

    /* 状态机可能已因故障回 IDLE，必须重判——否则会对着已关桥的电机
     * 继续提交参考和写比较值。 */
    if (MotorApp_IsControlState(foc_motor_state) != 0U) {
      MotorApp_ResolveControlReference();

      /* 参考复查：角度/Id/Iq 任一非有限都要在进电流环前拦下。 */
      if (!isfinite(motor_control.reference.theta_ctrl) ||
          !isfinite(motor_control.reference.id_ref) ||
          !isfinite(motor_control.reference.iq_ref)) {
        MotorApp_EnterFault(MOTOR_APP_FAIL_NONFINITE);
      } else {
        MotorApp_RunCurrentLoop(&motor_control);

        /* SVPWM 之后的值将直接写硬件，同样复查。 */
        if (!isfinite(foc.state.u_dq.d) || !isfinite(foc.state.u_dq.q) ||
            !isfinite(foc.svpwm.duty_a) || !isfinite(foc.svpwm.duty_b) ||
            !isfinite(foc.svpwm.duty_c))
          MotorApp_EnterFault(MOTOR_APP_FAIL_NONFINITE);
      }
    }

    /* 可选的接管/闭环健康闸门默认关闭，以免高速负载下把电流跟踪误差当成
     * 停机条件；有限值、母线和硬件保护路径仍独立有效。需要时将编译宏打开。 */
    if ((MOTOR_APP_CLOSED_LOOP_HEALTH_ENABLE != 0U) &&
        ((foc_motor_state == FOC_MOTOR_OBSERVER_HANDOVER) ||
         (foc_motor_state == FOC_MOTOR_CLOSED_LOOP))) {
      uint8_t unhealthy = 0U;

      /* 跟踪偏差：比较与参考的偏差而非绝对值（接管期 Id 允许非零），
       * 且用本拍刚算出的参考，避免用到过期值。 */
      if ((fabsf(foc.state.i_dq.d - motor_control.reference.id_ref) >
           MOTOR_APP_CLOSED_LOOP_MAX_ID_A) ||
          (fabsf(foc.state.i_dq.q - motor_control.iq_ref_active) >
           MOTOR_APP_CLOSED_LOOP_MAX_IQ_ERROR_A)) unhealthy = 1U;
      /* 磁链越界：低于下限=观测器失效；psi_max<=0 表示不查上限。 */
      if ((foc.observer.state.psi_mag < foc.observer.config.psi_min) ||
          ((foc.observer.config.psi_max > 0.0f) &&
           (foc.observer.state.psi_mag > foc.observer.config.psi_max))) unhealthy = 1U;
      /* 启动过渡期：观测转速（乘方向）必须高于 400 rpm——远低于接管点
       * 1000 rpm 的粗判，低于它认为拖动已经丢掉。 */
      if ((startup.speed_startup_active != 0U) &&
          (foc.observer.state.speed_rpm * startup.if_direction < 400.0f))
        unhealthy = 1U;

      /* 去抖：任一判据异常累加、全部正常清零。 */
      if (unhealthy != 0U) startup.fault_count++;
      else startup.fault_count = 0U;
      if (startup.fault_count >=
          MOTOR_APP_MS_TO_TICKS(MOTOR_APP_CLOSED_LOOP_FAULT_TIME_MS))
        MotorApp_EnterFault(MOTOR_APP_FAIL_LOST_SYNC);
    }

    /* 仍处于主动控制状态才写比较值（健康检查可能刚触发故障）；三通道
     * 经影子寄存器在下一更新事件统一加载，三相同步。 */
    if (MotorApp_IsControlState(foc_motor_state) != 0U) {
      TIM1->CCR1 = foc.svpwm.ccr_a;
      TIM1->CCR2 = foc.svpwm.ccr_b;
      TIM1->CCR3 = foc.svpwm.ccr_c;

      /* 进闭环第一拍抓一次快照（valid 守卫只抓一次），供 startdiag 回放
       * "接管刚完成时"的转速与电流状态。 */
      if ((foc_motor_state == FOC_MOTOR_CLOSED_LOOP) &&
          (motor_closed_snapshot.valid == 0U))
        MotorApp_CaptureDiagnostic(&motor_closed_snapshot, MOTOR_APP_FAIL_NONE);

      /* 最后提交状态切换：本拍已用旧状态控制量完成输出，新状态下一拍
       * 生效，避免边界拍参考跳变。 */
      MotorApp_UpdateStateTransition();
    }
  }

  /* 高频更新估算值供低频 CAN 反馈；IDLE 时函数内部同步清零。 */
  FOC_UpdateBusCurrentEstimate(&foc);

  /* ---- 波形提交（尽力而为）：总开关 / 文本发送未占用串口 / 主动控制
   * 状态 / TC=1（上一帧确已发完）四个门控任一不满足就安静跳过，
   * 不等待、不阻塞 ---- */
  if ((just_float_enabled != 0U) &&
      (motor_console_tx_active == 0U) &&
      (MotorApp_IsControlState(foc_motor_state) != 0U) &&
      ((USART1->ISR & USART_ISR_TC) != 0U)) {
    if (motor_wave_mode == MOTOR_APP_WAVE_DEFAULT) {
      /* 默认通道（尽力 25 kHz，实际受串口带宽限制）：三相电流、滤波转速、
       * 电角度、母线、原始转速（滤波/原始转速非独立测量，不能据此判定
       * 转子真实转动）。 */
      (void)MotorApp_SendJustFloat(foc.state.i_abc.a, foc.state.i_abc.b,
          foc.state.i_abc.c, foc.observer.state.speed_rpm_f,
          foc.observer.state.phase_raw * RAD_TO_DEG_F, foc.state.vbus,
          foc.observer.state.speed_rpm);
    } else if ((motor_adc_irq_count % 25U) == 0U) {
      /* 诊断通道 25 分频降到 1 kHz：启动是百毫秒量级，1 kHz 足够看趋势，
       * 串口带宽也有富余。 */
      if (motor_wave_mode == MOTOR_APP_WAVE_TIMING) {
        uint32_t hclk_hz = motor_dwt_hclk_hz;
        float time_us = (hclk_hz != 0U)
            ? ((float)motor_callback_cycles_last * 1000000.0f /
               (float)hclk_hz) : 0.0f;
        float max_time_us = (hclk_hz != 0U)
            ? ((float)motor_callback_cycles_max * 1000000.0f /
               (float)hclk_hz) : 0.0f;
        float budget_us = 1000000.0f / (float)MOTOR_APP_CONTROL_HZ;
        float load = (budget_us > 0.0f) ? time_us * 100.0f / budget_us : 0.0f;
        float max_load = (budget_us > 0.0f)
            ? max_time_us * 100.0f / budget_us : 0.0f;
        /* 保持 7-float/32-byte JustFloat 帧：当前时间、当前周期、最大时间、
         * 最大周期、周期预算、当前占用率、历史最大占用率。 */
        (void)MotorApp_SendJustFloat(time_us,
            (float)motor_callback_cycles_last, max_time_us,
            (float)motor_callback_cycles_max, budget_us, load, max_load);
      } else if (motor_wave_mode == MOTOR_APP_WAVE_STARTUP) {
        /* 启动时序：相位码、虚拟转速、观测转速、参考转速、角差、Iq 参考、
         * 观测系 Iq——iq_obs 用观测角对 alpha-beta 电流直接 Park（不经
         * 电流环）算出，用于判断控制坐标系是否对齐。 */
        FOC_SIN_COS_t sc;
        MotorApp_SinCos(foc.observer.state.phase_raw, &sc);
        float iq_obs = -foc.state.i_alpha_beta.alpha * sc.sin +
                        foc.state.i_alpha_beta.beta * sc.cos;
        (void)MotorApp_SendJustFloat((float)MotorApp_StartupPhaseCode(),
            MotorApp_VirtualRpm(), foc.observer.state.speed_rpm,
            motor_control.speed_ref_active_rpm,
            FOC_WrapToPiFast(motor_control_angle_rad - foc.observer.state.phase_raw),
            motor_control.iq_ref_active, iq_obs);
      } else if (motor_wave_mode == MOTOR_APP_WAVE_VOLTAGE) {
        /* 无协议长度变化：控制反馈(speed_rpm_f)、参考、电流请求/反馈、
         * Ud/Uq/Ulim。 */
        (void)MotorApp_SendJustFloat(foc.observer.state.speed_rpm_f,
            motor_control.speed_ref_active_rpm, motor_control.iq_ref_active,
            foc.state.i_dq.q, foc.state.u_dq.d, foc.state.u_dq.q,
            motor_control.voltage_limit);
      } else {
        /* 电流跟踪：相位码、Id 参考/实测、Iq 参考/实测、母线、就绪失败
         * 掩码（按位解读）——诊断参考与实测是否贴合、失败位在哪条判据。 */
        (void)MotorApp_SendJustFloat((float)MotorApp_StartupPhaseCode(),
            motor_control.reference.id_ref, foc.state.i_dq.d,
            motor_control.iq_ref_active, foc.state.i_dq.q, foc.state.vbus,
            (float)motor_ready_fail_mask);
      }
    }
  }

  /* 耗时统计：DWT 差值即本次回调 CPU 周期数（不含中断入口/分派/返回
   * 开销，真实占用需在此之上留余量：预算 40 us = 6800 周期）。 */
  motor_callback_cycles_last = DWT->CYCCNT - callback_cycles_start;
  if (motor_callback_cycles_last > motor_callback_cycles_max)
    motor_callback_cycles_max = motor_callback_cycles_last;
}
