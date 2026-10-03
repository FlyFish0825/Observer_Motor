/*
 * ============================================================================
 * controller.h —— 无感 FOC 串级控制器（速度环 + 双电流环）对外接口
 *
 *
 * 本模块是纯算法层，只做数学运算，不直接读写任何外设寄存器（TIM1/ADC/CORDIC
 * 等由应用层 app_motor / foc 模块负责）。因此 CubeMX 重新生成外设初始化代码
 * 时不会覆盖这里的控制逻辑。它被 FOC 电流环入口
 * （ADC1_2_IRQHandler -> MotorApp_OnInjectedConversion，25 kHz）每拍调用一次。
 *
 * 数据流总览（单位一律在字段注释中标注）：
 *   - 应用层状态机（IDLE/ALIGN/OPEN_LOOP_IF/OBSERVER_HANDOVER/CLOSED_LOOP）
 *     每拍先算好本拍控制角 theta_ctrl 和 dq 电流参考，通过
 *     FOC_Control_SubmitReference() 提交到唯一的仲裁结构 reference；
 *   - FOC_Control_Run() 消费 reference：电流环（Id/Iq 各一个 PI）每拍都跑，
 *     速度 PI 按 speed_loop_divider（默认 25）分频、即 1 kHz 跑一次；
 *   - 输出 ud_output/uq_output（V）交给应用层做逆 Park + SVPWM。
 *
 * 执行域与并发约定：
 *   - 本模块所有函数都在中断上下文（25 kHz 注入转换完成中断）里被调用，
 *     属于单一执行域；
 *   - 带 volatile 的字段是"可被通信/主循环等其它域写入"的参数，本模块只保证
 *     单字段读取的原子性，不保证多字段成组一致（成组快照由别处的 seqlock 负责）；
 *   - 不带 volatile 的字段（reference、各反馈与输出缓存）只在本执行域内读写，
 *     跨域观察时只用于显示，允许读到某一拍的中间值。
 * ============================================================================
 */

#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PI输出饱和状态
 *
 * 本枚举描述"未限幅输出 output_unsaturated 越过输出上下限的方向"，
 * 不是"输出被夹住"的绝对值判断。写入方只有两处：PI_Controller_RunError()
 * 与 PI_Controller_PreloadOutput()，它们在每次运算末尾按同一套三段比较写入。
 * 应用层 Ready、调试曲线与速度环均读取该字段；速度环用 q 轴饱和方向
 * 判断是否应暂停继续推动下游电压饱和的积分。
 * 取值范围只能是 -1/0/+1 三者之一；不会出现其它值。
 */
typedef enum {
  PI_SATURATION_NONE = 0,  /* 未饱和。判据 output_min <= output_unsaturated <= output_max，数值 0。 */
  PI_SATURATION_LOW = -1,  /* 下饱和（负向打满）。判据 output_unsaturated < output_min，数值 -1。 */
  PI_SATURATION_HIGH = 1   /* 上饱和（正向打满）。判据 output_unsaturated > output_max，数值 +1。 */
} PI_Saturation_t;

/**
 * @brief 通用PI控制器数据结构
 */
/*
 * 本结构共有三种成员，用途与并发属性完全不同，阅读时务必区分：
 *   (1) 配置段（前 8 个 volatile float）：由通信/参数下发路径在运行中改写，
 *       本控制器每次运算先整体快照到局部变量再用，避免同一次计算里出现
 *       "前半拍用旧 Kp、后半拍用新 Kp"的撕裂；
 *   (2) 历史/状态段（reference/feedback/error/proportional/integral/
 *       output_unsaturated/output/saturation）：只在中断执行域内读写，
 *       因此不加 volatile；reset/预加载会改写它们；
 *   (3) 输出段（output）既是历史又是对外的结果，跨域只用于显示。
 * 结构体按值以指针方式传给所有 PI 接口，调用方负责保证指针非空（接口内部
 * 也做了 NULL 保护，NULL 时返回 0.0f 并不写内存）。
 */
typedef struct {
  /* 可调参数：Kp为输出/输入单位，Ki为输出/(输入*s)，sample_time单位s。
   * volatile保证运行时重新读取，不保证多个参数能作为一个事务同时更新。
   */
  volatile float kp;          /* 比例增益。单位 = 输出量单位 / 输入量单位。电流环为 V/A（欧姆量纲），速度环为 A/rpm。调大加快响应但容易振荡/噪声放大，调小响应变慢。写入方：参数下发/上位机；读取方：PI_Controller_RunError 每拍快照。 */
  volatile float ki;          /* 积分增益，单位 = 输出量单位 /（输入量单位·秒）。电流环为 V/(A·s)，速度环为 A/(rpm·s)。注意本实现把它当作"连续时间增益"，每拍显式乘 sample_time 离散化（见 controller.c 的 integral_candidate 计算），所以改速度环分频时必须同步改 sample_time，否则等效积分强度会变。 */
  volatile float sample_time; /* 离散积分步长，单位 s。电流环 = 1/25000 = 0.00004 s；速度环 = current_loop_sample_time * speed_loop_divider = 0.001 s（1 kHz）。被 PI_Normalize 为负值时会钳到 0（0 使积分完全停止，等效纯比例控制）。 */

  volatile float output_min;  /* 输出下限，单位同 output。电流环由 FOC_Control_Run 按实时母线电压动态改写（dq 圆形限幅），不再使用固定 -20 V。 */
  volatile float output_max;  /* 输出上限，单位同 output。与 output_min 成对使用，PI_NormalizeLimits 保证 output_min <= output_max。 */

  volatile float integral_min; /* 积分项下限，单位同 output（因为 integral 已包含 Ki 累积，不再单独乘 Kp）。默认与 output_min 相同；电流环里 FOC_Control_Run 调用 PI_Controller_SetLimits 时会与输出限幅一起被改成动态电压限幅。 */
  volatile float integral_max; /* 积分项上限，单位同 output。抗饱和第三道保险：即使条件积分失效，integral 也不会超出该范围。 */

  /* 运行状态：integral保存已乘Ki并积分后的输出贡献，单位与output相同。 */
  float reference;  /* 上一拍（本拍）使用的参考值，输入量单位（电流环 A；速度环 rpm）。写入方：PI_Controller_Run（由参数传入）或 PI_Controller_PreloadOutput；读取方：调试观察。RunError 不写它，这是刻意的——PLL 鉴相等场景没有传统意义的目标值。 */
  float feedback;   /* 参考值对应的反馈值，输入量单位。由 PI_Controller_Run 写入（RunError 路径保持上一拍内容）。 */
  float error;      /* 本拍误差 = reference - feedback，输入量单位。RunError 路径下由调用方直接给出误差再写入此处，所以 PLL 之类场景里 reference/feedback 可能是陈旧的，只有 error 有意义。 */

  float proportional; /* 本拍比例项 = kp * error，单位同 output。单独留一份是为了让 PreloadOutput 能反算 integral，同时方便观察"P 与 I 各贡献多少"。 */
  float integral;     /* 积分项累加器，已经乘过 Ki 并累加了 sample_time，因此单位与 output 相同（历史遗留习惯常把它当"积分器的原始状态"，在别处可能被复用为非速度量，阅读时要按写入方确认）。写入方：RunError（条件积分/撤销）、PreloadOutput（反算预装）、Reset/SetLimits 等（清零或限幅）。读取方：输出合成、PreloadOutput、FOC_Control_RotateCurrentIntegrals（坐标旋转搬运）。 */

  float output_unsaturated; /* 未限幅输出 = proportional + integral（抗饱和撤销后的版本），单位同 output。保留它是为了判断饱和方向以及区分"被夹住"与"真实需求"。 */
  float output;             /* 限幅后的最终输出，单位同 output。电流环为 Ud/Uq（V），速度环为 Iq 参考（A）。跨域只读用于显示，不是每拍的唯一真值来源（真值另有 control->ud_output/uq_output 或 iq_ref_active）。 */

  PI_Saturation_t saturation; /* 最近一次运算的饱和方向；供 Ready、诊断及速度环下游抗饱和使用。 */
} PI_Controller_t;


/* ======================== 电机闭环默认参数 ======================== */

/*
 * 电流环参数采用相同0.5 ohm、100 uH电机分支中已使用的保守初值。
 * 电流环每次ADC注入转换完成时运行，当前频率25kHz。
 * 电流PI误差单位A、输出单位V。运行时按实时母线电压建立dq圆形限幅，
 * 不再使用固定的正负20 V独立轴限幅。
 *
 *
 * - PI 输入（误差）单位：A，来自 ADC 注入组采样经 Clarke/Park 得到的 id/iq 反馈；
 * - PI 输出单位：V，最终经逆 Park + SVPWM 变成三路占空比；
 * - 采样周期 0.00004 s（25 kHz）由 FOC_Control_Init 的 current_loop_sample_time
 *   实参传入，不体现在本组宏里；
 * - Kp = 0.220 V/A，Ki = 1257.0 V/(A·s)。0.5 ohm、100 uH 的电机电气时间常数
 *   约 L/R = 200 us，是开关周期 40 us 的 5 倍，属于"电流环带宽远高于被控对象"的
 *   常规配置，故取比例主导、积分中等的保守值；
 * - 调大 Kp：电流阶跃响应变快，但 PWM 死区非线性与电流采样噪声会被放大成高频振荡；
 *   调大 Ki：稳态误差收敛更快，但过大时会与电压限幅耦合产生低频摆动（甚至极限环）。
 * - 本组宏不配置任何寄存器，只是 PI_Controller_Init 的实参。
 */
#define FOC_ID_PI_KP_DEFAULT 0.220f   /* d 轴电流环比例增益，单位 V/A。写入 PI_Controller_Init 的 kp。 */
#define FOC_ID_PI_KI_DEFAULT 1257.0f  /* d 轴电流环积分增益，单位 V/(A·s)。每拍积分增量 = Ki*error*4e-5 ≈ 0.0503*error（V），即 1 A 误差约 20 拍积到 1 V。 */

#define FOC_IQ_PI_KP_DEFAULT 0.220f   /* q 轴电流环比例增益，单位 V/A。与 d 轴同值，因为当前是表贴式（Ld≈Lq）电机，两轴可用同一套整定。 */
#define FOC_IQ_PI_KI_DEFAULT 1257.0f  /* q 轴电流环积分增益，单位 V/(A·s)。与 d 轴同值；q 轴的实际输出电压还会被 FOC_Control_Run 的圆形限幅按 d 轴占用后的余量动态收紧。 */

/* 线性SVPWM最大dq矢量为Vbus/sqrt(3)，电压利用率直接用满线性区。 */
/*
 * 这两个宏只在 FOC_Control_Run 里相乘，用来把实时母线电压换算成
 * 允许的 dq 电压矢量幅值上限 voltage_limit（单位 V）：
 *     voltage_limit = dc_bus_voltage * FOC_INV_SQRT3_DEFAULT
 *                                   * FOC_VOLTAGE_UTILIZATION_DEFAULT
 * - 1/sqrt(3) ≈ 0.57735 来自线性调制的几何关系：三相桥在线性区（不进入六步
 *   过调制）能输出的最大相电压矢量幅值为 Vbus/sqrt(3)，对应 SVPWM 调制比 1.0，
 *   即最大相电压峰值 Vbus/sqrt(3)；
 * - 当前取 1.0：用满线性调制区，不进入六步过调制；死区和采样延迟由底层
 *   定时器/采样链路承担，不再由软件额外扣除 2% 电压裕量；
 * - 调低该值会主动降低可用电压和最高转速，调高超过 1.0 则会进入过调制，
 *   本工程不做后者。
 * - 二者都不对应具体寄存器，最终通过 SVPWM 的占空比寄存器体现。
 */
#define FOC_VOLTAGE_UTILIZATION_DEFAULT 1.0f    /* 电压利用率，无量纲；1.0 表示用满线性SVPWM调制区。 */
#define FOC_INV_SQRT3_DEFAULT 0.57735026919f    /* 1/√3 的常量近似值，无量纲。用于把母线电压换算成线性 SVPWM 的最大 dq 矢量幅值。 */

/*
 * 速度环输出单位为Iq参考值(A)。
 * 当前硬件电流采样：5 mOhm分流电阻、24倍模拟增益、1.65 V中点偏置、3.3 V ADC。
 * 理论双向量程约为 +/-13.75 A；10 A相电流峰值时ADC输入约为0.45~2.85 V，
 * 两端仍各保留约0.45 V裕量，可覆盖正常纹波和小幅瞬态。
 *
 * 注意：这里的+/-10 A只是速度环允许请求的Iq软件上限，不等同于硬件过流保护值。
 * 提升该上限的目的是避免水下大负载时原5 A限幅过早限制可用电磁转矩。
 */
/*
 *
 * - 输入（误差）单位：rpm（机械转速，来自总磁链观测器 SRF-PLL 的电角速度换算）；
 * - 输出单位：A，即 q 轴电流参考 Iq*，串级结构里直接成为电流环的给定；
 * - 采样周期 = 电流环周期 × FOC_SPEED_LOOP_DIVIDER_DEFAULT = 40 us × 25 = 1 ms，
 *   由 FOC_Control_Init 计算后写入 speed_pi.sample_time；
 * - 数值很小（Kp=5e-4 A/rpm）是因为量纲差异：1000 rpm 误差才产生 0.5 A 的
 *   比例输出，避免速度环一步就把电流环推到限幅；
 * - Ki = 0.005 A/(rpm·s)，每拍积分增量 = 0.005 * error * 0.001 s = 5e-6*error（A），
 *   即 1000 rpm 恒定误差约 20 ms 积到 0.1 A；实际积分还受输出和下游限幅影响。
 */
#define FOC_SPEED_PI_KP_DEFAULT 0.0005f              /* 速度环比例增益，单位 A/rpm。调大提速响应但易在观测器噪声上抖动。 */
#define FOC_SPEED_PI_KI_DEFAULT 0.005f               /* 速度环积分增益，单位 A/(rpm·s)。调大可消静差，过大易与电流限幅耦合产生低频振荡。 */
#define FOC_SPEED_PI_OUTPUT_MIN_DEFAULT (-10.0f)     /* 速度环输出下限，单位 A。负值表示允许反向电磁转矩（制动/反转），不是"不能为负"。绝对值是软件限幅，不等于硬件过流阈值。 */
#define FOC_SPEED_PI_OUTPUT_MAX_DEFAULT 10.0f        /* 速度环输出上限，单位 A。与下限对称，见上面的硬件量程推导。 */

/* 运行中的外部速度阶跃转换成斜坡，默认加速每秒最多变化20000 rpm。 */
/*
 * 该宏初始化 FOC_Control_t.speed_slew_rpm_per_s，单位 rpm/s。
 * 由 FOC_SlewSpeedReference 把外部（控制台/FDCAN）下发的阶跃 speed_ref_rpm
 * 变成斜坡，避免阶跃直接进速度 PI 造成 Iq 冲击和观测器失锁。
 * 20000 rpm/s 在 25 kHz 电流环下相当于每拍最多 0.8 rpm（该函数用电流环
 * sample_time 调用，见 FOC_Control_Run）。
 * 调大：跟随快但电流冲击大；调小：平滑但大阶跃时响应迟钝。
 * 特殊值 0：斜坡被夹为 0，参考值冻结不动（用于"保持当前转速"）。
 */
#define FOC_SPEED_REFERENCE_SLEW_RPM_PER_S_DEFAULT 20000.0f   /* 速度参考最大变化率，单位 rpm/s，取值 [0, +inf)。 */
/* 降速/反转时提高参考回落速度；Iq 仍受独立的电流变化率限制，避免把
 * 旧正向速度积分拖到目标之后。仅在没有应用层显式 speed_slew 限制时启用。 */
#define FOC_SPEED_DECEL_REFERENCE_SLEW_RPM_PER_S_DEFAULT 60000.0f

/* 25kHz电流环 / 25 = 1kHz速度环 */
/*
 * 速度环相对电流环的执行分频比，无量纲，上电写入
 * FOC_Control_t.speed_loop_divider。电流环每拍计数，计满该值才执行一次速度 PI，
 * 因此速度环频率 = 25 kHz / 25 = 1 kHz，周期 1 ms。
 * 该值同时决定速度环 PI 的 sample_time（乘法关系），所以运行时改分频必须
 * 同步调用 PI_Controller_SetSampleTime，否则积分强度会按比例偏离设计值。
 * 调大：速度环更慢、更稳但动态迟滞；调小：更接近电流环带宽，容易与电流环
 * 及观测器动态互相激励。
 */
#define FOC_SPEED_LOOP_DIVIDER_DEFAULT 25U   /* 分频比，单位"个电流环周期"，取值 >=1（代码里对 0 做了保护，0 当作 1）。 */


/**
 * @brief FOC控制参考仲裁结果：一个控制周期的唯一控制目标。
 *
 * 写入方与读取方约定：
 * - 应用层每拍只提交一次（FOC_Control_SubmitReference），其它模块不写。
 * - 控制器只读本结构，不再回读下面的外部命令字段。
 * - 启动、观测器接管和反转状态不允许修改控制器内部状态，只能在本结构里
 *   用状态自己的参考覆盖命令值。
 *
 * 控制角来源由状态机决定：ALIGN用固定定位角，I/F用虚拟角，闭环用观测器磁链角。
 *
 *
 * - 生命周期：作为 FOC_Control_t 的成员，随控制器一起静态分配，上电由
 *   FOC_Control_Init 建立初值，之后每个电流环周期被 FOC_Control_SubmitReference
 *   整体覆盖一次；
 * - 执行域：写入方是中断执行域内的应用层状态机（它在调用 FOC_Control_Run 之前
 *   先调用 SubmitReference），读取方是同域的 FOC_Control_Run，因此本结构
 *   不需要 volatile，也不存在跨域撕裂问题；
 * - 与下面 FOC_Control_t 里那批 volatile 外部命令字段的区别：那些是"人/上位机
 *   想干什么"，本结构是"控制器这一拍实际该执行什么"，状态机负责在启动/接管/
 *   反转等阶段用它覆盖命令值，从而保证启动逻辑绝不去篡改控制器内部积分状态。
 * - 三个"电流参考"字段的量纲都是 A（安培），theta_ctrl 是电角度 rad，
 *   注意电角度 = 机械角度 × 极对数。
 */
typedef struct {
  float theta_ctrl;          /* 本周期Park/逆Park使用的电角度，rad。取值通常 [0, 2π)（FOC_atan2_Fast 的输出范围），由状态机按当前状态选择来源——ALIGN 为固定定位角、OPEN_LOOP_IF 为虚拟角、OBSERVER_HANDOVER 为观测器角+渐消偏置、CLOSED_LOOP 为观测器磁链角。本模块只负责转发给应用层做 Park 变换，自身不消费。 */
  float id_ref;              /* 本周期d轴电流参考，A。闭环时通常 0 A（表贴式电机 MTPA 近似 d 轴不产生转矩）；ALIGN 阶段给正值用于把转子拉到已知电角；I/F 阶段也可能给 0。写入方：应用层状态机；读取方：FOC_Control_Run 里 id_pi 的参考。 */
  float iq_ref;              /* 本周期q轴电流参考，A。仅在 speed_loop_enable=0（电流模式/开环 I/F）时被直接当作 Iq 给定；speed_loop_enable=1 时该值被速度 PI 的输出取代，但仍被用作模式切换时的种子值。 */
  uint8_t speed_loop_enable; /* 1=速度环产生Iq参考，0=直接使用iq_ref。取值被 FOC_Control_Run 规范化成 0 或 1。该位发生跳变的那一拍会触发无扰模式切换（预装/回写速度 PI 积分），见 FOC_Control_Run。 */
  /* Application-owned startup overrides. -1 uses the normal speed slew;
   * nonnegative speed_slew_limit caps it (0 holds the seeded reference).
   * iq_slew_limit > 0 limits the applied Iq rate, in A/s.
   * 这两个字段是"应用层持有的启动期覆盖参数"，
   * 用于在启动/接管阶段临时改变斜坡强度，控制器本身不去猜测启动状态。
   * speed_slew_limit：单位 rpm/s。取 -1 表示不加限制、使用
   *   control->speed_slew_rpm_per_s 的常规斜坡；取 >=0 表示给斜坡速率设上限，
   *   控制器取"常规值与它之中的较小者"；取 0 表示把参考值钉死（斜坡速率为 0，
   *   speed_ref_active_rpm 冻结在当前值，用于保持种子转速）。
   * iq_slew_limit：单位 A/s。>0 时对实际施加的 Iq 参考（iq_ref_active）做速率
   *   限制；<=0 或非有限值时不限速，直接跟随目标。注意它与 speed_slew_limit
   *   方向相反：前者是"上限为正数才生效"，后者是"负数表示不生效"。
   * 两个字段都会被 FOC_Control_SubmitReference 和 FOC_Control_Reset 复位成
   * (-1.0f, 0.0f)，即默认"不额外限制"。
   */
  float speed_slew_limit;   /* 速度参考斜坡上限，单位 rpm/s，范围 [-1, +inf)。-1 = 使用常规斜坡；0 = 冻结当前参考；>0 = 取常规斜坡与该值的较小者。 */
  float iq_slew_limit;      /* Iq 参考斜坡上限，单位 A/s，范围 (-inf, +inf)，仅 >0 且有限时生效。用于 ALIGN->IF->闭环切换过程中限制转矩电流突加。 */
} FOC_Control_Reference_t;

/**
 * @brief FOC电流环和速度环总控制器
 *
 * 工作方式：
 * 1. speed_loop_enable=0：电流模式，iq_ref直接作为Iq给定。
 * 2. speed_loop_enable=1：速度模式，速度PI输出iq_ref_active。
 * 3. Id、Iq电流PI每个电流环周期都运行。
 * 4. 速度PI按照speed_loop_divider分频运行。
 *
 *
 * 本控制器不知道启动状态机在哪个状态，它只认 reference 里的两个信息——控制角
 * theta_ctrl 和电流参考。五种启动状态与"本拍如何决定控制角与 dq 参考"的对应：
 *   - IDLE              ：不进入 Run（或输出被应用层忽略），电流参考 0；
 *   - ALIGN             ：theta_ctrl = 固定定位电角，id_ref = 定位电流（>0），
 *                         iq_ref = 0，speed_loop_enable = 0；
 *   - OPEN_LOOP_IF      ：theta_ctrl = 虚拟角（受电角加速度限制爬升），
 *                         id_ref = 0，iq_ref = 受限的爬升电流，使能 = 0；
 *   - OBSERVER_HANDOVER ：theta_ctrl = 观测器磁链角 + 渐消偏置（偏置随每拍
 *                         递减到 0），电流参考交给速度环（使能 = 1）；
 *   - CLOSED_LOOP       ：theta_ctrl = 观测器磁链角，双闭环，使能 = 1。
 * 由于速度环只有在使能位从 0 变 1 的那一拍才做无扰预装，从 OPEN_LOOP_IF 切到
 * OBSERVER_HANDOVER 时不会产生 Iq 阶跃——这是本文件里最需要小心的一处时序。
 */
/*
 * 本结构是控制器在内存中唯一的实例（静态分配，随工程一起占用 RAM，
 * 不动态申请）。它把"参数、历史状态、调度计数、观测量"全部收在一处：
 * - 前三个 PI 是真正的运算状态；
 * - reference 是每拍的控制目标入口；
 * - id_ref/iq_ref/speed_command_rpm/speed_ref_rpm/speed_slew_rpm_per_s/
 *   speed_loop_enable 这六个 volatile 字段是可被通信/主循环域改写的外部命令，
 *   本模块只在初始化、模式切换回写和斜坡计算中读取，绝不直接进 PI；
 * - 其余非 volatile 字段只在 25 kHz 中断域内读写，跨域观察（Live Watch/上位机）
 *   时可能看到同一拍内的中间值，这是允许的。
 */
typedef struct {
  /* 三个PI控制器 */
  PI_Controller_t id_pi;    /* d 轴电流环。输入 A、输出 V，25 kHz 每拍运行；限幅每拍由 FOC_Control_Run 按实时母线电压设成 ±voltage_limit。 */
  PI_Controller_t iq_pi;    /* q 轴电流环。输入 A、输出 V，25 kHz 每拍运行；限幅每拍被设成 ±sqrt(voltage_limit²-ud²)，即 d 轴占用后的圆形余量。 */
  PI_Controller_t speed_pi; /* 速度环。输入 rpm、输出 A（Iq 参考），按 speed_loop_divider 分频运行（默认 1 kHz）；限幅为 ±10 A（FOC_SPEED_PI_OUTPUT_*_DEFAULT）。 */

  /* 控制参考唯一入口：应用层仲裁后提交，控制器只读。 */
  FOC_Control_Reference_t reference; /* 见上面的结构说明。本模块只读它，写它的唯一函数是 FOC_Control_SubmitReference（以及 Init/Reset 里的初值设定）。 */

  /* 外部命令，可由串口实时修改；只作为仲裁输入，不直接进入PI。
   * id_ref/iq_ref单位A；speed_command_rpm为控制台目标，应用层复制给
   * speed_ref_rpm，控制器再生成speed_ref_active_rpm作为实际PI参考。
   */
  volatile float id_ref;   /* 外部（控制台/FDCAN/上位机）下发的 d 轴电流命令，单位 A。volatile：可被主循环通信域改写。上电默认 0.0 A。注意它只是仲裁输入：正常运行时实际使用的 d 轴参考来自 reference.id_ref。 */
  volatile float iq_ref;   /* 外部下发的 q 轴电流命令，单位 A，上电默认 0.20 A（保留一点转矩电流用于建立可观测的端电压）。volatile：可被通信域改写。它是电流模式下的给定种子，也是退出速度模式时被回写的地方（见 FOC_Control_Run 的离开速度模式分支）。 */
  volatile float speed_command_rpm; /* 控制台/上位机下发的目标转速，单位 rpm，上电默认 1500.0 rpm。volatile：可被通信域改写。应用层负责把它复制到 speed_ref_rpm；控制器本身不读它。 */
  volatile float speed_ref_rpm;     /* 经应用层确认后的速度参考，单位 rpm，上电默认 1500.0 rpm。volatile：可被通信域改写。速度模式下它作为斜坡的终点（FOC_SlewSpeedReference 的 target），斜坡结果存 speed_ref_active_rpm。 */
  volatile float speed_slew_rpm_per_s; /* 常规速度参考斜坡速率上限，单位 rpm/s，上电取 FOC_SPEED_REFERENCE_SLEW_RPM_PER_S_DEFAULT（20000 rpm/s）。volatile：可被通信域改写。FOC_Control_Run 会再与 reference.speed_slew_limit 取小，得到本拍实际使用的斜率。 */
  volatile uint32_t speed_loop_enable; /* 速度环使能"请求"，0=电流模式，非0=速度模式，上电默认 1U。volatile：可被 FOC_Control_EnableSpeedLoop（通信域）改写。它只是请求，真正的仲裁结果看 reference.speed_loop_enable，两者跳变时由 FOC_Control_Run 做无扰切换。 */

  /* 调度参数和内部状态；修改分频时须同步速度PI的sample_time。 */
  uint32_t speed_loop_enable_last; /* 上一拍生效的仲裁使能状态（规范化后的 0/1），用于检测 reference.speed_loop_enable 的边沿并触发一次无扰模式切换。上电由 Init 置 0，使第一次进入速度模式时必然走一遍预装分支。 */
  uint16_t speed_loop_divider;     /* 速度环分频比，单位"个电流环周期"，上电 = FOC_SPEED_LOOP_DIVIDER_DEFAULT（25），即 1 kHz。运行时要改它，必须同时调用 PI_Controller_SetSampleTime(&speed_pi, 电流环周期*新分频)，否则速度 PI 的等效积分强度会偏离设计。 */
  uint16_t speed_loop_counter;     /* 速度环分频计数器，每个电流环周期 +1，达到 speed_loop_divider 时归零并执行一次速度 PI。范围 [0, speed_loop_divider-1]；模式切换与 Reset 时被清零（保证切换后立刻获得一个完整的 1 ms 窗口）。 */

  /* 反馈量，便于Live Watch和上位机观察 */
  float id_feedback;         /* 最近一拍的 d 轴电流反馈，单位 A，由 FOC_Control_Run 的实参写入。仅作为回显/调试量，PI 内部另有自己的 feedback 字段。 */
  float iq_feedback;         /* 最近一拍的 q 轴电流反馈，单位 A，来源同上。 */
  float speed_feedback_rpm;  /* 最近一拍的转速反馈，单位 rpm（机械转速），来自观测器 SRF-PLL 的电角速度换算。它是速度 PI 的反馈，也是进入速度模式时 speed_ref_active_rpm 的种子值，使接管瞬间不会产生转速阶跃。 */

  /* 速度斜坡后的内部参考，仅在正常速度闭环中使用 */
  float speed_ref_active_rpm; /* 斜坡平滑后的内部转速参考，单位 rpm。速度模式下每拍由 FOC_SlewSpeedReference 朝终点 speed_ref_rpm 逼近；进入速度模式时可被重置为当前反馈转速（无扰接管）。电流模式下该字段不被斜坡更新。 */
  float speed_ref_previous_rpm; /* 上一拍实际使用的速度目标；用于只在命令变化时撤销减速方向相反的推进历史。初始化、复位和进入速度模式时播种，恒定命令不重复重置。 */

  /* 速度环最终产生的有效Iq参考值 */
  float iq_ref_active; /* 实际送入 q 轴电流 PI 的 Iq 参考，单位 A。它是"速度 PI 输出（或电流模式的命令值）经 iq_slew_limit 限速后"的结果，是整条链路上唯一的 Iq 真值。写出方：FOC_Control_Run 每拍末尾；读取方：iq_pi 的参考、模式切换时的种子、以及调试观察。 */
  float iq_ref_target; /* Rate-limited speed PI / direct-current target. */ /* 本拍 Iq 的目标值（限速之前），单位 A。速度模式下 = 速度 PI 输出（若 iq_slew_limit 生效且下游限速真实起作用，则被改写成"可达值"以避免速度积分器继续累积）；电流模式下 = reference.iq_ref 或退出速度模式时的回写值。上电初值等于 iq_ref。 */

  /* 电流环输出电压 */
  float ud_output; /* d 轴电流 PI 限幅后的输出，单位 V。本模块只算数值，由应用层拿去逆 Park 并送给 SVPWM；它是诊断的直流分量（旋转坐标系下），不是相电压。 */
  float uq_output; /* q 轴电流 PI 限幅后的输出，单位 V。与 ud_output 共同构成 dq 电压矢量，其幅值被约束在 voltage_limit 之内。 */

  /* 根据实时母线电压得到的dq电压矢量上限，单位V。 */
  float voltage_limit; /* 本拍允许的 dq 电压矢量最大幅值，单位 V = max(0, Vbus) / sqrt(3)。由 FOC_Control_Run 每拍根据实参 dc_bus_voltage 重算；母线电压非有限值或 <=0 时置 0（此时两轴电压都会被限成 0，电流环失去输出能力，是一种故障态的表现）。 */
  uint8_t speed_voltage_limited; /* 最近一次速度 PI 因下游电压饱和暂停同向积分。 */
} FOC_Control_t;

/* ======================== 通用PI接口 ======================== */

/**
 * @brief 初始化PI控制器并清零运行状态。
 * @param pi          PI控制器结构体指针。
 * @param kp          比例增益，单位 = 输出单位/输入单位。
 * @param ki          积分增益，单位 = 输出单位/(输入单位·s)。
 * @param sample_time 离散积分使用的采样周期（s）。
 * @param output_min  输出下限（同时作为积分下限默认值）。
 * @param output_max  输出上限（同时作为积分上限默认值）。
 *
 * 本函数把配置写进结构体后调用 PI_Controller_Reset 清零全部历史，
 * 因此不能用于"运行中重新整定"（会丢掉积分）。运行中改增益请用
 * PI_Controller_SetGains，改周期用 PI_Controller_SetSampleTime，改限幅用
 * PI_Controller_SetLimits 系列。
 * 边界行为：pi 为 NULL 直接返回（不解引用）；sample_time < 0 被钳为 0；
 * output_min/output_max 顺序颠倒会由 PI_NormalizeLimits 自动交换。
 * 前置条件：pi 指向一块已分配、尚未被使用的 FOC 控制器成员。
 * 后置条件：结构体所有字段均为确定值；saturation = PI_SATURATION_NONE。
 * 调用时机/调用者：上电初始化阶段由 FOC_Control_Init 调用三次（id/iq/speed），
 * 之后不再被运动控制路径调用。
 */
void PI_Controller_Init(PI_Controller_t *pi, float kp, float ki,
                        float sample_time, float output_min, float output_max);

/**
 * @brief 给定参考和反馈值，计算一次PI输出。
 * @param pi        PI控制器指针。
 * @param reference 参考值（目标）。
 * @param feedback  反馈值（实测）。
 * @return 限幅后的PI输出。
 *
 * 本函数只做两件事——把 reference/feedback 存进结构体（供调试观察和
 * PreloadOutput 使用），然后用 reference-feedback 调用 PI_Controller_RunError。
 * 全部积分与抗饱和逻辑都在 RunError 里，此处不重复。
 * 边界：pi 为 NULL 时返回 0.0f 且不写任何内存；误差含 NaN 时输出行为由浮点
 * 比较决定（比较为假则视为不饱和），调用方负责保证反馈值已做有效性检查。
 * 调用者/频率：FOC_Control_Run 每拍调用两次（25 kHz），分别对应 id_pi 和 iq_pi；
 * 速度环每 25 拍一次。它不在中断以外被调用。
 */
float PI_Controller_Run(PI_Controller_t *pi, float reference, float feedback);

/**
 * @brief 直接使用误差值计算一次PI输出（适用于PLL等已计算好误差的场景）。
 * @param pi     PI控制器指针。
 * @param error  本拍误差（= reference - feedback）。
 * @return 限幅后的PI输出。
 *
 * 单位随控制器而定（电流环 A、速度环 rpm）。设计这一入口是为了
 * 支持"误差天然存在、没有传统参考值的控制器"——典型场景是观测器里的
 * SRF-PLL 鉴相器，它直接给出角度误差，用不到 reference/feedback 这一对字段。
 * 本函数不写 pi->reference / pi->feedback（它们保持上一拍内容，可能陈旧），
 * 只写 pi->error 及其后所有输出相关字段。
 * 边界与饱和行为：先算积分候选并夹到 [integral_min, integral_max]，再合成
 * 未限幅输出；若未限幅输出已越过输出限幅且误差仍朝同一方向推（误差符号与
 * 越界方向一致），则撤回本拍积分（integral 保持上一拍）并重新合成输出；
 * 反向误差仍允许积分，从而使控制器能自行退出饱和。
 * 注意本函数内的抗饱和只看单个 PI 的输出限幅，不感知后级 SVPWM 的电压缩放。
 * 前置条件：pi 非空且已初始化。后置条件：pi->output 一定是限幅后的值，
 * pi->saturation 反映本拍未限幅输出的越界方向。
 * 调用者：PI_Controller_Run（常规路径）以及观测器 PLL 等直接传误差的调用点。
 */
float PI_Controller_RunError(PI_Controller_t *pi, float error);

/**
 * @brief 清除PI运行状态（误差、积分、输出），保留增益和限幅配置。
 * @param pi PI控制器指针。
 *
 * 清零 reference、feedback、error、proportional、integral、
 * output_unsaturated、output 并把 saturation 复位为 NONE；kp、ki、sample_time
 * 和四个限幅字段全部保留。
 * 副作用：积分被清零意味着下一拍输出只取决于 Kp*error，因此本函数只能在
 * "允许输出跳变"的时刻调用（上电初始化、故障复位、退出闭环）。在运行中调用
 * 会造成 Iq/Ud/Uq 突变，进而导致电流冲击或观测器失步。
 * 边界：pi 为 NULL 时直接返回。
 * 调用者：PI_Controller_Init（初始化时）、FOC_Control_Reset（三个 PI 一起清）。
 * 注意它与 PI_Controller_PreloadOutput 的区别：Reset 是"归零"，PreloadOutput
 * 是"按期望输出反算积分"，后者才是模式切换时应使用的无扰手段。
 */
void PI_Controller_Reset(PI_Controller_t *pi);

/**
 * @brief 运行时更新比例和积分增益，不改变积分历史。
 * @param pi   PI控制器指针。
 * @param kp   新的比例增益。
 * @param ki   新的积分增益。
 *
 * 这是热更新整定的推荐入口——积分历史 integral 原样保留，所以改
 * 增益不会引起输出跳变（下一拍的比例项变化除外）。单位必须与初始化时一致
 * （Kp 为输出/输入，Ki 为输出/(输入·s)），本函数不做任何量纲检查，也不做
 * 符号或有限性校验；填入 NaN/Inf 会污染整个控制器状态，调用方（参数下发路径）
 * 必须自行校验。边界：pi 为 NULL 时直接返回。
 */
void PI_Controller_SetGains(PI_Controller_t *pi, float kp, float ki);

/**
 * @brief 运行时更新离散积分的采样周期。
 * @param pi          PI控制器指针。
 * @param sample_time 新的采样周期（s），负值被钳为0。
 *
 * 单位 s。本实现的积分增量是 Ki*error*sample_time，所以该值直接等于
 * 积分强度的缩放系数：把它改大一倍等于把等效 Ki 翻倍，反之亦然。
 * 钳位行为：负值被钳到 0.0f，而 sample_time = 0 会让积分项完全停止增长
 * （控制器退化成纯比例 P），这是有意保留的"冻结积分"手段。
 * 常见误用：改了 FOC_Control_t.speed_loop_divider 却忘了同步调用本函数，
 * 结果速度环积分强度按分频比成比例偏离设计值。
 * 边界：pi 为 NULL 时直接返回；NaN 不会被钳位（比较为假），会直接写进结构体。
 */
void PI_Controller_SetSampleTime(PI_Controller_t *pi, float sample_time);

/**
 * @brief 同时设置输出和积分限幅，并立即将已有状态修正到新范围。
 * @param pi      PI控制器指针。
 * @param minimum 新的下限（自动与maximum交换若顺序颠倒）。
 * @param maximum 新的上限。
 *
 * 一次性把输出限幅与积分限幅都改成同一个区间（两者共用是为了简化
 * 抗饱和推理：积分限幅不小于输出限幅时，积分单独不会造成不可恢复的饱和）。
 * 同时会把已存在的 integral 与 output 立刻夹进新区间，所以当新限幅比旧限幅窄
 * 时可能引起输出跳变——这是限幅收窄的必然结果，属于设计行为。
 * 单位同 output。minimum/maximum 顺序颠倒会由 PI_NormalizeLimits 交换。
 * 调用者：FOC_Control_Run 每个电流环周期对 id_pi、iq_pi 各调用一次，用实时
 * 母线电压算出的电压限幅覆盖上一次的值。边界：pi 为 NULL 时直接返回。
 */
void PI_Controller_SetLimits(PI_Controller_t *pi, float minimum, float maximum);

/**
 * @brief 仅设置PI输出限幅；积分限幅保持不变。
 * @param pi      PI控制器指针。
 * @param minimum 输出下限。
 * @param maximum 输出上限。
 *
 * 单位同 output。只改 output_min/output_max，并把当前 output 夹进
 * 新区间；integral_min/integral_max 与 integral 都不动。适合"只想限制最终电压、
 * 不想动积分器"的场合。若把输出限幅设得比积分限幅窄，条件积分抗饱和仍然能
 * 防止积分继续朝越界方向累积（RunError 判断的是输出越界 + 误差同向）。
 * 边界：pi 为 NULL 时直接返回；顺序颠倒会自动交换。
 */
void PI_Controller_SetOutputLimits(PI_Controller_t *pi, float minimum,
                                   float maximum);

/**
 * @brief 仅设置积分项限幅，并将当前积分值夹到新范围。
 * @param pi      PI控制器指针。
 * @param minimum 积分下限。
 * @param maximum 积分上限。
 *
 * 单位同 output（因为 integral 已包含 Ki 与 sample_time 的累积）。
 * 只改 integral_min/integral_max，并把当前 integral 夹进新区间；输出限幅不变。
 * 典型用途：把积分限幅收得比输出限幅更窄，使控制器在稳态时保留一点"积分余量"，
 * 从而在参考微调时仍能快速响应；或把积分限幅设成 0 来禁掉积分作用。
 * 边界：pi 为 NULL 时直接返回；顺序颠倒会自动交换。
 */
void PI_Controller_SetIntegralLimits(PI_Controller_t *pi, float minimum,
                                     float maximum);

/**
 * @brief 按期望输出反算积分状态，用于模式切换时无扰预加载。
 * @param pi             PI控制器指针。
 * @param desired_output 期望本拍输出的值（将被限幅）。
 * @param reference     当前参考值，用于计算比例项。
 * @param feedback      当前反馈值，用于计算比例项。
 *
 * 数学关系是 desired_output = Kp*error + integral，所以反算
 * integral = desired_output - Kp*(reference-feedback)，再夹进积分限幅，
 * 最后把 output/proportional/output_unsaturated/saturation 一并更新成
 * 与"接下来真的跑一拍"一致的状态。这样下一次 Run 的输出就从 desired_output
 * 附近连续演化，不产生阶跃。
 * 单位：三个参数都与该 PI 的输入/输出量纲配合（速度环中 desired_output 是
 * 电流 A，reference/feedback 是转速 rpm，Kp 是 A/rpm）。
 * 重要限制：若反算出的 integral 超过 integral 限幅，则只能保持"限幅后可达的
 * 输出"，此时实际输出会小于（或大于）desired_output，即无扰预装是不完整的。
 * 调用时机：只在模式切换的那一拍调用——FOC_Control_Run 里进入速度模式时用
 * 当前 iq_ref_active 预装 speed_pi；以及 Iq 下游限速生效时用"可达值"回装，
 * 防止速度积分器继续累积形成积分饱和（windup）。
 * 边界：pi 为 NULL 时直接返回；desired_output 含 NaN 会原样写进状态。
 */
void PI_Controller_PreloadOutput(PI_Controller_t *pi, float desired_output,
                                 float reference, float feedback);

/* ======================== FOC电流环/速度环接口 ======================== */

/**
 * @brief 初始化电流环和速度环
 * @param current_loop_sample_time 电流环周期，当前工程传0.00004f
 *
 * 单位 s。本函数是控制器的唯一初始化入口，调用后结构体所有字段都是
 * 确定值，可以安全地开始被 25 kHz 中断调用。
 * 具体动作：设置 speed_loop_divider = 25；算速度环周期 = 电流环周期×25；
 * 用默认宏初始化三个 PI；把外部命令字段设成上电默认（id_ref=0、iq_ref=0.20 A、
 * speed_command_rpm=speed_ref_rpm=1500 rpm、斜坡 20000 rpm/s、使能=1）；
 * reference 也按"速度模式 + 电流种子值"建立初值；speed_loop_enable_last 置 0
 * 以便第一次运行必然触发一次无扰接管。
 * 边界：control 为 NULL 直接返回；current_loop_sample_time <= 0 时回退到
 * 0.00004f（25 kHz），避免把 0 传进积分器导致积分永久不增长。
 * 前置条件：HAL/时钟已就绪；后置条件：voltage_limit = 0（等第一拍 Run 用实测
 * 母线电压重算）。
 * 调用者：应用层上电初始化，仅一次；不在运行中调用（会丢积分）。
 */
void FOC_Control_Init(FOC_Control_t *control, float current_loop_sample_time);

/**
 * @brief 清空三个PI的运行状态，不修改Kp、Ki和命令值
 *
 * 把 id_pi/iq_pi/speed_pi 的积分与输出全部归零，分频计数器清零，
 * speed_loop_enable_last 同步为当前仲裁使能（避免复位后立刻触发一次预装），
 * 各反馈与输出缓存归零，speed_ref_active_rpm 重新对齐 speed_ref_rpm，
 * iq_ref_active/iq_ref_target 重置为 iq_ref，并把两个斜坡覆盖参数复位成
 * (-1.0f, 0.0f)。
 * 使用场景：故障清除后重新起飞、通信超时停车后重新使能。因为积分被清零，
 * 调用后 Uq 会从 0 附近重新建立，应在电机确实停稳、PWM 已关闭或电流为零时调用。
 * 边界：control 为 NULL 直接返回。Kp/Ki/sample_time/限幅与外部命令值都不动。
 */
void FOC_Control_Reset(FOC_Control_t *control);

/* Transport only the additional handover-frame rotation, NOT normal rotor
 * rotation. sin_delta/cos_delta describe old offset minus new offset.
 * 本函数只搬运"接管过程中因坐标系切换而额外多出来的那一次旋转"，不搬运转子
 * 本身的正常旋转——每拍角度前进造成的变化已经在 dq 电流 PI 的正常闭环里被
 * 电流环修正，若在这里再乘一次就会把积分状态转错。
 * sin_delta/cos_delta 描述的是"旧偏置减去新偏置"这个角度差的三角函数值：
 * 即从切换前的等效控制角到切换后的控制角之间的差值 Δ。
 * 数学动作：把 (id_pi.integral, iq_pi.integral) 视为 dq 平面上的一个矢量，
 * 用标准旋转矩阵把它转到新坐标系（源码即按此实现）：
 *     d' = d*cosΔ - q*sinΔ
 *     q' = d*sinΔ + q*cosΔ
 * 注意两个积分器是"同时"用旧值计算的，源码先把旧 d、q 读到局部变量再写回，
 * 因此不存在"先更新 d 再用新 d 算 q"的顺序依赖。搬运后 (d,q) 的幅值不变，
 * 只有相位偏移 Δ，这正是"电压矢量在空间中的方向不变、只是换了描述坐标系"。
 * 单位：integral 是电压量（V），旋转本身无量纲；sin/cos 应满足 sin²+cos²=1，
 * 但函数不校验这一点。
 * 边界：control 为 NULL、或任一角函数值非有限（NaN/Inf）时直接返回、不改状态，
 * 这是为了不在观测器失步的情况下破坏积分器。
 * 调用时机/调用者：应用层在 OBSERVER_HANDOVER 切换偏置的那一拍、调用
 * FOC_Control_Run 之前调用一次。调用后 FOC_Control_Run 会在同一拍用实时母线
 * 电压重设限幅（SetLimits 会把旋转后的积分夹回允许区间）。
 */
void FOC_Control_RotateCurrentIntegrals(FOC_Control_t *control,
                                       float sin_delta, float cos_delta);

/**
 * @brief 提交本周期的控制参考，是写入reference的唯一接口。
 *
 * 应用层状态机每拍调用一次：先按当前状态决定控制角和dq电流参考，
 * 再交给FOC_Control_Run消费。控制器内部不再从外部命令字段推断参考。
 *
 * @param control           FOC总控制器指针。
 * @param theta_ctrl        本周期控制电角度（rad）。
 * @param id_ref            d轴电流参考（A）。
 * @param iq_ref            q轴电流参考（A）。
 * @param speed_loop_enable 0=电流模式直接使用iq_ref；非0=速度模式。
 *
 * 单位——theta_ctrl 为电角度 rad，id_ref/iq_ref 为 A。
 * 本函数用实参整体覆盖 reference 的四个字段，并把两个斜坡覆盖参数复位为
 * (-1.0f, 0.0f)，也就是说：**应用层若想使用有限斜坡，必须在调用本函数之后
 * 再自行写 reference.speed_slew_limit / reference.iq_slew_limit**（启动阶段
 * 就是这样做的）。这个"先清后写"的顺序保证了漏写时退化为常规斜坡，而不是
 * 沿用上一次启动阶段的限制。
 * speed_loop_enable 会被规范化成 0 或 1，非 0 一律当 1。
 * 前置条件：应在 FOC_Control_Run 之前、同一拍内调用；每拍调用一次。
 * 边界：control 为 NULL 时直接返回（此时本拍 Run 会读到上一拍的参考值）。
 * 后置条件：控制器内部状态（积分等）完全不受影响——这是刻意的设计约束：
 * 启动/接管/反转状态只能通过本函数改变控制目标，不能直接改 PI 状态。
 */
void FOC_Control_SubmitReference(FOC_Control_t *control, float theta_ctrl,
                                 float id_ref, float iq_ref,
                                 uint8_t speed_loop_enable);

/**
 * @brief 每个电流环周期调用一次
 *
 * 速度模式下，函数内部自动按speed_loop_divider运行速度PI；
 * 电流模式下，直接使用id_ref和iq_ref。
 *
 * @param control           FOC总控制器指针。
 * @param id_feedback       d轴电流反馈（A）。
 * @param iq_feedback       q轴电流反馈（A）。
 * @param speed_feedback_rpm 转速反馈（rpm），来自观测器PLL。
 * @param dc_bus_voltage    当前直流母线电压（V），用于电压矢量限幅。
 * @param ud_output         [out] d轴电压输出（V），可为NULL。
 * @param uq_output         [out] q轴电压输出（V），可为NULL。
 *
 *
 * 1) 记录反馈：id_feedback/iq_feedback（A）、speed_feedback_rpm（rpm）写进结构体；
 * 2) 读仲裁使能：只看 reference.speed_loop_enable，与 speed_loop_enable_last
 *    比较；不相等说明模式发生跳变，本拍执行一次无扰切换：计数器清零、
 *    iq_ref_target 从 iq_ref_active 续起；进入速度模式时把
 *    speed_ref_active_rpm 设为当前反馈转速（避免转速阶跃）并用当前
 *    iq_ref_active 预装速度 PI；退出速度模式时把速度 PI 刚刚产出的
 *    iq_ref_active 回写到 iq_ref，使电流模式从当前的 Iq 连续接管；
 * 3) 速度模式：用 FOC_SlewSpeedReference 平滑 speed_ref_active_rpm，计数器
 *    累加，计满 speed_loop_divider 才跑一次速度 PI；若 reference.iq_slew_limit>0，
 *    再把 PI 输出经同一条斜坡函数限速成"可达值"，并在限速真实起作用时用该
 *    可达值回装速度 PI（防止积分饱和到不可实现的 Iq）；电流模式：iq_ref_target
 *    直接取 reference.iq_ref（或退出速度模式时的回写值）；
 * 4) Iq 限速：把 iq_ref_target 经 iq_slew_limit（单位 A/s，用电流环周期作为
 *    步长）平滑成 iq_ref_active，这是最终送进 q 轴 PI 的值；
 * 5) 电压限幅：voltage_limit = Vbus/sqrt(3)（Vbus 非有限或 <=0 时置 0）；
 * 6) 电流环：先给 id_pi 设限幅 ±voltage_limit 并运行（优先保证 d 轴），再用
 *    sqrt(voltage_limit²-ud²) 作为 q_pi 的限幅（圆形限幅，两个轴合起来不超过
 *    voltage_limit），最后运行 iq_pi；
 * 7) 通过指针回传 ud/uq（指针可为 NULL，此时只更新结构体）。
 * 调用频率：25 kHz（ADC 注入转换完成中断），因此本函数必须是常数时间、
 * 无阻塞、不调用任何 HAL 阻塞接口。函数内部只用栈变量和结构体字段，不动态分配。
 * 边界：control 为 NULL 时把两个输出指针写成 0.0f 并返回（安全失效）；
 * speed_loop_divider 为 0 时按 1 处理（等价于速度环 25 kHz，仅防御性保护）。
 * 后置条件：ud_output/uq_output/voltage_limit/iq_ref_active 等字段均为本拍值；
 * speed_loop_enable_last 已同步为本拍使能。
 */
void FOC_Control_Run(FOC_Control_t *control, float id_feedback,
                     float iq_feedback, float speed_feedback_rpm,
                     float dc_bus_voltage, float *ud_output,
                     float *uq_output);

/**
 * @brief 切换电流模式/速度模式
 * @param enable 0=电流模式，非0=速度模式
 *
 * 本函数只写"请求"字段 control->speed_loop_enable（volatile，
 * 可由通信/主循环域调用），规范化成 0 或 1。真正的模式切换动作（预装/回写 PI、
 * 清零分频计数器）延迟到下一次 FOC_Control_Run 检测到 reference.speed_loop_enable
 * 与 speed_loop_enable_last 不一致时才执行。
 * 因此调用本函数之后并不会立刻改变控制行为：还需要应用层在仲裁时把新的使能
 * 状态通过 FOC_Control_SubmitReference 提交到 reference，控制器才会响应。
 * 边界：control 为 NULL 时直接返回。
 */
void FOC_Control_EnableSpeedLoop(FOC_Control_t *control, uint8_t enable);


#ifdef __cplusplus
}
#endif

#endif
