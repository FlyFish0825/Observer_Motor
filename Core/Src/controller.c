/**
 * @file controller.c
 * @brief 位置式PI和FOC串级控制：速度PI给出Iq参考，电流PI给出Ud/Uq。
 * Ki使用连续时间增益，离散积分在每次运算时显式乘sample_time。
 * 电流环按40us运行，速度环默认25分频，即1ms运行一次。
 *
 *
 *   TIM1 中心对齐互补 PWM 25 kHz（ARR=3399）-> CH4 触发 ADC1 注入组 ->
 *   JEOS 产生 ADC1_2 中断 -> HAL_ADCEx_InjectedConvCpltCallback ->
 *   MotorApp_OnInjectedConversion -> 本文件的 FOC_Control_Run（25 kHz 电流环）。
 *   速度环只是电流环的 1/25 分频，不占用独立定时器。
 * 执行域：本文件所有函数都在中断上下文运行（唯一例外是上电初始化的
 * FOC_Control_Init）。因此这里不出现任何阻塞调用、浮点除法以外的耗时运算，
 * 唯一的外部依赖是 CMSIS-DSP 的 arm_sqrt_f32（Cortex-M4F 上由 VSQRT.F32
 * 单指令完成）。
 * 数值类型：全部使用单精度 float（Cortex-M4F 的 FPU 只对单精度有硬件加速），
 * 角度用弧度而不是定点；三角函数由应用层用 CORDIC 或 FOC_atan2_Fast 提供，
 * 本文件不直接调用三角函数。
 *
 * 计数器与角度单位约定（阅读本文件时必须分清）：
 *   - theta_ctrl 是"电角度"，单位 rad，机械角度 = 电角度 / 极对数；
 *   - speed_feedback / speed_ref_rpm 是"机械转速"，单位 rpm；
 *   - 所有电流单位 A，所有电压单位 V，时间单位 s，角度斜坡单位 rpm/s 或 A/s。
 */
#include "controller.h"

#include "arm_math.h"   /* CMSIS-DSP：本文件只用 arm_sqrt_f32 做 dq 圆形限幅的 q 轴余量开方，返回 ARM_MATH_SUCCESS 才认为结果有效。 */
#include <stddef.h>     /* 提供 NULL，用于所有对外接口的指针合法性检查。 */

/*
 * 本文件实现电流环、速度环 PI 控制器以及两环之间的调度。
 * 这里仅保存控制算法状态和计算结果；外设寄存器的读写由应用层完成，
 * 因此 CubeMX 重新生成初始化文件时不会覆盖本文件中的控制逻辑。
 *
 * 本文件不含任何 volatile 写-写竞争保护、也不加锁，理由是：
 * 控制算法状态只在 25 kHz 中断域内被读写；跨域可写的参数（Kp/Ki/限幅/命令
 * 值）在每次运算开始时被"快照"到局部变量，从而保证一次计算内部自洽。这不是
 * 事务级一致性（改 Kp 的同时改 Ki 可能出现一拍混用新旧值），但对该控制对象
 * 完全够用，因为参数下发远比 40 us 慢。
 */

/**
 * @brief 浮点限幅
 *
 * 把 value 夹到闭区间 [minimum, maximum] 内并返回结果。
 * @param value   待限幅的量，单位任意，但必须与 minimum/maximum 同量纲。
 * @param minimum 下限；@param maximum 上限。本函数**假定 minimum <= maximum**，
 *        不做顺序交换（需要交换的场合由调用者先调用 PI_NormalizeLimits）。
 *        若调用者传反了，行为是"先判 value>maximum 返回 maximum"，结果不可预期。
 * @return 限幅后的值：NaN 输入会原样返回（两个比较都为假，直接落到 return value），
 *         这是有意的——限幅不能悄悄吞掉非有限值，好让上游的有效性检查发现它。
 * 使用范围：static inline，仅本文件可见，被 PI_Controller_RunError、
 *         PI_Controller_PreloadOutput、PI_Controller_SetLimits 系列以及
 *         FOC_Control_Run 的电压限幅计算使用。
 * 性能：在中断里每拍最多调用几次，分支预测友好（绝大多数情况落在 return value）。
 */
static inline float PI_Clamp(float value, float minimum, float maximum) {
  if (value > maximum) {
    return maximum;
  }

  if (value < minimum) {
    return minimum;
  }

  return value;
}

/**
 * @brief 按最大变化率平滑速度参考，避免外部阶跃直接进入速度PI。
 *
 *
 *   (1) 平滑转速参考：current/target 单位 rpm，slew 单位 rpm/s；
 *   (2) 平滑 Iq 参考（FOC_Control_Run 里的 iq_slew_limit 分支）：
 *       current/target 单位 A，slew 单位 A/s，函数本身只做"线性斜坡"这件事，
 *       与物理量无关，所以可以直接复用同一份代码。
 * @param current        当前（上一拍）值，单位随用途 rpm 或 A。
 * @param target         本拍目标值，单位同上。
 * @param slew_rpm_per_s 最大变化率，单位 "单位/秒"（rpm/s 或 A/s），必须 >= 0。
 * @param sample_time    本次调用对应的步长，单位 s。速度环用电流环周期（40 us）
 *                       调用，所以虽然速度 PI 是 1 kHz，斜坡仍按 25 kHz 细化，
 *                       阶跃被摊成 25 段/ms 的细爬升，避免速度 PI 每毫秒吃到大跳变。
 * @return 新的当前值：向 target 逼近了最多 slew*sample_time 的距离；input 异常时
 *         返回 0.0f，步长非法时返回 current（原地不动）。
 * 边界与容错（顺序很重要，逐条对应源码）：
 *   - current 或 target 非有限（NaN/Inf）-> 返回 0.0f（把参考拉回零点，属于
 *     故障回退；这里选择 0 而不是 current，是为了避免把 NaN 传染给速度 PI）；
 *   - slew 非有限或为负 -> 当作 0（不爬升）；**特别地 slew=0 会让参考冻结在
 *     current**，这就是 reference.speed_slew_limit = 0 用来"钉住当前转速"的机制；
 *   - sample_time 非有限或 <= 0 -> 直接返回 current（无事发生，而不是除零）。
 * 为什么用"限幅 delta"而不是"按比例插值"：限幅是纯线性斜坡，速度 PI 在每一小步
 * 上看到的都是小误差，积分器不会因为一次大跳变而冲到限幅；按比例插值反而会在
 * 步长变化时改变等效加速度。
 * 调用者：FOC_Control_Run（速度参考斜坡、Iq 目标斜坡）共两处；纯函数，无副作用，
 * 不读写任何全局状态。执行域：25 kHz 中断内。
 */
static float FOC_SlewSpeedReference(float current, float target,
                                    float slew_rpm_per_s,
                                    float sample_time) {
  float maximum_step; /* 当前采样周期内允许的最大转速变化量。= slew*sample_time，单位与 current 相同（rpm 或 A），恒 >= 0。 */
  float delta;        /* 目标转速与当前平滑转速之间的差值。= target-current，带符号，随后被夹到 ±maximum_step。 */

  if ((!isfinite(current)) || (!isfinite(target))) {
    return 0.0f;
  }

  if ((!isfinite(slew_rpm_per_s)) || (slew_rpm_per_s < 0.0f)) {
    slew_rpm_per_s = 0.0f;
  }

  if ((!isfinite(sample_time)) || (sample_time <= 0.0f)) {
    return current;
  }

  /* 20000rpm/s与40us对应每拍最多变化0.8rpm；斜率为0时参考保持不动。 */
  /* 0.8 rpm/拍 这个数字是这么来的——20000 rpm/s × 0.00004 s = 0.8 rpm。
   * 也就是说默认斜坡下，1500 rpm 的阶跃需要 1500/0.8 = 1875 拍 = 75 ms 才能走完，
   * 这段时间里速度 PI 看到的是连续的小误差而不是一个 1500 rpm 的大误差，因此
   * 它不会一次性把 Iq 推到限幅。若把 sample_time 换成速度环周期（1 ms），
   * 每拍步长会变成 20 rpm，等效加速度不变但速度 PI 看到的台阶变粗。
   * 斜率为 0 时 maximum_step = 0，delta 被夹成 0，返回 current，即参考冻结。 */
  maximum_step = slew_rpm_per_s * sample_time;
  delta = target - current;

  if (delta > maximum_step) {
    delta = maximum_step;
  } else if (delta < -maximum_step) {
    delta = -maximum_step;
  }

  return current + delta;
}

/**
 * @brief 修正限幅参数顺序
 *
 * 保证调用方的上下限参数满足 minimum <= maximum，否则原地交换。
 * 之所以要"容忍传反"：参数下发（上位机/控制台）很容易把上下限写反，直接在
 * PI_Clamp 里用反了的区间会得到完全错误的输出（先命中 maximum 分支），
 * 这里统一在入口处纠正，比在每个调用点做校验要可靠。
 * @param minimum 指向下限的指针，不可为 NULL（本函数不做 NULL 检查，调用者
 *        都是本文件内的函数，实参都是栈变量地址）。
 * @param maximum 指向上限的指针。
 * @return 无返回值，通过指针就地修改。
 * 单位：无量纲地比较大小，因此对电压（V）、电流（A）、rpm 等任何量纲都适用。
 * 注意：NaN 比较为假，若传入 NaN 不会被交换，NaN 会顺着后面的 PI_Clamp
 * 原样通过——这是刻意的"不掩盖非法值"策略。
 * 调用者：PI_Controller_Init、PI_Controller_SetLimits、SetOutputLimits、
 * SetIntegralLimits。
 */
static void PI_NormalizeLimits(float *minimum, float *maximum) {
  float temporary; /* 交换上下限时使用的临时变量。栈上临时量，仅在需要交换时被写入，不参与后续计算。 */

  if ((*minimum) > (*maximum)) {
    temporary = *minimum;
    *minimum = *maximum;
    *maximum = temporary;
  }
}

/**
 * @brief 配置增益、积分周期及限幅，并清空历史运行状态。
 * 输出与积分默认共用同一限幅区间；后续可通过各自设置接口单独调整。
 *
 * 本函数只负责"配置"，历史状态一律交给 PI_Controller_Reset 清零。
 * 参数单位见 PI_Controller_t 的字段说明：kp 为 输出/输入，ki 为 输出/(输入·s)，
 * sample_time 为 s，output_min/output_max 为输出量纲。
 * 写入顺序说明：先 Normalize 上下限、再钳 sample_time，最后才写进结构体，
 * 因此结构体里永远不会出现"瞬时非法值"（比如限幅反序）被中断读取的窗口——
 * 本函数虽然只在初始化阶段调用，但保持这个习惯仍然必要。
 * 默认策略：integral_min/integral_max 直接取输出的上下限，即"积分单独不超过
 * 输出范围"。这样即使条件积分抗饱和失效，积分项自身也有硬边界。之后电流环
 * 会通过 PI_Controller_SetLimits 把它改成按母线电压动态变化的限幅。
 */
void PI_Controller_Init(PI_Controller_t *pi, float kp, float ki,
                        float sample_time, float output_min, float output_max) {
  /* 初始化增益、采样周期、输出上下限，并清零历史积分状态。 */
  /* NULL 检查放在最前，保证后面所有解引用都安全；返回后调用方
   * 的控制器保持未初始化状态，后续 Run 会因为同样的 NULL 检查而返回 0.0f
   * （但请注意：FOC_Control_Init 传入的是 &control->xxx_pi，实际上不可能是
   * NULL，这个检查是防御性编程）。 */
  if (pi == NULL) {
    return;
  }

  PI_NormalizeLimits(&output_min, &output_max);
  /* 就地交换反序的上下限，保证后面写进结构体的区间一定合法。
   * 注意交换的是形参（栈上的副本），不会影响调用方的变量。 */

  if (sample_time < 0.0f) {
    sample_time = 0.0f;
  }
  /* 负周期没有物理意义（会造成积分反向增长），钳为 0。
   * sample_time = 0 的后果是积分增量恒为 0，控制器退化成纯比例控制，
   * 这是可控的退化，比产生反向积分安全。NaN 不会被这里拦住（比较为假）。 */

  pi->kp = kp;
  pi->ki = ki;
  pi->sample_time = sample_time;

  pi->output_min = output_min;
  pi->output_max = output_max;
  /* 以上五个字段是 volatile 的，此处是在初始化阶段写入，之后由
   * 参数下发路径更新；本模块每次运算开始时会各自快照到局部变量。 */

  /*
   * 默认让积分限幅等于输出限幅。
   */
  pi->integral_min = output_min;
  pi->integral_max = output_max;
  /* 电流环里这两个值随后会被 FOC_Control_Run 每拍用 ±voltage_limit
   * 覆盖；速度环里则一直保持 ±10 A（来自 FOC_SPEED_PI_OUTPUT_*_DEFAULT）。 */

  PI_Controller_Reset(pi);
  /* 清零 reference/feedback/error/proportional/integral/
   * output_unsaturated/output/saturation。放在最后是为了让"配置已就绪、
   * 历史已清空"这两件事在同一次调用里完成，避免调用者漏掉 Reset。 */
}

/* 仅清除误差、积分和输出历史，保留增益、周期与限幅参数。 */
/* 清除误差、积分和输出历史，但保留控制器的增益与限幅配置。 */
/* 本函数是"软复位"——只把运行历史清零，配置（kp/ki/sample_time 和
 * 四个限幅字段）原封不动。调用后控制器等价于"刚上电但已整定完毕"。
 * 副作用：积分归零意味着下一拍输出 = Kp*error（可能从 0 跳到一个非零值），
 * 因此不要在电机带载运行中随意调用本函数，除非紧接着也会重置输出侧（例如
 * 正在停机、PWM 已关断）。
 * 执行域：可能被初始化路径（主循环/上电）和中断路径（FOC_Control_Reset）
 * 调用；由于两者不会同时发生（上电初始化未完成时中断尚未使能），这里不加保护。
 */
void PI_Controller_Reset(PI_Controller_t *pi) {
  if (pi == NULL) {
    return;
  }

  pi->reference = 0.0f;   /* 参考值回零，仅影响观察与 PreloadOutput 的输入。 */
  pi->feedback = 0.0f;    /* 反馈回零，同样只影响观察。 */
  pi->error = 0.0f;       /* 误差回零，RunError 会在下一次运算时重新写入。 */

  pi->proportional = 0.0f; /* 比例项回零，下一拍由 kp*error 重新计算。 */
  pi->integral = 0.0f;     /* 积分项回零——这是本函数最关键的副作用，直接决定复位后第一拍的输出。 */

  pi->output_unsaturated = 0.0f; /* 未限幅输出回零（= proportional + integral 的一致值）。 */
  pi->output = 0.0f;             /* 对外输出回零。电流环的这两路输出会经 FOC_Control_Run 回传给应用层。 */

  pi->saturation = PI_SATURATION_NONE; /* 饱和标志复位为"未饱和"，避免复位后仍显示上一次的饱和告警。 */
}

/* 根据给定参考值和反馈值计算一次闭环 PI 输出。 */
/* 这是最常用的入口，语义为"把 reference/feedback 记录进结构体，
 * 再用二者之差调用 PI_Controller_RunError"，因此所有积分、限幅与抗饱和行为
 * 由 RunError 定义，本函数自身没有任何控制逻辑，只有一处减法。
 * 参数单位：reference 与 feedback 必须同量纲（电流环 A、速度环 rpm），
 * 返回值单位为该 PI 的输出量纲（电流环 V、速度环 A）。
 * 为什么要单独记录 reference/feedback：FOC_Control_Run 只把电流 PI 的输出
 * 传给应用层，应用层若要画"给定 vs 反馈"曲线就得靠这两个字段；同时
 * PI_Controller_PreloadOutput 也需要用它们反算比例项。
 * 边界：pi 为 NULL 时返回 0.0f（注意此时应用层会拿到 0 V 而不是"保持上一次
 * 输出"，对电流环来说等于瞬时去掉电压，属于故障态，正常不会发生）。
 * 调用频率：电流环 25 kHz × 2 个轴，速度环 1 kHz × 1 个轴。
 */
float PI_Controller_Run(PI_Controller_t *pi, float reference, float feedback) {
  if (pi == NULL) {
    return 0.0f;
  }

  pi->reference = reference;
  pi->feedback = feedback;

  return PI_Controller_RunError(pi, reference - feedback);
}

/**
 * @brief 直接使用误差运行PI，适用于PLL鉴相器等已经计算好误差的场景。
 * I候选 = I上一拍 + Ki*error*Ts，未限幅输出 = Kp*error + I候选。
 * 若输出饱和且误差仍使其向饱和方向发展，撤回本拍积分；反向误差仍
 * 允许积分退出饱和。此函数不改reference/feedback，由普通Run接口填写。
 *
 * 控制律（位置式 PI，离散积分显式乘周期）：
 *     integral_candidate = clamp(integral_old + ki*error*sample_time)   // 先夹积分
 *     output_unsaturated = kp*error + integral_candidate                // 再合成
 *     if (饱和 && 误差同向) 撤回本拍积分，用 integral_old 重新合成
 *     output = clamp(output_unsaturated)                                // 最后夹输出
 * 三个 clamp 的顺序不可交换：先夹积分让积分本身有界（防止大误差长时间累积），
 * 再判断饱和方向，最后才夹输出。若先夹输出再判断，就无法区分"输出被夹住"和
 * "输出本来就在范围内"。
 * @param error 本拍误差，单位 = 输入量纲（电流环 A、速度环 rpm）。允许为负，
 *        允许含噪声（电流采样噪声会直接体现在这里）。
 * @return 限幅后的输出，单位 = 输出量纲（电流环 V、速度环 A）。
 * 边界与错误路径：pi 为 NULL 返回 0.0f；error 为 NaN 时所有比较为假，
 *        走"不饱和"分支，NaN 会被写进输出（下游 SVPWM 需要自行防护）。
 * 调用者：PI_Controller_Run（常规闭环）与观测器 PLL 等直接传误差的场景。
 * 执行域：25 kHz 中断；本函数是常数时间、无循环、无函数调用（除内联的
 * PI_Clamp 外），适合高频调用。
 */
float PI_Controller_RunError(PI_Controller_t *pi, float error) {
  float kp;          /* 本次计算使用的比例增益快照。单位 输出/输入（电流环 V/A，速度环 A/rpm）。 */
  float ki;          /* 本次计算使用的积分增益快照。单位 输出/(输入·s)。 */
  float sample_time; /* 积分离散化所用的采样周期。单位 s，电流环 4e-5，速度环 1e-3；为 0 时本次积分不增长。 */

  float output_min; /* PI 输出下限。单位同输出；电流环由 FOC_Control_Run 动态设为 -voltage_limit。 */
  float output_max; /* PI 输出上限。单位同输出；与 output_min 成对，保证 minimum<=maximum（由 SetLimits 保证）。 */

  float integral_min; /* 积分项下限。单位同输出，因为 integral 已含 Ki 累积。 */
  float integral_max; /* 积分项上限。单位同输出。 */

  float integral_old;       /* 上一次保存的积分项。进入本拍时的积分状态，是本函数唯一的"历史记忆"，条件积分会用它来回退。 */
  float integral_candidate; /* 当前误差积分后的候选值。先无条件积分并夹限幅，之后可能被抗饱和逻辑丢弃。 */

  float output_unsaturated; /* 尚未经过输出限幅的 PI 结果。= proportional + （被采纳的）integral，用于判断饱和方向。 */
  float output;             /* 限幅后的最终输出。= clamp(output_unsaturated)，是要写入 pi->output 并返回的值。 */

  if (pi == NULL) {
    return 0.0f;
  }

  /*
   * 先读取到局部变量，避免一次计算中参数被重复读取。
   */
  /* 这 8 个字段都是 volatile，可能被通信/主循环域在任意时刻改写。
   * 逐个快照到栈变量后，本次计算从头到尾用的是同一组参数，结果自洽；
   * 代价是"同时改 Kp 和 Ki"可能被拆成两拍生效（非事务），但对 40 us 的控制
   * 周期而言完全可接受，也省掉了临界区/关中断的开销。 */
  kp = pi->kp;
  ki = pi->ki;
  sample_time = pi->sample_time;

  output_min = pi->output_min;
  output_max = pi->output_max;

  integral_min = pi->integral_min;
  integral_max = pi->integral_max;

  pi->error = error;
  /* 只写 error，不写 reference/feedback——这正是 RunError 与 Run 的
   * 分工。PLL 场景下 reference/feedback 会保持上一拍的陈旧值，调试时不要
   * 用它们反推 PLL 的误差。 */

  /*
   * 比例项
   */
  pi->proportional = kp * error;
  /* 比例项单独存字段是为了让 PI_Controller_PreloadOutput 能反算
   * integral = desired_output - proportional，避免它重复算一遍 kp*error。
   * 注意比例项是"瞬时"的、无记忆的：它随 error 每拍变化，是电流环带宽的
   * 主要来源。 */

  /*
   * 计算积分候选值
   */
  integral_old = pi->integral;
  /* 先把历史积分读进局部变量，后面抗饱和回退时直接用这份副本，
   * 保证"回退"就是精确地恢复到进入本函数前的状态，不引入任何累积误差。 */

  integral_candidate = integral_old + ki * error * sample_time;
  /* 离散积分。这里显式乘 sample_time，所以 ki 是连续时间增益
   * （占空比意义上的"每周期增量"已经被吸收）。电流环：ki*error*4e-5；
   * 速度环：ki*error*1e-3。sample_time 为 0 时该项不增长（冻结积分）。 */

  integral_candidate = PI_Clamp(integral_candidate, integral_min, integral_max);
  /* 第一道抗饱和——积分自身硬限幅。即使下面条件积分的判断失效
   * （例如 error 是 NaN，比较全为假），积分也不会无限增长。 */

  /*
   * 使用新的积分值计算未限幅输出。
   */
  output_unsaturated = pi->proportional + integral_candidate;
  /* 位置式结构下"未限幅输出"就是 P 与 I 的直接和。它保留下来供
   * 饱和判断使用，也是 Live Watch 里区分"真实需求"和"被夹住后输出"的依据。 */

  /*
   * 条件积分抗饱和。
   *
   * 先决定是否撤销本拍积分，最后只执行一次输出限幅。
   * 此处判断的是单个PI输出限幅，不包含后级SVPWM的电压缩放反馈。
   */
  /* 条件积分（conditional integration）抗饱和，判据是两个条件的与：
   *   - 未限幅输出已经越过上限且 error>0（误差还在把输出往上推），或
   *     未限幅输出已经越过下限且 error<0（误差还在往下推）；
   * 命中就把 integral_candidate 丢弃，退回 integral_old，并用旧积分重新合成
   * 未限幅输出。这样积分器不会在"输出已经打满"的情况下继续累积，从而让控制器
   * 能在误差反号时立刻（而不是等积分慢慢退饱和）恢复线性区。
   * 为什么用 error 的符号而不是 output_unsaturated 的符号来判断方向：位置式
   * 结构里输出与误差的"推动方向"是同一件事，直接用 error 符号最直观，也避免
   * 在输出恰好等于 0 时判断失效。
   * 局限（源码注释已点明）：这里只看本 PI 自己的输出限幅，不看后级 SVPWM 的
   * 电压缩放是否还额外砍掉了一部分。当前实现已经用"实时母线电压 + dq 圆形
   * 限幅"把这件事提前到限幅里解决，所以不再需要末端缩放。
   * 注意 fallthrough 的代价：抗饱和生效时本拍的积分等于上一拍，即这一拍"白积"，
   * 这是有意的（宁可少积，不可超限）。 */
  if (((output_unsaturated > output_max) && (error > 0.0f)) ||
      ((output_unsaturated < output_min) && (error < 0.0f))) {
    integral_candidate = integral_old;
    output_unsaturated = pi->proportional + integral_old;
  }

  output = PI_Clamp(output_unsaturated, output_min, output_max);
  /* 第二道也是最后一道限幅。即使在抗饱和回退之后，比例项本身仍可能
   * 让输出越界（误差很大时 kp*error 单独就超限），所以这次 clamp 不能省。 */

  pi->integral = integral_candidate;
  /* 写回被采纳的积分（可能是新积分，也可能是回退后的旧积分）。
   * 这是本函数唯一修改历史状态的语句，也是控制器"记忆"的全部来源。 */

  pi->output_unsaturated = output_unsaturated;
  /* 写回抗饱和处理之后的未限幅输出，供外部观察"真实需求"。
   * 注意它与 pi->output 的差就是被限幅削掉的部分。 */

  pi->output = output;
  /* 写回最终输出。电流环里这两路输出会被 FOC_Control_Run 复制到
   * control->ud_output / uq_output 并回传给应用层。 */

  if (output_unsaturated > output_max) {
    pi->saturation = PI_SATURATION_HIGH;
  } else if (output_unsaturated < output_min) {
    pi->saturation = PI_SATURATION_LOW;
  } else {
    pi->saturation = PI_SATURATION_NONE;
  }
  /* 饱和方向供诊断、Ready 及上游速度积分抗饱和使用，取值 -1/0/+1。 */

  return output;
  /* 返回值与 pi->output 永远是同一个值，调用者用哪个都可以；
   * FOC_Control_Run 采用返回值赋值给结构体字段的写法。 */
}

/* 运行时更新比例和积分增益，不改变积分历史。 */
/* 热更新整定值的入口。两个字段连续写入（volatile 之间没有原子性
 * 保证），因此中断可能在两次写之间插入一拍计算，那一拍会用"新 Kp + 旧 Ki"，
 * 造成输出有一次很小的毛刺；对本工程使用的频率（人工/上位机下发）可以忽略。
 * 若要完全避免，可先关中断再调用，但这不在本函数的职责范围内。
 * 参数单位：kp = 输出/输入，ki = 输出/(输入·s)。本函数不做有限性、符号或
 * 量纲检查，NaN/Inf/负增益会被原样写入——参数下发路径必须自己校验。
 * 边界：pi 为 NULL 时直接返回。积分 integral 保持不动，所以稳态输出不会突变。
 */
void PI_Controller_SetGains(PI_Controller_t *pi, float kp, float ki) {
  if (pi == NULL) {
    return;
  }

  pi->kp = kp;
  pi->ki = ki;
}

/* 运行时更新离散积分使用的采样周期。 */
/* sample_time 直接是积分强度的缩放因子（增量 = ki*error*sample_time），
 * 改大等价于把 Ki 同比放大。电流环应填 1/25000 = 4e-5；速度环应填
 * 电流环周期 × speed_loop_divider（默认 1e-3）。
 * 钳位：负值被钳成 0.0f（积分停止增长，等价于把控制器退化为纯比例）。
 * 典型误用：改了 FOC_Control_t.speed_loop_divider 而忘记同步本函数，
 * 结果速度环积分强度按分频比成比例偏大/偏小。
 * 边界：pi 为 NULL 直接返回；NaN 不会被钳位（比较为假）。
 */
void PI_Controller_SetSampleTime(PI_Controller_t *pi, float sample_time) {
  if (pi == NULL) {
    return;
  }

  if (sample_time < 0.0f) {
    sample_time = 0.0f;
  }

  pi->sample_time = sample_time;
}

/* 同时设置输出和积分限幅，并立即修正已有状态。 */
/* 这是电流环每拍都会调用的函数（FOC_Control_Run 里对 id_pi、iq_pi
 * 各一次），因此它的执行开销直接落在 25 kHz 中断里——本函数只做几次比较和
 * 赋值，没有循环，开销可忽略。
 * 单位：与输出量纲一致（电流环为 V）。同一个区间同时用于输出限幅和积分限幅，
 * 这样"积分单独不会超过输出范围"，条件积分抗饱和的推理更简单。
 * 副作用：会把已有的 integral 和 output 立即夹进新区间。当母线电压下降导致
 * voltage_limit 变小时，这会让被限幅削掉的积分当场退掉（而不是慢慢退饱和），
 * 这正是"母线跌落时电流环要立刻放弃多余电压"想要的行为。
 * 顺序：先 Normalize 再写四字段，避免中断读到反序区间。
 * 注意 output_unsaturated 不在这里夹——它按定义就是"未限幅"值，且会在下一拍
 * RunError 里被重新计算，保留它可以真实反映"需求被削掉了多少"。
 * 边界：pi 为 NULL 直接返回。
 */
void PI_Controller_SetLimits(PI_Controller_t *pi, float minimum,
                             float maximum) {
  if (pi == NULL) {
    return;
  }

  PI_NormalizeLimits(&minimum, &maximum);

  pi->output_min = minimum;
  pi->output_max = maximum;

  pi->integral_min = minimum;
  pi->integral_max = maximum;

  pi->integral = PI_Clamp(pi->integral, minimum, maximum);

  pi->output = PI_Clamp(pi->output, minimum, maximum);
}

/* 仅设置 PI 输出限幅；积分限幅保持不变。 */
/* 只改 output_min/output_max 并把当前 output 夹进新区间，
 * integral 与 integral_min/integral_max 完全不动。
 * 适用场合：想让"输出电压上限"单独收紧、但希望积分器保留原有工作点，
 * 以便限幅放开后立刻恢复到之前的电压需求。若输出限幅比积分限幅更窄，
 * 条件积分抗饱和仍会阻止积分朝越界方向继续累积。
 * 单位：与输出量纲一致。边界：pi 为 NULL 直接返回；上下限反序会自动交换。
 */
void PI_Controller_SetOutputLimits(PI_Controller_t *pi, float minimum,
                                   float maximum) {
  if (pi == NULL) {
    return;
  }

  PI_NormalizeLimits(&minimum, &maximum);

  pi->output_min = minimum;
  pi->output_max = maximum;

  pi->output = PI_Clamp(pi->output, minimum, maximum);
}

/* 仅设置积分项限幅，并将当前积分值夹到新范围内。 */
/* 只改 integral_min/integral_max，并把当前 integral 夹进新区间；
 * output_min/output_max 与 output 都不动。单位与输出量纲一致（因为 integral
 * 已经包含 Ki 与 sample_time 的累积，与输出同量纲）。
 * 适用场合：把积分限幅收得比输出限幅更窄，留出"积分余量"以便参考微调时
 * 仍能快速响应；或把两个极限都设为 0 来临时禁用积分作用（等效纯比例）。
 * 边界：pi 为 NULL 直接返回；上下限反序会自动交换。
 */
void PI_Controller_SetIntegralLimits(PI_Controller_t *pi, float minimum,
                                     float maximum) {
  if (pi == NULL) {
    return;
  }

  PI_NormalizeLimits(&minimum, &maximum);

  pi->integral_min = minimum;
  pi->integral_max = maximum;

  pi->integral = PI_Clamp(pi->integral, minimum, maximum);
}

/**
 * @brief 按希望保持的输出反算积分状态，用于速度环使能时减少Iq跳变。
 * 若反算积分超出积分限幅，只能保持限幅后可实现的输出。
 *
 *
 *   (1) FOC_Control_Run 检测到使能位由 0 变 1（进入速度模式）时，用当前的
 *       iq_ref_active 作为 desired_output、用当前转速同时作为 reference 和
 *       feedback 预装 speed_pi —— 误差为 0，于是 integral 直接被设成目标 Iq，
 *       速度 PI 接管的第一拍输出就等于原来的电流模式 Iq，不会产生转矩跳变；
 *   (2) reference.iq_slew_limit 生效且下游限速真实削掉了 PI 输出时，用"可达值"
 *       回装，避免速度积分器记住一个物理上达不到的 Iq（windup）。
 * @param desired_output 希望本拍保持的输出，单位 = 输出量纲（速度环为 A，电流环为 V）。
 * @param reference      当前参考值，单位 = 输入量纲（速度环 rpm），仅用于算比例项。
 * @param feedback       当前反馈值，单位 = 输入量纲，仅用于算比例项。
 * @return 无返回值（void），全部结果写回结构体。
 * 边界与限制：
 *   - pi 为 NULL 直接返回；
 *   - desired_output 先被夹到 [output_min, output_max]；
 *   - 反算出的 integral 再被夹到 [integral_min, integral_max]。**若积分限幅比
 *     输出限幅窄，实际保持的输出会小于 desired_output**，此时无扰是"部分无扰"，
 *     源码注释已明确说明这一点；
 *   - 本函数只快照了 output_min/output_max，integral_min/integral_max 是直接
 *     从结构体读取的，因此恰好在调用瞬间被跨域改写的极端情况下，两者可能来自
 *     不同的参数版本（无害，最坏只是这一次预装略偏）。
 * 后置条件：结构体状态与"刚跑完一拍 RunError 且误差为 reference-feedback"完全
 * 一致，因此紧接着的 Run 会从 desired_output 附近连续演化。
 * 执行域：25 kHz 中断内调用（模式切换路径），不可阻塞。
 */
void PI_Controller_PreloadOutput(PI_Controller_t *pi, float desired_output,
                                 float reference, float feedback) {
  float output_min; /* 预加载时采用的输出下限。函数开头快照，避免计算中途被改写。 */
  float output_max; /* 预加载时采用的输出上限。同上，与 output_min 配合保证 desired_output 合法。 */

  if (pi == NULL) {
    return;
  }

  output_min = pi->output_min;
  output_max = pi->output_max;

  desired_output = PI_Clamp(desired_output, output_min, output_max);
  /* 目标输出本身不能超过输出限幅，否则反算出的积分会立刻被夹，
   * 结果不可预期。先夹一次让后面的推理成立。 */

  pi->reference = reference;
  pi->feedback = feedback;
  pi->error = reference - feedback;
  /* 把三个观察字段也设成"接管时"的一致值。进入速度模式时
   * reference 与 feedback 都传当前转速，因此 error = 0、proportional = 0，
   * integral 就等于 iq_ref_active，语义非常清晰。 */

  pi->proportional = pi->kp * pi->error;
  /* 比例项必须用同一拍的误差算出来，否则反算的积分会带上一个偏差，
   * 导致下一拍输出偏离 desired_output。 */

  /*
   * desired_output = proportional + integral
   *
   * 所以：
   * integral = desired_output - proportional
   */
  pi->integral = desired_output - pi->proportional;
  /* 核心一步——位置式 PI 的结构是 output = kp*error + integral，
   * 于是反解 integral = desired_output - kp*error。这就是"按输出反算状态"
   * 的全部内容，没有任何近似或迭代。 */

  pi->integral = PI_Clamp(pi->integral, pi->integral_min, pi->integral_max);
  /* 积分必须落在自己允许的范围内，否则下一拍 RunError 会立刻把它
   * 夹回去，预装就失败（输出会跳变）。注意这里读的是结构体里的积分限幅
   * （未快照），与上面快照的输出限幅可能不是同一次参数版本。 */

  pi->output_unsaturated = pi->proportional + pi->integral;
  /* 用夹过的积分重新合成未限幅输出，保证结构与"真跑一拍"一致；
   * 当积分被夹时这里会小于 desired_output，即"只能保持限幅后可实现的输出"。 */

  pi->output = PI_Clamp(pi->output_unsaturated, output_min, output_max);
  /* 最后再夹一次输出。理论上 output_unsaturated 已被 desired_output
   * 的预先夹位与积分夹位约束住，但保留这次 clamp 可以覆盖"积分限幅比输出限幅
   * 宽"的参数组合，保证 pi->output 永远合法。 */

  if (pi->output_unsaturated > output_max) {
    pi->saturation = PI_SATURATION_HIGH;
  } else if (pi->output_unsaturated < output_min) {
    pi->saturation = PI_SATURATION_LOW;
  } else {
    pi->saturation = PI_SATURATION_NONE;
  }
  /* 与 PI_Controller_RunError 末尾完全相同的三段饱和判定，保证
   * "预装"和"真跑一拍"留下的诊断状态语义一致，调试时不会因为走了预装分支
   * 就看到不同的饱和标志。注意这里判定用的是夹过积分之后的 output_unsaturated，
   * 所以当 desired_output 本身就在限幅内时，饱和标志一般会回到 NONE。 */
}

/* ======================== FOC电流环/速度环 ======================== */
/* 以上是通用 PI 层（对本工程无任何电机相关假设，可被观测器 PLL 等
 * 复用）；以下是 FOC 专用层，负责把三个 PI 组织成"电流环 25 kHz + 速度环 1 kHz"
 * 的串级结构，并处理模式切换与电压限幅。 */

/* 初始化电流环、速度环及其调度状态，建立启动时的默认参考值。 */
/* 调用一次即可让控制器进入可运行状态。参数 current_loop_sample_time
 * 单位 s，本工程由应用层传 0.00004f（25 kHz）。传入 <=0 或异常值时回退到 25 kHz，
 * 避免把 0 周期写进积分器导致积分永不增长（那会让电流环退化为纯比例）。
 * 注意本函数不配置任何外设（PWM/ADC/定时器由应用层负责），也不使能中断。
 * 调用者：应用层上电初始化，且必须在开启 25 kHz 注入中断之前完成。
 */
void FOC_Control_Init(FOC_Control_t *control, float current_loop_sample_time) {
  float speed_loop_sample_time; /* 速度环分频后对应的离散采样周期。单位 s，= current_loop_sample_time * speed_loop_divider，默认 4e-5*25 = 1e-3 s（1 kHz）。它直接作为 speed_pi 的积分步长。 */

  if (control == NULL) {
    return;
  }

  if (current_loop_sample_time <= 0.0f) {
    current_loop_sample_time = 0.00004f;
  }
  /* 回退值 0.00004f 就是 25 kHz 对应的周期，与 TIM1 的
   * ARR=3399、170 MHz 中心对齐（f = 170e6/(2*3400) = 25 kHz）一致。
   * 用 <=0 判断而不是 ==0，可以顺带挡住负数；NaN 仍会漏过（比较为假）。 */

  control->speed_loop_divider = FOC_SPEED_LOOP_DIVIDER_DEFAULT;
  /* 25，即速度环 1 kHz。运行时若要改分频，必须同时调用
   * PI_Controller_SetSampleTime(&control->speed_pi, 电流环周期*新分频)。 */

  speed_loop_sample_time =
      current_loop_sample_time * (float)control->speed_loop_divider;
  /* 显式转 float 是为了让乘法按单精度进行（divider 是 uint16_t，
   * 在 C 的整数提升规则下会被提升为 int 再转 float，写法上更明确）。 */

  /* Id电流环使用新电机分支中已有的默认参数。 */
  /* 输出上下限先传 0.0f/0.0f（因此初值限幅被夹成 ±0 V，输出恒为 0），
   * 这只是"占位"：真正的限幅由 FOC_Control_Run 每拍用 PI_Controller_SetLimits
   * 按实时母线电压动态设置（±voltage_limit）。之所以不在这里给一个固定值，
   * 是因为母线电压在运行时可能变化（例如电池放电），固定限幅会造成积分饱和。 */
  PI_Controller_Init(&control->id_pi, FOC_ID_PI_KP_DEFAULT,
                     FOC_ID_PI_KI_DEFAULT, current_loop_sample_time, 0.0f,
                     0.0f);

  /* Iq电流环与Id环使用相同参数，匹配当前表贴式电机模型。 */
  /* 表贴式（隐极）PMSM 的 Ld≈Lq，两轴电气动态一致，所以 Kp/Ki 相同。
   * 若换成凸极（内置式）电机，q 轴通常需要单独整定。 */
  PI_Controller_Init(&control->iq_pi, FOC_IQ_PI_KP_DEFAULT,
                     FOC_IQ_PI_KI_DEFAULT, current_loop_sample_time, 0.0f,
                     0.0f);

  /* 速度环输出为Iq参考值，使用偏保守参数并保留正负5 A限幅。 */
  /* 注意源码注释里的"正负5 A"是历史遗留描述，实际使用的宏
   * FOC_SPEED_PI_OUTPUT_MIN_DEFAULT/MAX_DEFAULT 已经是 ±10 A（见 controller.h
   * 中的量程推导：5 mΩ×24 倍 = 0.12 V/A，1.65 V 偏置对应 ±13.75 A）——以源码
   * 为准，注释未同步。速度环的 output_min/max 与 integral_min/max 都被设成
   * ±10 A，且之后不再被 FOC_Control_Run 改动（动态限幅只作用于两个电流环）。 */
  PI_Controller_Init(&control->speed_pi, FOC_SPEED_PI_KP_DEFAULT,
                     FOC_SPEED_PI_KI_DEFAULT, speed_loop_sample_time,
                     FOC_SPEED_PI_OUTPUT_MIN_DEFAULT,
                     FOC_SPEED_PI_OUTPUT_MAX_DEFAULT);

  control->id_ref = 0.0f;
  /* d 轴命令默认 0 A。表贴式 PMSM 在基速以下 d 轴电流不产生转矩，
   * 给它 0 是最省的（无额外铜损），也让"电流环只需跟 0"最好整定。 */

  control->iq_ref = 0.20f;
  /* q 轴命令默认 0.20 A（而不是 0）。给一点点转矩电流的目的是：
   * 让电机在闭环前有确定的小转矩/端电压，便于观测器在启动阶段获得有效的
   * 电压-电流关系；同时也验证电流采样与 Park 变换的符号是否正确。
   * 这个值只在电流模式下直接生效（或作为退出速度模式时的回写起点）。 */

  /* 上电默认仲裁结果：电流参考直接取命令值，速度环使能。 */
  /* 这里先按"速度模式 + 用命令值做种子"建立一份合法参考。
   * 真正的第一拍参考会在应用层状态机提交（FOC_Control_SubmitReference）时被
   * 整体覆盖，所以这些初值只保证"在提交之前控制器也有确定行为"。 */
  control->reference.theta_ctrl = 0.0f;
  /* 控制角默认 0 rad，即把转子电角度假设为 0；在真实启动流程里
   * 会被 ALIGN 的固定定位角覆盖。用 0 而不是随机值是为了让上电瞬间的
   * Park 变换有确定结果（哪怕它并不对准转子）。 */
  control->reference.id_ref = control->id_ref;
  control->reference.iq_ref = control->iq_ref;
  control->reference.speed_loop_enable = 1U;
  /* 默认走速度模式；但 speed_loop_enable_last = 0U（见下），因此
   * 第一次 FOC_Control_Run 会检测到 0->1 的跳变并执行一次无扰预装，把速度 PI
   * 的积分预置成当时的 iq_ref_active（=0.20 A），不会产生转矩冲击。 */
  control->reference.speed_slew_limit = -1.0f;
  /* -1 表示"不额外限制"，使用 control->speed_slew_rpm_per_s 的常规斜坡。 */
  control->reference.iq_slew_limit = 0.0f;
  /* 0 表示"不限制 Iq 变化率"（只有 >0 才生效）。 */

  control->speed_command_rpm = 1500.0f;
  /* 控制台/上位机目标转速的默认值，单位 rpm。应用层负责把它复制到
   * speed_ref_rpm；本控制器不直接读它。1500 rpm 属于中低速区间，仍是
   * "低速用实测端电压"的适用范围（1200 rpm 为重构切换门槛附近）。 */
  control->speed_ref_rpm = 1500.0f;
  /* 实际参与斜坡的参考，单位 rpm。上电即 1500，配合
   * speed_ref_active_rpm 的初值（同样 1500）使第一拍不产生转速阶跃。 */
  control->speed_slew_rpm_per_s =
      FOC_SPEED_REFERENCE_SLEW_RPM_PER_S_DEFAULT;
  /* 常规斜坡速率 20000 rpm/s。注意 FOC_Control_Run 每次都会把它与
   * reference.speed_slew_limit 取小，因此启动阶段可以被应用层临时压低。 */

  /* 上电默认使用速度闭环。 */
  control->speed_loop_enable = 1U;
  /* volatile 的"请求"字段，可被通信域改；本模块只在模式切换回写时
   * 读它（其实 Run 读的是 reference 里的仲裁结果）。 */
  control->speed_loop_enable_last = 0U;
  /* 故意与 speed_loop_enable（1）不一致，从而保证第一次 Run 必然走
   * 一次模式切换分支，完成速度环种子设定。若这里也写 1U，第一次进入就跳过
   * 预装，速度 PI 会从零积分起步，可能造成 Iq 缓慢建立而丢失启动时机。 */

  control->speed_loop_counter = 0U;
  /* 分频计数器清零，使上电后可获得一个完整的 1 ms 速度环窗口。 */

  control->id_feedback = 0.0f;
  control->iq_feedback = 0.0f;
  /* 两个反馈量初值 0（电机还没转，也没有电流采样结果）。 */
  control->speed_feedback = 0.0f;
  /* 转速反馈初值 0，进入速度模式前由应用层每拍覆写。 */
  control->speed_ref_active_rpm = control->speed_ref_rpm;
  /* 斜坡内部状态初值 = 参考值，使未进入速度模式前两者一致。 */
  control->speed_ref_previous_rpm = control->speed_ref_rpm;

  control->iq_ref_active = control->iq_ref;
  control->iq_ref_target = control->iq_ref;
  /* 实际 Iq 与目标 Iq 都从 0.20 A 起步，保证第一拍电流环有一个
   * 确定的小给定；之后 iq_ref_target 由速度 PI 或仲裁参考驱动。 */

  control->ud_output = 0.0f;
  control->uq_output = 0.0f;
  /* 两路电压输出初值 0 V，即上电不输出电压（PWM 占空比应为 50%
   * 由应用层在 SVPWM 里按 0 矢量处理）。 */

  control->voltage_limit = 0.0f;
  control->speed_voltage_limited = 0U;
  /* 电压矢量上限初值 0，等第一拍 FOC_Control_Run 用实测母线电压
   * 计算。设 0 而不是某个猜测值，是为了避免在母线电压还没采样到之前就给出
   * 过大电压（那会在启动瞬间冲击电流）。 */

}

/* 复位两个 PI 的历史状态和 FOC 运行时反馈，不改变参考值配置。 */
/* 源码注释写的是"两个 PI"，实际复位的是三个（id/iq/speed），
 * 以代码为准——注释未同步。
 * 使用场景：故障清除、通信超时停车、从 IDLE 重新开始一套完整启动流程之前。
 * 与 FOC_Control_Init 的区别：本函数保留所有可调参数与外部命令（Kp/Ki/
 * sample_time/限幅/速度命令），只清运行历史，因此可以在运行期间（停机后）
 * 反复调用而不用重新下发参数。
 */
void FOC_Control_Reset(FOC_Control_t *control) {
  if (control == NULL) {
    return;
  }

  PI_Controller_Reset(&control->id_pi);
  PI_Controller_Reset(&control->iq_pi);
  PI_Controller_Reset(&control->speed_pi);
  /* 三个 PI 的积分、输出、误差历史与饱和标志全部清零，但每个 PI 的
   * Kp/Ki/sample_time/限幅保持不变。注意速度 PI 的积分被清零意味着复位后速度
   * 模式第一拍的 Iq 只来自比例项——不过紧接着的速度环输出会很小（误差也不大时），
   * 加上 iq_ref_active 被重置为 iq_ref，因此不会产生大转矩冲击。 */

  control->speed_loop_counter = 0U;
  /* 分频计数器清零，复位后重新计满 25 拍再跑速度环。 */
  control->speed_loop_enable_last =
      (control->speed_loop_enable != 0U) ? 1U : 0U;
  /* 把"上一拍使能"同步为当前请求，**避免复位后又触发一次无扰预装**。
   * 这一点很关键：复位已经把速度 PI 清空，如果此时再走一次预装分支，
   * 就会用归零后的 iq_ref_active 去预装，语义混乱；同步后复位是"干净起点"。
   * 这里是本文件里少数读取 volatile speed_loop_enable 的地方之一。 */

  control->id_feedback = 0.0f;
  control->iq_feedback = 0.0f;
  /* 反馈缓存清零，使复位后的显示/曲线与"尚未采样"状态一致。 */
  control->speed_feedback = 0.0f;
  control->speed_ref_active_rpm = control->speed_ref_rpm;
  /* 斜坡内部状态重新对齐当前参考，复位后不产生额外爬升。注意此时
   * speed_ref_rpm 是"命令值"而不是实测转速，所以复位后下一次速度环会朝着
   * 已设定的目标转速重新爬升。 */
  control->speed_ref_previous_rpm = control->speed_ref_rpm;

  control->iq_ref_active = control->iq_ref;
  control->iq_ref_target = control->iq_ref;
  /* Iq 链路重置为电流命令值（当前可能是 0.20 A 或应用层改过的值），
   * 而不是清零——这样"电流模式"在复位后立刻有一个确定的给定。 */

  control->ud_output = 0.0f;
  control->uq_output = 0.0f;
  control->voltage_limit = 0.0f;
  control->speed_voltage_limited = 0U;
  /* 电压侧归零。voltage_limit = 0 会让下一次 SetLimits 把两个电流环
   * 限成 ±0 V，即复位后的第一拍不输出电压；但同一拍 Run 会立刻用实测母线电压
   * 重算 voltage_limit，所以实际上只有"复位到第一拍之间"是零电压窗口。 */

  control->reference.speed_slew_limit = -1.0f;
  control->reference.iq_slew_limit = 0.0f;
  /* 清除启动阶段遗留的斜坡覆盖参数，恢复"常规斜坡 + 不限 Iq 速率"。
   * 注意 reference 的 theta_ctrl/id_ref/iq_ref/speed_loop_enable 没有被复位——
   * 它们会在下一拍被应用层重新提交，本函数刻意不去猜测控制目标。 */
}

/* 提交本周期仲裁结果；reference是控制器的唯一参考入口。 */
/* 调用顺序要求——应用层必须在同一拍、调用 FOC_Control_Run **之前**
 * 调用本函数，否则 Run 会消费上一拍的参考（角度和控制目标是旧值，对启动过程
 * 是致命的）。本函数不修改任何 PI 状态，这是刻意的约束：启动/接管/反转状态
 * 只能通过 reference 表达控制意图，不能去动积分器。
 * 参数单位：theta_ctrl 电角度 rad；id_ref/iq_ref 电流 A；speed_loop_enable 布尔。
 * 关键副作用（容易踩坑）：两个斜坡覆盖字段被强制复位为 (-1.0f, 0.0f)。因此
 * 应用层若需要有限斜坡，必须在本函数之后自己再写这两个字段——启动状态机就是
 * 这样做的（否则会沿用上一次启动期设置的旧限制）。
 * 边界：control 为 NULL 时直接返回，本拍参考保持上一拍内容（控制器会继续按旧
 * 目标输出，属于故障兜底）。
 */
void FOC_Control_SubmitReference(FOC_Control_t *control, float theta_ctrl,
                                 float id_ref, float iq_ref,
                                 uint8_t speed_loop_enable) {
  if (control == NULL) {
    return;
  }

  control->reference.theta_ctrl = theta_ctrl;
  /* 本模块只把角度转交给应用层做 Park/逆 Park，自身不消费它。 */
  control->reference.id_ref = id_ref;
  control->reference.iq_ref = iq_ref;
  /* iq_ref 只在使能=0 时被直接使用，或在使能跳变的那一拍作为
   * iq_ref_target 的起点。 */
  control->reference.speed_loop_enable = (speed_loop_enable != 0U) ? 1U : 0U;
  /* 规范化为 0/1，保证与 speed_loop_enable_last 的比较是布尔比较。 */
  control->reference.speed_slew_limit = -1.0f;
  /* 复位为"不额外限制"，本拍速度参考按常规斜坡逼近。 */
  control->reference.iq_slew_limit = 0.0f;
  /* 复位为"不限制 Iq 速率"，本拍 iq_ref_active 直接等于目标。 */
}

/* 执行一次电流环控制，并按分频条件更新速度环输出。 */
/* 本函数是 25 kHz 中断的实际工作函数，调用链为
 * ADC1_2_IRQHandler -> HAL_ADC_IRQHandler(&hadc1) ->
 * HAL_ADCEx_InjectedConvCpltCallback -> MotorApp_OnInjectedConversion -> 本函数。
 * 它必须是常数时间、无阻塞、无动态分配的；本函数内部唯一的外部计算是
 * arm_sqrt_f32（VSQRT.F32 单指令）。
 * 参数单位：id_feedback/iq_feedback 为 A（Park 变换后的 dq 电流）；
 * dc_bus_voltage 为 V（母线电压采样，可能还没做滤波）；ud_output/uq_output
 * 为可选的输出指针，单位 V，可为 NULL。
 * 转速反馈不经过形参：应用层在本拍调用前把实际反馈量写入 control->speed_feedback
 * （闭环/接管时为观测器 50 Hz 滤波值，I/F 时为虚拟转速），本函数直接读该字段。
 * 前置条件：应用层已在同一拍调用 FOC_Control_SubmitReference 提交本拍参考，
 * 并已写入 control->speed_feedback。
 * 后置条件：control->ud_output/uq_output/voltage_limit/iq_ref_active/
 * speed_loop_enable_last 等字段为本拍值；若输出指针非空，其中写入与结构体
 * 相同的电压值。
 * 边界与错误路径：control 为 NULL 时把两个指针写成 0 V 并返回（安全失效，
 * 相当于输出零矢量）；speed_loop_divider 为 0 时按 1 处理（防御性，避免除零
 * 或计数器永不触发）；母线电压非有限或 <=0 时 voltage_limit = 0，两个电流环
 * 的限幅都变成 ±0 V，即停止输出电压（故障可见、可控）。
 */
void FOC_Control_Run(FOC_Control_t *control, float id_feedback,
                     float iq_feedback, float dc_bus_voltage, float *ud_output,
                     float *uq_output) {
  float uq_limit_squared; /* 扣除 d 轴电压后的 q 轴电压平方余量。= voltage_limit² - ud²，单位 V²，可以为负（表示 d 轴已把矢量额度用尽）。 */
  float uq_limit;         /* q 轴 PI 的动态输出限幅。= sqrt(max(uq_limit_squared,0))，单位 V，恒 >= 0；作为 iq_pi 的 ±限幅。 */
  float voltage_limit;    /* 当前母线电压下允许的电压矢量幅值。= Vbus/√3，单位 V；母线异常时为 0。 */
  uint32_t speed_enabled; /* 规范化后的速度环使能状态。只可能取 0 或 1，来源是 reference.speed_loop_enable（仲裁结果，不是 volatile 请求字段）。 */
  uint16_t speed_divider; /* 速度环相对电流环的执行分频。单位"个电流环周期"，本拍从 control->speed_loop_divider 读出，为 0 时按 1 用。 */

  if (control == NULL) {
    if (ud_output != NULL) {
      *ud_output = 0.0f;
    }

    if (uq_output != NULL) {
      *uq_output = 0.0f;
    }

    return;
  }
  /* 故障兜底——无法访问控制器时输出 0 V（零电压矢量），而不是
   * "保持上一次电压"。零矢量会让电机自然续流衰减，比维持错误电压更安全。
   * 两个指针都可以为 NULL，这里先判空再写，避免空指针解引用。 */

  control->id_feedback = id_feedback;
  control->iq_feedback = iq_feedback;
  /* 先把反馈写进结构体，纯粹是为了 Live Watch/上位机观察；
   * 真正的计算直接用形参，不依赖这些字段。转速反馈不同：应用层
   * 预先写入 speed_feedback 字段，本函数的计算直接读该字段。 */

  /* 仲裁结果由应用层在调用本函数前提交，这里只读取并规范化使能标志。 */
  speed_enabled = (control->reference.speed_loop_enable != 0U) ? 1U : 0U;
  /* 注意读的是 reference（仲裁结果），不是 volatile 的
   * control->speed_loop_enable（请求）。因此外部调用
   * FOC_Control_EnableSpeedLoop 之后还必须由应用层把新状态提交到 reference，
   * 模式才会真正改变。 */

  /*
   * 仲裁后的使能状态发生跳变时执行无扰模式切换：
   * 进入速度模式前按当前有效Iq预装速度PI，退出速度模式时把速度环输出
   * 保存回直接Iq命令。
   */
  /* 以下三行（leaving_speed_mode 与两个斜率快照）是 C99 的
   * "声明与语句混合"，位于函数中部。这样写是为了让局部变量在其首次使用的
   * 位置附近定义，减少阅读跨度；编译时仍然全部是栈分配，没有额外开销。 */
  uint8_t leaving_speed_mode = 0U;
  /* 标记"本拍刚刚从速度模式切换到电流模式"，用于后面的
   * iq_ref_target 选择——切换的当拍要沿着速度 PI 最后一次输出续上，
   * 而不是立刻跳到 reference.iq_ref（那会造成转矩断崖）。 */
  float speed_slew = control->speed_slew_rpm_per_s;
  /* 常规速度斜坡速率快照，单位 rpm/s（volatile，跨域可改）。 */
  float speed_target = control->speed_ref_rpm;
  /* 每拍只读取一次外部目标，事件检测与斜坡使用同一份命令。 */
  float iq_slew = control->reference.iq_slew_limit;
  /* 本拍 Iq 速率上限，单位 A/s，只有 >0 且有限时才生效。 */
  if ((control->reference.speed_slew_limit >= 0.0f) &&
      (speed_slew > control->reference.speed_slew_limit)) {
    speed_slew = control->reference.speed_slew_limit;
  }
  /* A falling target must start braking promptly.  Keep a user supplied
   * speed_slew value authoritative; only the default acceleration slope is
   * replaced by the faster deceleration slope. */
  if (isfinite(speed_target) && isfinite(control->speed_ref_previous_rpm) &&
      isfinite(control->speed_feedback) &&
      ((control->speed_feedback > 0.0f && speed_target < control->speed_feedback) ||
       (control->speed_feedback < 0.0f && speed_target > control->speed_feedback))) {
    if ((control->reference.speed_slew_limit < 0.0f) &&
        (speed_slew >=
         (FOC_SPEED_REFERENCE_SLEW_RPM_PER_S_DEFAULT - 0.001f)) &&
        (speed_slew < FOC_SPEED_DECEL_REFERENCE_SLEW_RPM_PER_S_DEFAULT)) {
      speed_slew = FOC_SPEED_DECEL_REFERENCE_SLEW_RPM_PER_S_DEFAULT;
    }
  }
  /* 应用层覆盖规则——speed_slew_limit >= 0 时才参与限制，且只取
   * "更小者"（限制只能收紧，不能用它把速率放大）。取 0 就得到 speed_slew = 0，
   * 于是 FOC_SlewSpeedReference 冻结参考（用于启动期"钉住"种子转速）。 */

  if (speed_enabled != control->speed_loop_enable_last) {
    control->speed_loop_counter = 0U;
    /* 模式切换当拍清零分频计数器——保证新模式下从完整窗口重新开始，
     * 不会因为沿用旧计数而在切换后立刻（或迟迟不）跑速度环。 */
    control->iq_ref_target = control->iq_ref_active;
    /* 先让本拍 Iq 目标从"当前实际 Iq"续起，形成一个无扰基线；
     * 速度模式下随后会被速度 PI 的输出覆盖，电流模式下会在下面的 else 分支
     * 里保持（或使用回写值）。 */
    if (speed_enabled != 0U) {
      /* Preserve signed torque current; do not multiply by direction again.
       * 保留带符号的转矩电流，不要再次乘以方向。
       * 原因是这里的 iq_ref_active 已经是"最终施加值"（可能来自电流模式下的
       * 仲裁参考，也可能来自速度 PI），若再按某个方向标志反号就会把制动力矩
       * 变成驱动力矩。反转运行由应用层通过 id_ref/iq_ref 的符号和角度处理，
       * 控制器不再引入第二处符号翻转。 */
      control->speed_ref_active_rpm = control->speed_feedback;
      control->speed_ref_previous_rpm = speed_target;
      /* 把斜坡内部状态直接设为当前实测转速，使速度 PI 接管瞬间
       * "参考=反馈"，误差为 0，避免参考从旧值（例如 1500 rpm）跳到实测值
       * 造成一次巨大的误差冲击。 */
      PI_Controller_PreloadOutput(&control->speed_pi, control->iq_ref_active,
                                  control->speed_feedback, control->speed_feedback);
      /* 用当前 Iq 作为期望输出预装速度 PI。因为 reference 与
       * feedback 都传实测转速，误差为 0、比例项为 0，所以等价于把速度 PI 的
       * 积分直接设成 iq_ref_active —— 速度环接管的第一拍输出就等于原来的 Iq，
       * 转矩连续。这是 OPEN_LOOP_IF -> OBSERVER_HANDOVER 切换不产生冲击的关键。 */
    } else {
      control->iq_ref = control->iq_ref_active;
      /* 退出速度模式时把速度 PI 最后产出的 Iq 回写到外部命令字段
       * iq_ref，使"电流模式"从当前的转矩水平接管。回写的是 volatile 字段，
       * 因此上位机随后读到的命令值就是实际生效值，不会出现"显示与执行不一致"。 */
      leaving_speed_mode = 1U;
      /* 置位标记，供本拍后面选择 iq_ref_target 时使用。 */
    }
    control->speed_loop_enable_last = speed_enabled;
    /* 更新边沿检测状态，保证同一个方向只在跳变的那一拍执行一次
     * 切换动作（否则每拍都会重新预装，速度 PI 永远无法积累积分）。 */
  }

  if (speed_enabled != 0U) {
    /* 减速/反转新命令撤销旧推进历史。参考若仍领先于实速，继续从旧
     * 参考往下爬会先维持正向转矩；已在制动一侧的参考则沿用原轨迹。
     * 这里只处理目标变化事件，不在恒定命令下反复清积分，也不改变
     * 进入速度模式时的无扰预装或 slew=0 的冻结语义。 */
    if ((speed_target != control->speed_ref_previous_rpm) &&
        isfinite(speed_target) && isfinite(control->speed_ref_previous_rpm) &&
        isfinite(control->speed_feedback) && isfinite(control->speed_ref_active_rpm) &&
        isfinite(speed_slew) && (speed_slew > 0.0f) &&
        (((control->speed_feedback > 0.0f) && (speed_target < control->speed_feedback)) ||
         ((control->speed_feedback < 0.0f) && (speed_target > control->speed_feedback)))) {
      if (((control->speed_feedback > 0.0f) &&
           (control->speed_ref_active_rpm > control->speed_feedback)) ||
          ((control->speed_feedback < 0.0f) &&
           (control->speed_ref_active_rpm < control->speed_feedback))) {
        control->speed_ref_active_rpm = control->speed_feedback;
      }
      if (((control->speed_feedback > 0.0f) && (control->speed_pi.integral > 0.0f)) ||
          ((control->speed_feedback < 0.0f) && (control->speed_pi.integral < 0.0f))) {
        control->speed_pi.integral = 0.0f;
      }
      /* iq_ref_active、iq_ref_target 和电流 PI 保持原值，后续仍通过
       * 原有 Iq 变化率限制卸下转矩，禁止在此直接跳变实际电流参考。 */
    }
    control->speed_ref_previous_rpm = speed_target;
    /* 速度模式分支。注意速度参考的斜坡用的是 id_pi.sample_time
     * （= 电流环周期 4e-5 s）而不是速度环周期，因此 25 kHz 每拍都在推进参考，
     * 阶跃被细分成 25 段/ms；速度 PI 本身仍然是 1 kHz。 */
    control->speed_ref_active_rpm = FOC_SlewSpeedReference(
        control->speed_ref_active_rpm, speed_target,
        speed_slew, control->id_pi.sample_time);
    speed_divider = control->speed_loop_divider;
    /* 速度环分频比快照（uint16_t，25 默认）。 */
    if (speed_divider == 0U) speed_divider = 1U;
    /* 防御性保护——若参数被错误下发成 0，按 1 处理（速度环变成
     * 25 kHz）。这样最坏情况是速度环过快、动态互相激励，而不是计数器永远
     * 无法触发导致速度环彻底不运行（那会直接失去速度控制）。 */
    control->speed_loop_counter++;
    if (control->speed_loop_counter >= speed_divider) {
      float requested;
      /* 速度 PI 本拍的原始输出（Iq 请求），单位 A，已含其自身 ±10 A 限幅。 */
      float reachable;
      /* 在下游 Iq 速率限制下真正可达的 Iq，单位 A。默认等于 requested。 */
      control->speed_loop_counter = 0U;
      /* 先归零再计算，保证即使下面的 PI 调用出现异常，计数器也不会
       * 溢出（uint16_t 虽然能计到 65535，但保持"到点即清"的语义更清晰）。 */
      float integral_before = control->speed_pi.integral;
      float speed_error = control->speed_ref_active_rpm - control->speed_feedback;
      float current_error = control->iq_ref_active - iq_feedback;
      /* 电压已打满且电流仍跟不上时，禁止速度积分继续要求同方向转矩。
       * 误差反号立即允许退积分；不降低电压、转速目标或 Iq 软件上限。 */
      control->speed_voltage_limited =
          (speed_error * current_error > 0.0f) &&
          (((control->iq_pi.saturation == PI_SATURATION_HIGH) && (speed_error > 0.0f)) ||
           ((control->iq_pi.saturation == PI_SATURATION_LOW) && (speed_error < 0.0f)));
      requested = PI_Controller_Run(&control->speed_pi,
          control->speed_ref_active_rpm, control->speed_feedback);
      if (control->speed_voltage_limited != 0U) {
        PI_Controller_PreloadOutput(&control->speed_pi,
            control->speed_pi.proportional + integral_before,
            control->speed_ref_active_rpm, control->speed_feedback);
        requested = control->speed_pi.output;
      }
      reachable = requested;
      if ((iq_slew > 0.0f) && isfinite(iq_slew)) {
        reachable = FOC_SlewSpeedReference(control->iq_ref_active, requested,
            iq_slew, control->id_pi.sample_time * (float)speed_divider);
        /* 复用了"速度斜坡"函数来限制 Iq，但步长用的是**速度环周期**
         * （电流环周期 × 分频比 = 1 ms），因为本次计算的是一毫秒内的变化量。
         * 若这里误用电流环周期，等效 Iq 加速度会被放大 25 倍。 */
        if ((reachable != requested) && (reachable * speed_error >= 0.0f)) {
          /* Track the realizable Iq target only when the downstream rate
           * limiter is active and its torque does not oppose the speed error.
           * During deceleration the old propulsive current takes time to slew
           * away; preloading it would recreate the integral just withdrawn.
           * 只有当下游速率限制器真正起作用时，才去
           * 跟踪"可达的 Iq 目标"。这样可以防止速度积分器累积饱和。
           * 若可达电流仍与速度误差反向，说明旧推进电流尚未卸完，禁止
           * 把它回装为新的推进积分；原积分继续朝制动方向消退。
           * reachable == requested 说明限速没削掉任何东西，
           * 此时不需要回装（回装反而会多算一次误差、浪费指令）。
           * 回装用的是同一拍的参考与反馈，所以误差、比例项与刚才
           * PI_Controller_Run 内部算出的一致，回装纯粹是把积分改写成"可达值"
           * 对应的状态，从而让速度 PI 下一拍从更低的目标继续。 */
          PI_Controller_PreloadOutput(&control->speed_pi, reachable,
              control->speed_ref_active_rpm, control->speed_feedback);
        }
      }
      control->iq_ref_target = reachable;
      /* 速度环的最终 Iq 目标。注意这个赋值只在分频到点的那一拍发生，
       * 其余 24 拍 iq_ref_target 保持不变（即速度环输出是 1 kHz 的阶梯信号）。 */
    }
  } else {
    control->speed_voltage_limited = 0U;
    control->iq_ref_target = (leaving_speed_mode != 0U)
        ? control->iq_ref_active : control->reference.iq_ref;
    /* 电流模式（含 ALIGN、OPEN_LOOP_IF 的开环 I/F 阶段）的 Iq 目标：
     * - 刚从速度模式切出来的那一拍，沿用 iq_ref_active（保证转矩连续）；
     * - 其余情况直接使用应用层仲裁提交的 reference.iq_ref。
     * 这里刻意不用 volatile 的 control->iq_ref，因为那是"人工命令值"，
     * 启动状态机可能希望给出与命令不同的受限电流。 */
  }

  if ((iq_slew > 0.0f) && isfinite(iq_slew)) {
    control->iq_ref_active = FOC_SlewSpeedReference(
        control->iq_ref_active, control->iq_ref_target,
        iq_slew, control->id_pi.sample_time);
    /* 把目标 Iq 限速成实际施加值，步长用**电流环周期**（4e-5 s），
     * 因为这一步在 25 kHz 每拍都执行。启动阶段用 iq_slew_limit 来平滑
     * ALIGN->I/F->闭环的转矩电流突加，避免电流冲击和观测器扰动。
     * 注意这条限速对速度环路径也生效：速度环给出的 iq_ref_target 会被它
     * 进一步平滑成 iq_ref_active，这就是上面"可达值回装"存在的原因。 */
  } else {
    control->iq_ref_active = control->iq_ref_target;
    /* 不限速路径（iq_slew <= 0 或非有限值）——直接跟随目标。
     * 上电默认 iq_slew_limit = 0，所以默认就是这条"无延迟"路径。 */
  }

  /*
   * 线性SVPWM的最大dq电压矢量为Vbus/sqrt(3)，按配置用满线性区。
   */
  /* 母线电压有效性检查与限幅换算。
   * 非有限值（NaN/Inf，通常来自采样通道异常或 ADC 未就绪）或 <=0 时把限幅置 0：
   * 结果是两个电流环的输出被限成 0 V，控制器"有意识地不输出"，比用一个错误
   * 的大电压去驱动电机安全得多。换算公式右上方的两个宏的物理来源见 controller.h。
   * 举例：Vbus = 24 V 时 voltage_limit = 24*0.57735 ≈ 13.86 V，即 dq 电压
   * 矢量长度不超过 13.86 V，对应相电压峰值不超过 Vbus/√3。 */
  if ((!isfinite(dc_bus_voltage)) || (dc_bus_voltage <= 0.0f)) {
    voltage_limit = 0.0f;
  } else {
    voltage_limit = dc_bus_voltage * FOC_INV_SQRT3_DEFAULT *
                    FOC_VOLTAGE_UTILIZATION_DEFAULT;
  }
  control->voltage_limit = voltage_limit;
  /* 写回结构体供应用层与上位机观察，也是"本拍允许了多少电压"的
   * 唯一记录。 */

  /*
   * 优先保证d轴电流调节，再把圆形电压矢量中剩余的幅值分配给q轴。
   * 两个PI直接使用最终可实现的限幅，饱和时条件积分能够及时停止，
   * 不再依赖SVPWM末端缩放来掩盖固定正负20 V造成的积分饱和。
   * dq参考分别取仲裁结果的id_ref和速度环/电流模式产生的iq_ref_active。
   */
  /* 电流环执行顺序——d 轴优先（先给它完整的 ±voltage_limit 额度），
   * q 轴只能使用圆形限幅剩下的部分。为什么 d 轴优先：d 轴电流在有凸极性时
   * 影响磁链与观测器估计，把 d 轴控制好对观测精度更重要；同时 q 轴是转矩轴，
   * 被削一点电压只是损失转矩，不会破坏角度估计。
   * 与"先固定限幅 + SVPWM 末端缩放"的旧方案相比，这里让 PI 直接使用最终可实现
   * 的限幅，条件积分抗饱和能及时生效，不会再出现积分饱和被末端缩放掩盖的问题。 */
  PI_Controller_SetLimits(&control->id_pi, -voltage_limit, voltage_limit);
  /* 每拍重设限幅——既跟随母线电压变化，也把上面积分旋转/上拍遗留的
   * 积分夹回合法范围。 */
  control->ud_output =
      PI_Controller_Run(&control->id_pi, control->reference.id_ref, id_feedback);
  /* d 轴参考来自仲裁结果（ALIGN 阶段为定位电流，闭环通常为 0 A），
   * 反馈是 Park 变换后的 id。返回值即限幅后的 Ud，单位 V。 */

  uq_limit_squared = voltage_limit * voltage_limit -
                     control->ud_output * control->ud_output;
  /* 圆形限幅的核心——dq 电压矢量长度不得超过 voltage_limit，
   * 因此 q 轴可用的平方额度 = voltage_limit² - Ud²。 */
  if ((uq_limit_squared <= 0.0f) ||
      (arm_sqrt_f32(uq_limit_squared, &uq_limit) != ARM_MATH_SUCCESS)) {
    uq_limit = 0.0f;
  }
  /* 两种退化情况都置 0：
   *   - uq_limit_squared <= 0：d 轴已经用满（甚至超出）整个矢量额度，q 轴无
   *     额度可用，只能给 0 V；置 0 而不是给负数再开方（会得到 NaN）；
   *   - arm_sqrt_f32 返回非 ARM_MATH_SUCCESS：CMSIS-DSP 对非法输入（NaN/负数）
   *     的显式失败返回，此时 uq_limit 的内容不可信，必须丢弃并置 0。
   * 置 0 的后果是 q 轴电压被限成 ±0 V，转矩瞬间消失——这是"电压预算耗尽"的
   * 可见表现，宁可丢转矩也不输出错误电压。 */

  PI_Controller_SetLimits(&control->iq_pi, -uq_limit, uq_limit);
  control->uq_output =
      PI_Controller_Run(&control->iq_pi, control->iq_ref_active, iq_feedback);
  /* q 轴参考是前面整条 Iq 链路（速度 PI / 电流模式 / 限速）算出的
   * iq_ref_active，反馈是 Park 后的 iq。返回值即限幅后的 Uq，单位 V。
   * 注意 iq_pi 的限幅是不对称以外的对称区间 ±uq_limit，而 id_pi 的限幅是
   * ±voltage_limit——两者合起来仍然满足 |(Ud,Uq)| <= voltage_limit。 */

  if (ud_output != NULL) {
    *ud_output = control->ud_output;
  }

  if (uq_output != NULL) {
    *uq_output = control->uq_output;
  }
  /* 把结果回传给应用层（用于逆 Park + SVPWM）。两个指针都允许为
   * NULL——若应用层只关心结构体里的值就能省掉两次写入。 */
}

/* 设置速度环使能请求；模式切换和 PI 预加载在下一次控制周期处理。 */
/* 本函数可以被通信域（主循环里的协议解析）调用，所以它只写一个
 * volatile 字段，不做任何 PI 操作——避免跨域去动中断正在使用的积分状态。
 * 真正生效需要两步：应用层下一次仲裁时把新使能通过 FOC_Control_SubmitReference
 * 提交到 reference，然后 FOC_Control_Run 检测到边沿才执行无扰切换。
 * 参数 enable 规范化成 0（电流模式）或 1（速度模式），非 0 一律当 1。
 * 边界：control 为 NULL 直接返回。
 */
void FOC_Control_EnableSpeedLoop(FOC_Control_t *control, uint8_t enable) {
  if (control == NULL) {
    return;
  }

  control->speed_loop_enable = (enable != 0U) ? 1U : 0U;
}

/* A change of handover offset is a coordinate change, not a reset. Normal
 * per-cycle rotor rotation is deliberately excluded from this transport.
 * 握手偏置的变化是"坐标系变换"，不是复位。
 * 正常的逐拍转子旋转被刻意排除在这次搬运之外。
 * 背景：OBSERVER_HANDOVER 阶段控制角 = 观测器磁链角 + 一个渐消偏置；当偏置
 * 本身发生阶跃变化（例如应用层一次性把偏置清到新的初值）时，等效于坐标系被
 * 旋转了 Δ 角。此时两个电流 PI 的积分（它们是 dq 坐标系下的电压量）必须做同样
 * 的旋转，否则"原本指向某个空间方向的电压矢量"会在新坐标系里指错方向，造成
 * 一次电压/电流冲击。而每拍由转子转动带来的坐标系变化**不需要**在这里处理，
 * 因为 PI 的闭环（误差来自同一坐标系下的电流反馈）本来就会自动修正。
 * 参数：sin_delta/cos_delta 是偏置差角的三角函数值，无量纲；调用者必须保证
 * sin²+cos²≈1（本函数不校验），且两者都有效。
 * 数学实现是标准旋转矩阵，并且先把旧 d、q 读到局部变量再计算，因此两个轴的
 * 更新互不依赖（等价于同时更新）：
 *     d' = d*cosΔ - q*sinΔ
 *     q' = d*sinΔ + q*cosΔ
 * 变换后 (d,q) 的模长不变，只是相位偏移 Δ —— 与"电压矢量在空间中的方向不变、
 * 只换了描述它的坐标系"完全对应。
 * 边界与错误路径：control 为 NULL，或 sin/cos 任一为非有限值（NaN/Inf，通常
 * 意味着观测器失步或角度计算异常）时**直接返回且不改任何状态**——宁可不搬运，
 * 也不能把 NaN 灌进积分器（那会让控制器永久失效）。
 * 调用时机：应用层在切换偏置的那一拍、FOC_Control_Run 之前调用一次；本函数
 * 自身不会重设限幅，紧接着的 FOC_Control_Run 会用当前母线电压限幅把旋转后的
 * 积分夹回合法区间（源码末尾注释说明了这一点）。
 */
void FOC_Control_RotateCurrentIntegrals(FOC_Control_t *control,
                                       float sin_delta, float cos_delta) {
  float d; /* id_pi.integral 的旧值快照，单位 V。先读出来是为了保证 d'、q' 用的是同一拍的输入。 */
  float q; /* iq_pi.integral 的旧值快照，单位 V。 */
  if ((control == NULL) || !isfinite(sin_delta) || !isfinite(cos_delta)) return;
  /* 单行守卫——三个条件任一不满足就放弃搬运。注意这里用的是
   * "control == NULL" 判断，与前面几个函数的风格一致；也说明本模块对角度
   * 相关的输入一律采取"不合法就不动状态"的策略。 */
  d = control->id_pi.integral;
  q = control->iq_pi.integral;
  control->id_pi.integral = d * cos_delta - q * sin_delta;
  control->iq_pi.integral = d * sin_delta + q * cos_delta;
  /* 两式都只用 d、q 这两个局部快照，因此中间不存在"先更新 d 再用
   * 新 d 算 q"的顺序依赖，结果与同时旋转等价。 */
  /* FOC_Control_Run applies the current bus-dependent voltage limits next.
   * 接下来 FOC_Control_Run 会施加基于当前母线电压的
   * 电压限幅。也就是说旋转后的积分若超出新的 ±voltage_limit/±uq_limit，
   * 会在同一拍被 SetLimits 夹回，调用者无需在此手动限幅。 */
}
