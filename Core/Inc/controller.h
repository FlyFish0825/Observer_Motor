#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PI输出饱和状态
 */
typedef enum {
  PI_SATURATION_NONE = 0,
  PI_SATURATION_LOW = -1,
  PI_SATURATION_HIGH = 1
} PI_Saturation_t;

/**
 * @brief 通用PI控制器数据结构
 */
typedef struct {
  /* 可调参数：Kp为输出/输入单位，Ki为输出/(输入*s)，sample_time单位s。
   * volatile保证运行时重新读取，不保证多个参数能作为一个事务同时更新。
   */
  volatile float kp;
  volatile float ki;
  volatile float sample_time;

  volatile float output_min;
  volatile float output_max;

  volatile float integral_min;
  volatile float integral_max;

  /* 运行状态：integral保存已乘Ki并积分后的输出贡献，单位与output相同。 */
  float reference;
  float feedback;
  float error;

  float proportional;
  float integral;

  float output_unsaturated;
  float output;

  PI_Saturation_t saturation;
} PI_Controller_t;


/* ======================== 电机闭环默认参数 ======================== */

/*
 * 电流环参数采用相同0.5 ohm、100 uH电机分支中已使用的保守初值。
 * 电流环每次ADC注入转换完成时运行，当前频率25kHz。
 * 电流PI误差单位A、输出单位V。运行时按实时母线电压建立dq圆形限幅，
 * 不再使用固定的正负20 V独立轴限幅。
 */
#define FOC_ID_PI_KP_DEFAULT 0.220f
#define FOC_ID_PI_KI_DEFAULT 1257.0f

#define FOC_IQ_PI_KP_DEFAULT 0.220f
#define FOC_IQ_PI_KI_DEFAULT 1257.0f

/* 线性SVPWM最大dq矢量为Vbus/sqrt(3)，保留2%调制和死区裕量。 */
#define FOC_VOLTAGE_UTILIZATION_DEFAULT 0.98f
#define FOC_INV_SQRT3_DEFAULT 0.57735026919f

/*
 * 速度环输出单位为Iq参考值(A)。
 * 当前硬件电流采样：5 mOhm分流电阻、24倍模拟增益、1.65 V中点偏置、3.3 V ADC。
 * 理论双向量程约为 +/-13.75 A；10 A相电流峰值时ADC输入约为0.45~2.85 V，
 * 两端仍各保留约0.45 V裕量，可覆盖正常纹波和小幅瞬态。
 *
 * 注意：这里的+/-10 A只是速度环允许请求的Iq软件上限，不等同于硬件过流保护值。
 * 提升该上限的目的是避免水下大负载时原5 A限幅过早限制可用电磁转矩。
 */
#define FOC_SPEED_PI_KP_DEFAULT 0.0005f
#define FOC_SPEED_PI_KI_DEFAULT 0.005f
#define FOC_SPEED_PI_OUTPUT_MIN_DEFAULT (-10.0f)
#define FOC_SPEED_PI_OUTPUT_MAX_DEFAULT 10.0f

/* 运行中的外部速度阶跃转换成斜坡，默认每秒最多变化20000 rpm。 */
#define FOC_SPEED_REFERENCE_SLEW_RPM_PER_S_DEFAULT 20000.0f

/* 25kHz电流环 / 25 = 1kHz速度环 */
#define FOC_SPEED_LOOP_DIVIDER_DEFAULT 25U


/**
 * @brief FOC电流环和速度环总控制器
 *
 * 工作方式：
 * 1. speed_loop_enable=0：电流模式，iq_ref直接作为Iq给定。
 * 2. speed_loop_enable=1：速度模式，速度PI输出iq_ref_active。
 * 3. Id、Iq电流PI每个电流环周期都运行。
 * 4. 速度PI按照speed_loop_divider分频运行。
 */
typedef struct {
  /* 三个PI控制器 */
  PI_Controller_t id_pi;
  PI_Controller_t iq_pi;
  PI_Controller_t speed_pi;

  /* 外部命令，可由串口实时修改。
   * id_ref/iq_ref单位A；speed_command_rpm为控制台目标，应用层复制给
   * speed_ref_rpm，控制器再生成speed_ref_active_rpm作为实际PI参考。
   */
  volatile float id_ref;
  volatile float iq_ref;
  volatile float speed_command_rpm;
  volatile float speed_ref_rpm;
  volatile float speed_slew_rpm_per_s;
  volatile uint32_t speed_loop_enable;

  /* 调度参数和内部状态；修改分频时须同步速度PI的sample_time。 */
  uint32_t speed_loop_enable_last;
  uint16_t speed_loop_divider;
  uint16_t speed_loop_counter;

  /* 反馈量，便于Live Watch和上位机观察 */
  float id_feedback;
  float iq_feedback;
  float speed_feedback_rpm;

  /* 速度斜坡后的内部参考，仅在正常速度闭环中使用 */
  float speed_ref_active_rpm;

  /* 速度环最终产生的有效Iq参考值 */
  float iq_ref_active;

  /* 电流环输出电压 */
  float ud_output;
  float uq_output;

  /* 根据实时母线电压得到的dq电压矢量上限，单位V。 */
  float voltage_limit;
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
 */
void PI_Controller_Init(PI_Controller_t *pi, float kp, float ki,
                        float sample_time, float output_min, float output_max);

/**
 * @brief 给定参考和反馈值，计算一次PI输出。
 * @param pi        PI控制器指针。
 * @param reference 参考值（目标）。
 * @param feedback  反馈值（实测）。
 * @return 限幅后的PI输出。
 */
float PI_Controller_Run(PI_Controller_t *pi, float reference, float feedback);

/**
 * @brief 直接使用误差值计算一次PI输出（适用于PLL等已计算好误差的场景）。
 * @param pi     PI控制器指针。
 * @param error  本拍误差（= reference - feedback）。
 * @return 限幅后的PI输出。
 */
float PI_Controller_RunError(PI_Controller_t *pi, float error);

/**
 * @brief 清除PI运行状态（误差、积分、输出），保留增益和限幅配置。
 * @param pi PI控制器指针。
 */
void PI_Controller_Reset(PI_Controller_t *pi);

/**
 * @brief 运行时更新比例和积分增益，不改变积分历史。
 * @param pi   PI控制器指针。
 * @param kp   新的比例增益。
 * @param ki   新的积分增益。
 */
void PI_Controller_SetGains(PI_Controller_t *pi, float kp, float ki);

/**
 * @brief 运行时更新离散积分的采样周期。
 * @param pi          PI控制器指针。
 * @param sample_time 新的采样周期（s），负值被钳为0。
 */
void PI_Controller_SetSampleTime(PI_Controller_t *pi, float sample_time);

/**
 * @brief 同时设置输出和积分限幅，并立即将已有状态修正到新范围。
 * @param pi      PI控制器指针。
 * @param minimum 新的下限（自动与maximum交换若顺序颠倒）。
 * @param maximum 新的上限。
 */
void PI_Controller_SetLimits(PI_Controller_t *pi, float minimum, float maximum);

/**
 * @brief 仅设置PI输出限幅；积分限幅保持不变。
 * @param pi      PI控制器指针。
 * @param minimum 输出下限。
 * @param maximum 输出上限。
 */
void PI_Controller_SetOutputLimits(PI_Controller_t *pi, float minimum,
                                   float maximum);

/**
 * @brief 仅设置积分项限幅，并将当前积分值夹到新范围。
 * @param pi      PI控制器指针。
 * @param minimum 积分下限。
 * @param maximum 积分上限。
 */
void PI_Controller_SetIntegralLimits(PI_Controller_t *pi, float minimum,
                                     float maximum);

/**
 * @brief 按期望输出反算积分状态，用于模式切换时无扰预加载。
 * @param pi             PI控制器指针。
 * @param desired_output 期望本拍输出的值（将被限幅）。
 * @param reference     当前参考值，用于计算比例项。
 * @param feedback      当前反馈值，用于计算比例项。
 */
void PI_Controller_PreloadOutput(PI_Controller_t *pi, float desired_output,
                                 float reference, float feedback);

/* ======================== FOC电流环/速度环接口 ======================== */

/**
 * @brief 初始化电流环和速度环
 * @param current_loop_sample_time 电流环周期，当前工程传0.00004f
 */
void FOC_Control_Init(FOC_Control_t *control, float current_loop_sample_time);

/**
 * @brief 清空三个PI的运行状态，不修改Kp、Ki和命令值
 */
void FOC_Control_Reset(FOC_Control_t *control);

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
 */
void FOC_Control_Run(FOC_Control_t *control, float id_feedback,
                     float iq_feedback, float speed_feedback_rpm,
                     float dc_bus_voltage, float *ud_output,
                     float *uq_output);

/**
 * @brief 切换电流模式/速度模式
 * @param enable 0=电流模式，非0=速度模式
 */
void FOC_Control_EnableSpeedLoop(FOC_Control_t *control, uint8_t enable);


#ifdef __cplusplus
}
#endif

#endif
