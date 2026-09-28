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
  PI_SATURATION_NONE = 0, /* 输出在上下限之间。 */
  PI_SATURATION_LOW = -1, /* 输出被下限截断。 */
  PI_SATURATION_HIGH = 1 /* 输出被上限截断。 */
} PI_Saturation_t;

/**
 * @brief 通用PI控制器数据结构
 */
typedef struct {
  /* 可调参数 */
  volatile float kp; /* 比例增益。 */
  volatile float ki; /* 积分增益，积分项按采样周期累加。 */
  volatile float sample_time; /* 控制器采样周期，单位 s。 */

  volatile float output_min; /* 控制器最终输出下限。 */
  volatile float output_max; /* 控制器最终输出上限。 */

  volatile float integral_min; /* 积分项独立下限，用于抗积分饱和。 */
  volatile float integral_max; /* 积分项独立上限。 */

  /* 运行状态 */
  float reference; /* 本次运行使用的目标值。 */
  float feedback; /* 本次运行使用的反馈值。 */
  float error; /* reference-feedback。 */

  float proportional; /* 比例通道输出。 */
  float integral; /* 当前积分状态。 */

  float output_unsaturated; /* 限幅前输出，用于诊断和饱和判断。 */
  float output; /* 限幅后的实际输出。 */

  PI_Saturation_t saturation; /* 本次输出的饱和方向。 */
} PI_Controller_t;

/* ======================== 电机闭环默认参数 ======================== */

/*
 * 电流环参数保持当前工程中的数值不变。
 * 电流环每次ADC注入转换完成时运行，当前频率25kHz。
 */
#define FOC_ID_PI_KP_DEFAULT 0.2f
#define FOC_ID_PI_KI_DEFAULT 100.0f
#define FOC_ID_PI_OUTPUT_MIN_DEFAULT (-20.0f)
#define FOC_ID_PI_OUTPUT_MAX_DEFAULT 20.0f

#define FOC_IQ_PI_KP_DEFAULT 0.5f
#define FOC_IQ_PI_KI_DEFAULT 300.0f
#define FOC_IQ_PI_OUTPUT_MIN_DEFAULT (-20.0f)
#define FOC_IQ_PI_OUTPUT_MAX_DEFAULT 20.0f

/*
 * 速度环先使用一组偏保守的初值，后续可在线微调。
 * 单位：
 *   speed PI输入  = rpm
 *   speed PI输出  = Iq参考值，A
 */
#define FOC_SPEED_PI_KP_DEFAULT 0.001f
#define FOC_SPEED_PI_KI_DEFAULT 0.05f
#define FOC_SPEED_PI_OUTPUT_MIN_DEFAULT (-0.5f)
#define FOC_SPEED_PI_OUTPUT_MAX_DEFAULT 0.5f

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

  /* 外部命令，可由串口实时修改 */
  volatile float id_ref; /* d轴电流给定，单位 A。 */
  volatile float iq_ref; /* q轴电流给定，单位 A。 */
  volatile float speed_ref_rpm; /* 速度给定，单位 rpm。 */
  volatile uint32_t speed_loop_enable; /* 非0时由速度PI接管Iq给定。 */

  /* 调度参数和内部状态 */
  uint32_t speed_loop_enable_last; /* 用于检测电流/速度模式切换。 */
  uint16_t speed_loop_divider; /* 电流环周期数/一次速度环周期。 */
  uint16_t speed_loop_counter; /* 当前速度环分频计数。 */

  /* 反馈量，便于Live Watch和上位机观察 */
  float id_feedback; /* d轴电流反馈，单位 A。 */
  float iq_feedback; /* q轴电流反馈，单位 A。 */
  float speed_feedback_rpm; /* 机械速度反馈，单位 rpm。 */

  /* 速度环最终产生的有效Iq参考值 */
  float iq_ref_active; /* 当前模式实际送入q轴PI的给定。 */

  /* 电流环输出电压 */
  float ud_output; /* d轴电压PI输出。 */
  float uq_output; /* q轴电压PI输出。 */
} FOC_Control_t;

/* ======================== 通用PI接口 ======================== */

void PI_Controller_Init(PI_Controller_t *pi, float kp, float ki,
                        float sample_time, float output_min, float output_max);

float PI_Controller_Run(PI_Controller_t *pi, float reference, float feedback);

float PI_Controller_RunError(PI_Controller_t *pi, float error);

void PI_Controller_Reset(PI_Controller_t *pi);

void PI_Controller_SetGains(PI_Controller_t *pi, float kp, float ki);

void PI_Controller_SetSampleTime(PI_Controller_t *pi, float sample_time);

void PI_Controller_SetLimits(PI_Controller_t *pi, float minimum, float maximum);

void PI_Controller_SetOutputLimits(PI_Controller_t *pi, float minimum,
                                   float maximum);

void PI_Controller_SetIntegralLimits(PI_Controller_t *pi, float minimum,
                                     float maximum);

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
 * @brief 开环切闭环前预装载控制器，减小Ud/Uq突变
 */
void FOC_Control_PreloadClosedLoop(FOC_Control_t *control, float desired_ud,
                                   float desired_uq, float id_feedback,
                                   float iq_feedback, float speed_feedback_rpm);

/**
 * @brief 每个电流环周期调用一次
 *
 * 速度模式下，函数内部自动按speed_loop_divider运行速度PI；
 * 电流模式下，直接使用id_ref和iq_ref。
 */
void FOC_Control_Run(FOC_Control_t *control, float id_feedback,
                     float iq_feedback, float speed_feedback_rpm,
                     float *ud_output, float *uq_output);

/**
 * @brief 切换电流模式/速度模式
 * @param enable 0=电流模式，非0=速度模式
 */
void FOC_Control_EnableSpeedLoop(FOC_Control_t *control, uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif
