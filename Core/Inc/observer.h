#ifndef OBSERVER_H
#define OBSERVER_H

#include "controller.h"
#include "stdint.h"


/*
 * 电机参数
 *
 * PMSM:
 *
 * Rs      定子相电阻，ohm
 * Ls      定子相电感，H；当前模型使用同一电感处理alpha/beta两轴
 * flux    永磁磁链幅值，Wb
 * pole_pairs 极对数
 */
typedef struct {

  float Rs;

  float Ls;

  float flux_linkage;

  uint8_t pole_pairs;

} Observer_MotorParam_t;

/*
 * 观测器参数
 */
typedef struct {

  /*
   * 观测器增益
   *
   * gamma
   */
  float gain;

  /*
   * 观测器调用周期，s；与电流中断周期一致
   */
  float Ts;

  /*
   * PLL接受磁链反馈的幅值范围，Wb；不是磁链状态的硬限幅
   */
  float psi_min;

  float psi_max;

  /* 经典PLL参数 */
  float pll_kp;

  float pll_ki;

  /* PLL输出电角速度限幅，单位rad/s */
  float pll_omega_limit;

} Observer_Config_t;

/*
 * 观测器输入
 */
typedef struct {
  /*
   * PWM占空比
   * 0~1
   */
  float duty_a;
  float duty_b;
  float duty_c;

  /*
   * PCB三相端电压采样经Clarke变换后的电压。
   * measured_voltage_weight=1时使用实测值，=0时使用占空比重构值。
   */
  float measured_u_alpha;
  float measured_u_beta;
  float measured_voltage_weight;

  /*
   * 直流母线电压，V，用于占空比电压重构
   */
  float vbus;
  /*
   * Clarke后的静止坐标电流，A
   */
  float i_alpha;
  float i_beta;

} Observer_Input_t;

/*
 * 观测器状态
 */
typedef struct {

  /*
   * 定子总磁链观测器状态：
   *
   * x = L*i + psi_f
   */
  float x_alpha;
  float x_beta;
  /*
   * 磁链估计
   */
  float psi_alpha;

  float psi_beta;

  /*
   * 磁链幅值平方误差乘以对应轴磁链，供非线性径向校正使用
   */
  float error_alpha;

  float error_beta;

  /*
   * 总磁链积分方程中的校正项，已乘观测器增益
   */
  float correction_alpha;

  float correction_beta;

  /*
   * 估计角度
   *
   * atan2(beta,alpha)
   */
  float phase_raw;

  float pll_phase; /* PLL积分电角度，rad，保持在[-pi,pi]范围 */
  float pll_phase_;//预留角度补偿字段，当前观测流程未更新
  float pll_omega_e; /* 最近一次有效磁链更新得到的电角速度，rad/s */
  //机械角速度，rad/s
  float omega_m;
  //机械转速  单位  rpm
  float speed_rpm;

  /*
   * 磁链大小
   */
  float psi_mag;

  /*
   * 初始化标志
   */
  uint8_t initialized;

} Observer_State_t;

/*
 * 总观测器句柄
 */
typedef struct {

  Observer_MotorParam_t motor;

  Observer_Config_t config;

  Observer_State_t state;

  /*
   * 复用已有PI控制器：
   * pll.error为相位误差，pll.output为电角速度。
   */
  PI_Controller_t pll;

} Observer_Handle_t;

/**
 * @brief 初始化观测器：复制电机参数和配置，设置初始磁链为alpha轴正方向。
 * @param obs    观测器句柄指针，保存运行时状态。
 * @param motor  电机参数（Rs、Ls、磁链、极对数），只读。
 * @param config 观测器配置（增益、周期、PLL参数），只读。
 */

void Observer_Init(Observer_Handle_t *obs, const Observer_MotorParam_t *motor,
                   const Observer_Config_t *config);

/**
 * @brief 每控制周期调用一次：融合电压、积分磁链、提取原始角度，再更新PLL。
 * @param obs    观测器句柄指针。
 * @param input  本拍输入（占空比、实测电压、权重、母线、电流），只读。
 */
void Observer_Run(Observer_Handle_t *obs, const Observer_Input_t *input);

/**
 * @brief 执行SRF-PLL鉴相和速度估计，由Observer_Run内部调用。
 * @note 独立调用时须先准备有效的磁链状态及已初始化句柄。
 * @param obs 观测器句柄指针。
 */
void Observer_PLL_Run(Observer_Handle_t *obs);

#endif
