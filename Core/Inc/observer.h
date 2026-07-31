#ifndef OBSERVER_H
#define OBSERVER_H

#include "controller.h"
#include "stdint.h"


/*
 * 电机参数
 *
 * PMSM:
 *
 * Rs      定子电阻
 * Ls      定子电感
 * flux    永磁磁链
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
   * 采样周期
   */
  float Ts;

  /*
   * 磁链幅值限制
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
   * 母线电压
   */
  float vbus;
  /*
   * Clarke后的电流
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
   * 电流误差
   */
  float error_alpha;

  float error_beta;

  /*
   * 校正量
   */
  float correction_alpha;

  float correction_beta;

  /*
   * 估计角度
   *
   * atan2(beta,alpha)
   */
  float phase_raw;

  float pll_phase;
  float pll_omega_e;
  //机械角速度
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

void Observer_Init(Observer_Handle_t *obs, const Observer_MotorParam_t *motor,
                   const Observer_Config_t *config);

void Observer_Run(Observer_Handle_t *obs, const Observer_Input_t *input);

void Observer_PLL_Run(Observer_Handle_t *obs);
#endif