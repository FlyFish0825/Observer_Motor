#ifndef OBSERVER_H
#define OBSERVER_H

/* 本文件描述手写磁链观测器的数据契约；计算实现位于 observer.c。 */

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

  float Rs; /* 定子电阻，单位 ohm。 */

  float Ls; /* 等效定子电感，单位 H。 */

  float flux_linkage; /* 永磁体磁链幅值，单位 Wb。 */

  uint8_t pole_pairs; /* 极对数，用于电角速度与机械速度换算。 */

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
  float gain; /* 非线性磁链径向校正增益。 */

  /*
   * 采样周期
   */
  float Ts; /* 观测器离散步长，单位 s。 */

  /*
   * 磁链幅值限制
   */
  float psi_min; /* 允许PLL使用的磁链幅值下限。 */

  float psi_max; /* 允许PLL使用的磁链幅值上限。 */

  /* 经典PLL参数 */
  float pll_kp; /* SRF-PLL比例增益。 */

  float pll_ki; /* SRF-PLL积分增益。 */

  /* PLL输出电角速度限幅，单位rad/s */
  float pll_omega_limit; /* PLL电角速度输出绝对值上限，单位 rad/s。 */

} Observer_Config_t;

/*
 * 观测器输入
 */
typedef struct {
  /*
   * PWM占空比
   * 0~1
   */
  float duty_a; /* 最终SVPWM A相占空比，范围通常为0..1。 */
  float duty_b; /* 最终SVPWM B相占空比。 */
  float duty_c; /* 最终SVPWM C相占空比。 */
  /*
   * 母线电压
   */
  float vbus; /* 实时母线电压，单位 V。 */
  /*
   * Clarke后的电流
   */
  float i_alpha; /* α轴电流，单位 A。 */
  float i_beta; /* β轴电流，单位 A。 */

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
  float x_alpha; /* 积分得到的定子总磁链α分量，单位 Wb。 */
  float x_beta; /* 积分得到的定子总磁链β分量。 */
  /*
   * 磁链估计
   */
  float psi_alpha; /* 去除L*i后的永磁磁链α分量。 */

  float psi_beta; /* 去除L*i后的永磁磁链β分量。 */

  /*
   * 电流误差
   */
  float error_alpha; /* 磁链幅值误差投影到α轴。 */

  float error_beta; /* 磁链幅值误差投影到β轴。 */

  /*
   * 校正量
   */
  float correction_alpha; /* 非线性校正项α分量。 */

  float correction_beta; /* 非线性校正项β分量。 */

  /*
   * 估计角度
   *
   * atan2(beta,alpha)
   */
  float phase_raw; /* 当前永磁磁链的atan2原始角，单位 rad。 */

  float pll_phase; /* PLL连续推进后的电角度，单位 rad。 */
  float pll_omega_e; /* PLL估计电角速度，单位 rad/s。 */
  //机械角速度
  float omega_m; /* 机械角速度，单位 rad/s。 */
  //机械转速  单位  rpm
  float speed_rpm; /* 机械转速，单位 rpm。 */

  /*
   * 磁链大小
   */
  float psi_mag; /* 永磁磁链幅值，单位 Wb。 */

  /*
   * 初始化标志
   */
  uint8_t initialized; /* 1表示初始化完成，允许运行观测器。 */

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
