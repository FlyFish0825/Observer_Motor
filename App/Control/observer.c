#include "observer.h"
#include "arm_math.h"
#include "foc_math.h"
#include <stddef.h>

/**
 * @brief 初始化滑模磁链观测器、PLL参数和磁链状态，建立电机模型初值。
 */
void Observer_Init(Observer_Handle_t *obs, const Observer_MotorParam_t *motor,
                   const Observer_Config_t *config) {

  obs->motor.Rs = motor->Rs;
  obs->motor.Ls = motor->Ls;
  obs->motor.flux_linkage = motor->flux_linkage;
  obs->motor.pole_pairs = motor->pole_pairs;

  obs->config.gain = config->gain;
  obs->config.Ts = config->Ts;
  obs->config.psi_min = config->psi_min;
  obs->config.psi_max = config->psi_max;
  obs->config.pll_kp = config->pll_kp;
  obs->config.pll_ki = config->pll_ki;
  obs->config.pll_omega_limit = config->pll_omega_limit;

  /*
   * 初始磁链
   */

  obs->state.x_alpha = motor->flux_linkage;
  obs->state.x_beta = 0.0f;

  obs->state.psi_alpha = motor->flux_linkage;
  obs->state.psi_beta = 0.0f;

  obs->state.psi_mag = motor->flux_linkage;

  obs->state.error_alpha = 0.0f;
  obs->state.error_beta = 0.0f;

  obs->state.correction_alpha = 0.0f;
  obs->state.correction_beta = 0.0f;

  obs->state.phase_raw = 0.0f;
  obs->state.pll_phase = 0.0f;

  PI_Controller_Init(&obs->pll, config->pll_kp, config->pll_ki, config->Ts,
                     -config->pll_omega_limit, config->pll_omega_limit);

  obs->state.initialized = 1U;
}

__STATIC_FORCEINLINE void
/**
 * @brief 根据三相占空比和真实母线电压重构静止坐标系下的α/β电压。
 */
Observer_RebuildVoltage(const Observer_Input_t *input,
                        float *u_alpha,
                        float *u_beta)
{
  float vbus;

  vbus = input->vbus;

  /*
   * 原式先计算三相相电压：
   *   Ua=(Da-0.5)*Vbus
   * 再做Clarke变换。
   *
   * 展开后0.5公共项会完全抵消，因此可直接由占空比重构：
   *   Ualpha = Vbus * (2Da-Db-Dc) / 3
   *   Ubeta  = Vbus * (Db-Dc) / sqrt(3)
   *
   * 数学结果相同，但减少3次减法和多次浮点乘法。
   */
  *u_alpha =
      vbus *
      (2.0f * input->duty_a - input->duty_b - input->duty_c) *
      FOC_ONE_THIRD_F;

  *u_beta =
      vbus * (input->duty_b - input->duty_c) * FOC_INV_SQRT3_F;
}

/**
 * @brief 非线性磁链观测器
 *
 * 输入:
 *
 *      u_alpha/beta
 *      i_alpha/beta
 *
 * 输出:
 *
 *      psi_alpha
 *      psi_beta
 *
 */
void Observer_Run(Observer_Handle_t *obs, const Observer_Input_t *input) {
  float Rs; /* 电机定子电阻模型参数。 */
  float Ls; /* 电机定子电感模型参数。 */
  float Ts; /* 离散积分步长。 */

  float u_alpha; /* 由占空比重构的α轴电压。 */
  float u_beta; /* 由占空比重构的β轴电压。 */

  float x_alpha; /* 上一拍定子总磁链α状态。 */
  float x_beta; /* 上一拍定子总磁链β状态。 */

  float psi_alpha; /* 去除电感磁链后的永磁磁链α分量。 */
  float psi_beta; /* 去除电感磁链后的永磁磁链β分量。 */

  float psi_mag_sq; /* 永磁磁链幅值的平方，避免中间开方。 */
  float error; /* 目标磁链幅值平方与当前值之差。 */

  float correction_alpha; /* α轴非线性校正量。 */
  float correction_beta; /* β轴非线性校正量。 */

  if ((obs == NULL) || (input == NULL)) {
    return;
  }

  if ((obs->state.initialized == 0U) || (obs->config.Ts <= 0.0f) ||
      (input->vbus <= 0.1f)) {
    return;
  }

  Rs = obs->motor.Rs;
  Ls = obs->motor.Ls;
  Ts = obs->config.Ts;

  /*
   * 由最终SVPWM占空比和母线电压
   * 重构实际施加的Ualpha、Ubeta。
   */
  Observer_RebuildVoltage(input, &u_alpha, &u_beta);

  /*
   * 读取上一拍的定子总磁链状态。
   */
  x_alpha = obs->state.x_alpha;
  x_beta = obs->state.x_beta;

  /*
   * 从定子总磁链中减去电感磁链：
   *
   * psi_f = x - L*i
   *
   * 这里得到的才是永磁体磁链，
   * 也是后面atan2真正应该使用的量。
   */
  psi_alpha = x_alpha - Ls * input->i_alpha;

  psi_beta = x_beta - Ls * input->i_beta;

  /*
   * 永磁磁链幅值误差。
   */
  psi_mag_sq = psi_alpha * psi_alpha + psi_beta * psi_beta;

  error = obs->motor.flux_linkage * obs->motor.flux_linkage - psi_mag_sq;

  /*
   * 非线性径向校正。
   *
   * psi太小时error>0，向外修正；
   * psi太大时error<0，向内修正。
   */
  obs->state.error_alpha = error * psi_alpha;

  obs->state.error_beta = error * psi_beta;

  correction_alpha = obs->config.gain * obs->state.error_alpha;

  correction_beta = obs->config.gain * obs->state.error_beta;

  /*
   * 更新定子总磁链状态：
   *
   * x_dot = u - R*i + correction
   */
  x_alpha += Ts * (u_alpha - Rs * input->i_alpha + correction_alpha);

  x_beta += Ts * (u_beta - Rs * input->i_beta + correction_beta);

  /*
   * 使用更新后的状态重新计算永磁磁链。
   */
  psi_alpha = x_alpha - Ls * input->i_alpha;

  psi_beta = x_beta - Ls * input->i_beta;

  psi_mag_sq = psi_alpha * psi_alpha + psi_beta * psi_beta;

  if (arm_sqrt_f32(psi_mag_sq, &obs->state.psi_mag) != ARM_MATH_SUCCESS) {
    obs->state.psi_mag = 0.0f;
  }

  /*
   * 保存总磁链积分状态。
   */
  obs->state.x_alpha = x_alpha;
  obs->state.x_beta = x_beta;

  /*
   * 保存永磁磁链输出。
   */
  obs->state.psi_alpha = psi_alpha;
  obs->state.psi_beta = psi_beta;

  obs->state.correction_alpha = correction_alpha;

  obs->state.correction_beta = correction_beta;

  obs->state.phase_raw = FOC_atan2_Fast(psi_beta, psi_alpha);
  Observer_PLL_Run(obs);
}

/*
 * @brief 经典磁链SRF-PLL
 *
 * 磁链有效：
 *   使用psi_q修正速度
 *
 * 磁链无效：
 *   保持上一拍速度继续推算角度
 */
void Observer_PLL_Run(Observer_Handle_t *obs) {
  float psi_alpha_n; /* 归一化永磁磁链α分量。 */
  float psi_beta_n; /* 归一化永磁磁链β分量。 */

  float inv_psi_mag; /* 磁链幅值倒数，复用以减少除法。 */
  float pll_error; /* SRF-PLL鉴相误差。 */
  float omega_e; /* 本拍沿用或更新的电角速度。 */

  int32_t pll_phase_q31; /* PLL角度对应的CORDIC Q1.31值。 */

  uint8_t psi_valid; /* 当前磁链幅值是否允许参与PLL校正。 */

  /*
   * 判断当前磁链幅值是否可信。
   */
  psi_valid = (obs->state.psi_mag > 1.0e-9f) &&
              ((obs->config.psi_min <= 0.0f) ||
               (obs->state.psi_mag >= obs->config.psi_min)) &&
              ((obs->config.psi_max <= 0.0f) ||
               (obs->state.psi_mag <= obs->config.psi_max));
         

  /*
   * 先保留上一拍PLL估计速度。
   *
   * 即使这一拍磁链无效，也不能让角度停止。
   */
  omega_e = obs->pll.output;

  if (psi_valid != 0U) {
    /*
     * 磁链归一化。
     */
    /* 一次除法得到倒数，替代原来的两次浮点除法。 */
    inv_psi_mag = 1.0f / obs->state.psi_mag;

    psi_alpha_n = obs->state.psi_alpha * inv_psi_mag;
    psi_beta_n = obs->state.psi_beta * inv_psi_mag;

    /*
     * 根据PLL估计角度计算sin/cos。
     */
    /* pll_phase始终已经保持在[-pi, pi]，无需再次循环归一化。 */
    pll_phase_q31 = CORDIC_RadToQ31_WrappedFast(obs->state.pll_phase);

    CORDIC_SinCos_FastF32(pll_phase_q31, &observer_sin_cos.sin,
                          &observer_sin_cos.cos);

    /*
     * SRF-PLL鉴相器：
     *
     * error = psi_q
     *       = sin(theta - theta_hat)
     */
    pll_error =
        -psi_alpha_n * observer_sin_cos.sin + psi_beta_n * observer_sin_cos.cos;

    /*
     * PI输出估计电角速度。
     */
    omega_e = PI_Controller_RunError(&obs->pll, pll_error);
    obs->state.pll_omega_e = omega_e;

    /* 常数乘法替代每拍浮点除法。 */
    obs->state.omega_m = omega_e * (1.0f / 7.0f);
    obs->state.speed_rpm =
        omega_e * (60.0f / (2.0f * FOC_PI * 7.0f));
  }

  /*
   * 无论磁链这一拍是否有效，
   * 都必须让PLL角度继续前进。
   */
  obs->state.pll_phase =
      FOC_WrapToPiFast(obs->state.pll_phase + omega_e * obs->config.Ts);
}
