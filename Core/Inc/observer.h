#ifndef OBSERVER_H
#define OBSERVER_H

#include "controller.h"
#include "stdint.h"

/**
 * @file observer.h
 * @brief 非线性磁链观测器与同步旋转坐标系PLL的数据结构和接口。
 * 电机参数、观测器参数、每拍输入、运行状态与总句柄集中在此声明；
 * 全部状态字段在25 kHz电流环中断内读写，主循环与CAN上报只读。
 * 单位约定：电阻ohm、电感H、磁链Wb、电压V、电流A、电角度rad、
 * 电角速度rad/s、机械转速rpm、时间s、频率Hz。
 */


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

  float Rs; /* 定子相电阻，ohm */
  float Ls; /* 定子相电感，H；alpha/beta两轴共用同一值 */
  float flux_linkage; /* 永磁磁链幅值，Wb */
  uint8_t pole_pairs; /* 极对数；注意观测器角度换算处未读取此字段 */

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

  /*
   * 速度上报低通滤波截止频率，Hz；<=0时滤波器退化为直通。
   * 用于上报值speed_rpm_f及启动速度匹配门。速度 PI 由应用层单独做
   * 100 Hz 单极点滤波；最低转速、方向和单拍跳变仍用原始speed_rpm。
   */
  float speed_filter_fc;

} Observer_Config_t;

/*
 * 速度上报滤波器：两级一阶低通级联，等效双极点。
 *
 * 每级按 y += k*(x-y) 更新，直流增益结构性为1，且系数无近抵消，
 * 稳态时speed_rpm_f与speed_rpm一致。
 * 不用单节二阶巴特沃斯的原因：fc/Ts很小(30Hz/25kHz)时其分母
 * 1+a1+a2≈5.7e-5，直接型结构的状态量化误差被放大约1.7万倍，
 * 恒转速下会钉死在偏差十几rpm的假平衡点。
 */
typedef struct {

  /*
   * 每级融合系数 k = w/(1+w)，w = 2*pi*fc*Ts。
   * 单级-3dB点在fc；级联后-6dB@fc、-40dB/dec。
   * fc<=0或Ts<=0时k=1，滤波器退化为直通。
   */
  float k;

  /* 第一节输出状态；第二节输出即speed_rpm_f。 */
  float y1;

} Observer_SpeedFilter_t;

/*
 * 观测器输入
 */
typedef struct {
  /*
   * PWM占空比
   * 0~1
   */
  float duty_a; /* A相PWM占空比，范围0~1 */
  float duty_b; /* B相PWM占空比，范围0~1 */
  float duty_c; /* C相PWM占空比，范围0~1；A/B/C对应U/V/W相 */

  /*
   * PCB三相端电压采样经Clarke变换后的电压。
   * measured_voltage_weight=1时使用实测值，=0时使用占空比重构值。
   */
  float measured_u_alpha; /* 实测三相端电压经Clarke变换后的alpha分量，V */
  float measured_u_beta; /* 实测三相端电压经Clarke变换后的beta分量，V */
  float measured_voltage_weight; /* 实测电压的融合权重，范围0~1；=1全用实测，=0全用占空比重构 */

  /*
   * 直流母线电压，V，用于占空比电压重构
   */
  float vbus; /* 直流母线电压，V；<=0.1V时本次观测直接返回 */
  /*
   * Clarke后的静止坐标电流，A
   */
  float i_alpha; /* 静止坐标alpha轴定子电流，A */
  float i_beta; /* 静止坐标beta轴定子电流，A */

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
  float x_alpha; /* 总磁链积分状态alpha分量，Wb */
  float x_beta; /* 总磁链积分状态beta分量，Wb */
  /*
   * 磁链估计
   */
  float psi_alpha; /* 永磁磁链估计alpha分量，Wb；= x - L*i */

  float psi_beta; /* 永磁磁链估计beta分量，Wb */

  /*
   * 磁链幅值平方误差乘以对应轴磁链，供非线性径向校正使用
   */
  float error_alpha; /* 幅值平方误差乘以alpha轴磁链，Wb^3量纲，供径向校正用 */

  float error_beta; /* 幅值平方误差乘以beta轴磁链，Wb^3量纲 */

  /*
   * 总磁链积分方程中的校正项，已乘观测器增益
   */
  float correction_alpha; /* alpha轴校正项，已乘观测器增益，Wb/s */

  float correction_beta; /* beta轴校正项，已乘观测器增益，Wb/s */

  /*
   * 估计角度
   *
   * atan2(beta,alpha)
   */
  float phase_raw; /* 磁链atan2原始电角度，rad，范围[-pi,pi] */

  float pll_phase; /* PLL积分电角度，rad，保持在[-pi,pi]范围 */
  float pll_phase_;//预留角度补偿字段，当前观测流程未更新
  float pll_omega_e; /* 最近一次有效磁链更新得到的电角速度，rad/s */
  float omega_m; /* 机械角速度，rad/s */
  float speed_rpm; /* 机械转速原始值，rpm；启动判据的快速保护门使用此值 */
  float speed_rpm_f; /* 机械转速滤波值，rpm；Ready速度匹配门与上报使用此值 */

  /*
   * speed_rpm_f的滤波器状态。
   * 磁链无效拍speed_rpm保持上一拍值，滤波器继续向该保持值收敛，
   * 因此上报值不会冻结。
   */
  Observer_SpeedFilter_t speed_filter;

  /*
   * 磁链大小
   */
  float psi_mag; /* 永磁磁链幅值，Wb；由arm_sqrt_f32开方得到 */

  /*
   * 初始化标志
   */
  uint8_t initialized; /* 1=已完成Observer_Init，0时Observer_Run直接返回 */

} Observer_State_t;

/*
 * 总观测器句柄
 */
typedef struct {

  Observer_MotorParam_t motor; /* 电机标定参数副本，Init时复制一次 */

  Observer_Config_t config; /* 观测器配置副本，Init时复制一次 */

  Observer_State_t state; /* 每拍更新的运行状态 */

  /*
   * 复用已有PI控制器：
   * pll.error为相位误差，pll.output为电角速度。
   */
  PI_Controller_t pll; /* 复用通用PI做SRF-PLL环路滤波；限幅为±pll_omega_limit */

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
