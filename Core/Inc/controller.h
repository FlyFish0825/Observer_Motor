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
 * @brief PI控制器数据结构
 */
typedef struct {
  /* ==================== 可调参数 ==================== */

  /**
   * 比例系数
   */
  volatile float kp;

  /**
   * 积分系数
   *
   * 控制器内部计算：
   * integral += ki * error * sample_time
   */
  volatile float ki;

  /**
   * 控制周期，单位：s
   */
  volatile float sample_time;

  /**
   * PI总输出限幅
   */
  volatile float output_min;
  volatile float output_max;

  /**
   * 积分项限幅
   */
  volatile float integral_min;
  volatile float integral_max;

  /* ==================== 运行状态 ==================== */

  float reference;
  float feedback;
  float error;

  float proportional;
  float integral;

  float output_unsaturated;
  float output;

  PI_Saturation_t saturation;

} PI_Controller_t;

/**
 * @brief 初始化PI控制器
 */
void PI_Controller_Init(PI_Controller_t *pi, float kp, float ki,
                        float sample_time, float output_min, float output_max);

/**
 * @brief 运行PI控制器
 *
 * @return PI输出
 */
float PI_Controller_Run(PI_Controller_t *pi, float reference, float feedback);

/**
 * @brief 已知误差时运行PI控制器
 */
float PI_Controller_RunError(PI_Controller_t *pi, float error);

/**
 * @brief 清空PI运行状态
 */
void PI_Controller_Reset(PI_Controller_t *pi);

/**
 * @brief 修改Kp和Ki
 */
void PI_Controller_SetGains(PI_Controller_t *pi, float kp, float ki);

/**
 * @brief 修改采样周期
 */
void PI_Controller_SetSampleTime(PI_Controller_t *pi, float sample_time);

/**
 * @brief 同时设置输出限幅和积分限幅
 */
void PI_Controller_SetLimits(PI_Controller_t *pi, float minimum, float maximum);

/**
 * @brief 单独设置输出限幅
 */
void PI_Controller_SetOutputLimits(PI_Controller_t *pi, float minimum,
                                   float maximum);

/**
 * @brief 单独设置积分限幅
 */
void PI_Controller_SetIntegralLimits(PI_Controller_t *pi, float minimum,
                                     float maximum);

/**
 * @brief PI输出预装载
 *
 * 用于开环切换到闭环，防止输出发生突变。
 *
 * desired_output：切换前的实际输出
 */
void PI_Controller_PreloadOutput(PI_Controller_t *pi, float desired_output,
                                 float reference, float feedback);

#ifdef __cplusplus
}
#endif

#endif