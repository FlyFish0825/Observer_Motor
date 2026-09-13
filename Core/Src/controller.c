/**
 * @file controller.c
 * @brief 位置式PI和FOC串级控制：速度PI给出Iq参考，电流PI给出Ud/Uq。
 * Ki使用连续时间增益，离散积分在每次运算时显式乘sample_time。
 * 电流环按40us运行，速度环默认25分频，即1ms运行一次。
 */
#include "controller.h"

#include "arm_math.h"
#include <stddef.h>

/**
 * @brief 浮点限幅
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
 */
static float FOC_SlewSpeedReference(float current, float target,
                                    float slew_rpm_per_s,
                                    float sample_time) {
  float maximum_step;
  float delta;

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
 */
static void PI_NormalizeLimits(float *minimum, float *maximum) {
  float temporary;

  if ((*minimum) > (*maximum)) {
    temporary = *minimum;
    *minimum = *maximum;
    *maximum = temporary;
  }
}

/**
 * @brief 配置增益、积分周期及限幅，并清空历史运行状态。
 * 输出与积分默认共用同一限幅区间；后续可通过各自设置接口单独调整。
 */
void PI_Controller_Init(PI_Controller_t *pi, float kp, float ki,
                        float sample_time, float output_min, float output_max) {
  if (pi == NULL) {
    return;
  }

  PI_NormalizeLimits(&output_min, &output_max);

  if (sample_time < 0.0f) {
    sample_time = 0.0f;
  }

  pi->kp = kp;
  pi->ki = ki;
  pi->sample_time = sample_time;

  pi->output_min = output_min;
  pi->output_max = output_max;

  /*
   * 默认让积分限幅等于输出限幅。
   */
  pi->integral_min = output_min;
  pi->integral_max = output_max;

  PI_Controller_Reset(pi);
}

/* 仅清除误差、积分和输出历史，保留增益、周期与限幅参数。 */
void PI_Controller_Reset(PI_Controller_t *pi) {
  if (pi == NULL) {
    return;
  }

  pi->reference = 0.0f;
  pi->feedback = 0.0f;
  pi->error = 0.0f;

  pi->proportional = 0.0f;
  pi->integral = 0.0f;

  pi->output_unsaturated = 0.0f;
  pi->output = 0.0f;

  pi->saturation = PI_SATURATION_NONE;
}

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
 */
float PI_Controller_RunError(PI_Controller_t *pi, float error) {
  float kp;
  float ki;
  float sample_time;

  float output_min;
  float output_max;

  float integral_min;
  float integral_max;

  float integral_old;
  float integral_candidate;

  float output_unsaturated;
  float output;

  if (pi == NULL) {
    return 0.0f;
  }

  /*
   * 先读取到局部变量，避免一次计算中参数被重复读取。
   */
  kp = pi->kp;
  ki = pi->ki;
  sample_time = pi->sample_time;

  output_min = pi->output_min;
  output_max = pi->output_max;

  integral_min = pi->integral_min;
  integral_max = pi->integral_max;

  pi->error = error;

  /*
   * 比例项
   */
  pi->proportional = kp * error;

  /*
   * 计算积分候选值
   */
  integral_old = pi->integral;

  integral_candidate = integral_old + ki * error * sample_time;

  integral_candidate = PI_Clamp(integral_candidate, integral_min, integral_max);

  /*
   * 使用新的积分值计算未限幅输出。
   */
  output_unsaturated = pi->proportional + integral_candidate;

  /*
   * 条件积分抗饱和。
   *
   * 先决定是否撤销本拍积分，最后只执行一次输出限幅。
   * 此处判断的是单个PI输出限幅，不包含后级SVPWM的电压缩放反馈。
   */
  if (((output_unsaturated > output_max) && (error > 0.0f)) ||
      ((output_unsaturated < output_min) && (error < 0.0f))) {
    integral_candidate = integral_old;
    output_unsaturated = pi->proportional + integral_old;
  }

  output = PI_Clamp(output_unsaturated, output_min, output_max);

  pi->integral = integral_candidate;

  pi->output_unsaturated = output_unsaturated;

  pi->output = output;

  if (output_unsaturated > output_max) {
    pi->saturation = PI_SATURATION_HIGH;
  } else if (output_unsaturated < output_min) {
    pi->saturation = PI_SATURATION_LOW;
  } else {
    pi->saturation = PI_SATURATION_NONE;
  }

  return output;
}

void PI_Controller_SetGains(PI_Controller_t *pi, float kp, float ki) {
  if (pi == NULL) {
    return;
  }

  pi->kp = kp;
  pi->ki = ki;
}

void PI_Controller_SetSampleTime(PI_Controller_t *pi, float sample_time) {
  if (pi == NULL) {
    return;
  }

  if (sample_time < 0.0f) {
    sample_time = 0.0f;
  }

  pi->sample_time = sample_time;
}

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
 */
void PI_Controller_PreloadOutput(PI_Controller_t *pi, float desired_output,
                                 float reference, float feedback) {
  float output_min;
  float output_max;

  if (pi == NULL) {
    return;
  }

  output_min = pi->output_min;
  output_max = pi->output_max;

  desired_output = PI_Clamp(desired_output, output_min, output_max);

  pi->reference = reference;
  pi->feedback = feedback;
  pi->error = reference - feedback;

  pi->proportional = pi->kp * pi->error;

  /*
   * desired_output = proportional + integral
   *
   * 所以：
   * integral = desired_output - proportional
   */
  pi->integral = desired_output - pi->proportional;

  pi->integral = PI_Clamp(pi->integral, pi->integral_min, pi->integral_max);

  pi->output_unsaturated = pi->proportional + pi->integral;

  pi->output = PI_Clamp(pi->output_unsaturated, output_min, output_max);

  if (pi->output_unsaturated > output_max) {
    pi->saturation = PI_SATURATION_HIGH;
  } else if (pi->output_unsaturated < output_min) {
    pi->saturation = PI_SATURATION_LOW;
  } else {
    pi->saturation = PI_SATURATION_NONE;
  }
}

/* ======================== FOC电流环/速度环 ======================== */

void FOC_Control_Init(FOC_Control_t *control, float current_loop_sample_time) {
  float speed_loop_sample_time;

  if (control == NULL) {
    return;
  }

  if (current_loop_sample_time <= 0.0f) {
    current_loop_sample_time = 0.00004f;
  }

  control->speed_loop_divider = FOC_SPEED_LOOP_DIVIDER_DEFAULT;

  speed_loop_sample_time =
      current_loop_sample_time * (float)control->speed_loop_divider;

  /* Id电流环使用新电机分支中已有的默认参数。 */
  PI_Controller_Init(&control->id_pi, FOC_ID_PI_KP_DEFAULT,
                     FOC_ID_PI_KI_DEFAULT, current_loop_sample_time, 0.0f,
                     0.0f);

  /* Iq电流环与Id环使用相同参数，匹配当前表贴式电机模型。 */
  PI_Controller_Init(&control->iq_pi, FOC_IQ_PI_KP_DEFAULT,
                     FOC_IQ_PI_KI_DEFAULT, current_loop_sample_time, 0.0f,
                     0.0f);

  /* 速度环输出为Iq参考值，使用偏保守参数并保留正负5 A限幅。 */
  PI_Controller_Init(&control->speed_pi, FOC_SPEED_PI_KP_DEFAULT,
                     FOC_SPEED_PI_KI_DEFAULT, speed_loop_sample_time,
                     FOC_SPEED_PI_OUTPUT_MIN_DEFAULT,
                     FOC_SPEED_PI_OUTPUT_MAX_DEFAULT);

  control->id_ref = 0.0f;
  control->iq_ref = 0.20f;
  control->speed_command_rpm = 1500.0f;
  control->speed_ref_rpm = 1500.0f;
  control->speed_slew_rpm_per_s =
      FOC_SPEED_REFERENCE_SLEW_RPM_PER_S_DEFAULT;

  /* 上电默认使用速度闭环。 */
  control->speed_loop_enable = 1U;
  control->speed_loop_enable_last = 0U;

  control->speed_loop_counter = 0U;

  control->id_feedback = 0.0f;
  control->iq_feedback = 0.0f;
  control->speed_feedback_rpm = 0.0f;
  control->speed_ref_active_rpm = control->speed_ref_rpm;

  control->iq_ref_active = control->iq_ref;

  control->ud_output = 0.0f;
  control->uq_output = 0.0f;
  control->voltage_limit = 0.0f;

}

void FOC_Control_Reset(FOC_Control_t *control) {
  if (control == NULL) {
    return;
  }

  PI_Controller_Reset(&control->id_pi);
  PI_Controller_Reset(&control->iq_pi);
  PI_Controller_Reset(&control->speed_pi);

  control->speed_loop_counter = 0U;
  control->speed_loop_enable_last =
      (control->speed_loop_enable != 0U) ? 1U : 0U;

  control->id_feedback = 0.0f;
  control->iq_feedback = 0.0f;
  control->speed_feedback_rpm = 0.0f;
  control->speed_ref_active_rpm = control->speed_ref_rpm;

  control->iq_ref_active = control->iq_ref;

  control->ud_output = 0.0f;
  control->uq_output = 0.0f;
  control->voltage_limit = 0.0f;
}

void FOC_Control_Run(FOC_Control_t *control, float id_feedback,
                     float iq_feedback, float speed_feedback_rpm,
                     float dc_bus_voltage, float *ud_output,
                     float *uq_output) {
  float uq_limit_squared;
  float uq_limit;
  float voltage_limit;
  uint32_t speed_enabled;
  uint16_t speed_divider;

  if (control == NULL) {
    if (ud_output != NULL) {
      *ud_output = 0.0f;
    }

    if (uq_output != NULL) {
      *uq_output = 0.0f;
    }

    return;
  }

  control->id_feedback = id_feedback;
  control->iq_feedback = iq_feedback;
  control->speed_feedback_rpm = speed_feedback_rpm;

  speed_enabled = (control->speed_loop_enable != 0U) ? 1U : 0U;

  /*
   * 自动识别串口或代码直接修改speed_loop_enable的情况，
   * 并完成无扰模式切换。
   */
  if (speed_enabled != control->speed_loop_enable_last) {
    control->speed_loop_counter = 0U;

    if (speed_enabled != 0U) {
      /*
       * 电流模式 -> 速度模式：
       * 启动接管时直接同步目标转速，不经过运行调速斜坡；
       * 速度PI第一拍输出保持当前有效Iq参考值。
       */
      control->speed_ref_active_rpm = control->speed_ref_rpm;
      PI_Controller_PreloadOutput(&control->speed_pi, control->iq_ref_active,
                                  control->speed_ref_active_rpm,
                                  speed_feedback_rpm);
    } else {
      /*
       * 速度模式 -> 电流模式：
       * 将当前速度环Iq输出保存为新的直接Iq命令，避免跳变。
       */
      control->iq_ref = control->iq_ref_active;
    }

    control->speed_loop_enable_last = speed_enabled;
  }

  if (speed_enabled != 0U) {
    /* 只有已经进入正常速度闭环后，外部阶跃才转换为斜坡。 */
    control->speed_ref_active_rpm = FOC_SlewSpeedReference(
        control->speed_ref_active_rpm, control->speed_ref_rpm,
        control->speed_slew_rpm_per_s, control->id_pi.sample_time);

    speed_divider = control->speed_loop_divider;

    if (speed_divider == 0U) {
      speed_divider = 1U;
    }

    control->speed_loop_counter++;

    /* 分频间隔内保持上一次Iq参考；电流PI仍在每一拍运行。 */
    if (control->speed_loop_counter >= speed_divider) {
      control->speed_loop_counter = 0U;

      control->iq_ref_active = PI_Controller_Run(
          &control->speed_pi, control->speed_ref_active_rpm,
          speed_feedback_rpm);
    }
  } else {
    control->iq_ref_active = control->iq_ref;
  }

  /*
   * 线性SVPWM的最大dq电压矢量为Vbus/sqrt(3)。保留2%裕量，
   * 避免死区、母线纹波和ADC刷新延迟使占空比长期贴住0%或100%。
   */
  if ((!isfinite(dc_bus_voltage)) || (dc_bus_voltage <= 0.0f)) {
    voltage_limit = 0.0f;
  } else {
    voltage_limit = dc_bus_voltage * FOC_INV_SQRT3_DEFAULT *
                    FOC_VOLTAGE_UTILIZATION_DEFAULT;
  }
  control->voltage_limit = voltage_limit;

  /*
   * 优先保证d轴电流调节，再把圆形电压矢量中剩余的幅值分配给q轴。
   * 两个PI直接使用最终可实现的限幅，饱和时条件积分能够及时停止，
   * 不再依赖SVPWM末端缩放来掩盖固定正负20 V造成的积分饱和。
   */
  PI_Controller_SetLimits(&control->id_pi, -voltage_limit, voltage_limit);
  control->ud_output =
      PI_Controller_Run(&control->id_pi, control->id_ref, id_feedback);

  uq_limit_squared = voltage_limit * voltage_limit -
                     control->ud_output * control->ud_output;
  if ((uq_limit_squared <= 0.0f) ||
      (arm_sqrt_f32(uq_limit_squared, &uq_limit) != ARM_MATH_SUCCESS)) {
    uq_limit = 0.0f;
  }

  PI_Controller_SetLimits(&control->iq_pi, -uq_limit, uq_limit);
  control->uq_output =
      PI_Controller_Run(&control->iq_pi, control->iq_ref_active, iq_feedback);

  if (ud_output != NULL) {
    *ud_output = control->ud_output;
  }

  if (uq_output != NULL) {
    *uq_output = control->uq_output;
  }
}

void FOC_Control_EnableSpeedLoop(FOC_Control_t *control, uint8_t enable) {
  if (control == NULL) {
    return;
  }

  control->speed_loop_enable = (enable != 0U) ? 1U : 0U;
}
