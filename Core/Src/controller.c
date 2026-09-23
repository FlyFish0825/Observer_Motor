#include "controller.h"

#include <stddef.h>

/**
 * @brief 浮点限幅
 */
static inline float PI_Clamp(float value, float minimum, float maximum) {
  /* 统一保证控制器输出或积分状态落在调用方给出的闭区间内。 */
  if (value > maximum) {
    return maximum;
  }

  if (value < minimum) {
    return minimum;
  }

  return value;
}

/**
 * @brief 修正限幅参数顺序
 */
static void PI_NormalizeLimits(float *minimum, float *maximum) {
  float temporary;

  /* 配置接口允许上下限反向传入，内部先恢复为 minimum <= maximum。 */
  if ((*minimum) > (*maximum)) {
    temporary = *minimum;
    *minimum = *maximum;
    *maximum = temporary;
  }
}

/**
 * @brief 初始化PI控制器的增益、采样周期、输出限幅和积分限幅，并清零运行状态。
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

/**
 * @brief 清除PI控制器的历史误差、积分项、输出和饱和状态，恢复到安全初始状态。
 */
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

/**
 * @brief 根据给定参考值和反馈值计算误差，并运行一次PI控制器。
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
 * @brief 使用已计算的误差运行PI控制器，执行比例、条件积分抗饱和及输出限幅。
 */
float PI_Controller_RunError(PI_Controller_t *pi, float error) {
  float kp; /* 本次计算使用的比例增益快照。 */
  float ki; /* 本次计算使用的积分增益快照。 */
  float sample_time; /* 本次积分换算所用周期。 */

  float output_min;
  float output_max;

  float integral_min;
  float integral_max;

  float integral_old; /* 上一拍积分状态。 */
  float integral_candidate; /* 未执行条件积分回退前的候选状态。 */

  float output_unsaturated; /* 比例项与候选积分项之和。 */
  float output; /* 最终限幅输出。 */

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
   * 原实现会先限幅一次；发生积分阻塞时又重新计算并限幅一次。
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

/**
 * @brief 在线更新PI控制器的比例增益和积分增益。
 */
void PI_Controller_SetGains(PI_Controller_t *pi, float kp, float ki) {
  if (pi == NULL) {
    return;
  }

  pi->kp = kp;
  pi->ki = ki;
}

/**
 * @brief 在线更新PI控制器采样周期，负值会被钳位为零。
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

/**
 * @brief 同时设置输出和积分限幅，并将现有积分、输出裁剪到新范围。
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

/**
 * @brief 只更新输出限幅，不改变积分限幅配置。
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

/**
 * @brief 只更新积分限幅，并立即修正当前积分值。
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
 * @brief 预装PI内部积分状态，使闭环切换时输出平滑且避免突变。
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

  /*
   * 保留当前已经跑通的Id电流环参数。
   */
  PI_Controller_Init(&control->id_pi, FOC_ID_PI_KP_DEFAULT,
                     FOC_ID_PI_KI_DEFAULT, current_loop_sample_time,
                     FOC_ID_PI_OUTPUT_MIN_DEFAULT,
                     FOC_ID_PI_OUTPUT_MAX_DEFAULT);

  /*
   * 保留当前已经跑通的Iq电流环参数。
   */
  PI_Controller_Init(&control->iq_pi, FOC_IQ_PI_KP_DEFAULT,
                     FOC_IQ_PI_KI_DEFAULT, current_loop_sample_time,
                     FOC_IQ_PI_OUTPUT_MIN_DEFAULT,
                     FOC_IQ_PI_OUTPUT_MAX_DEFAULT);

  /*
   * 速度环输出为Iq参考值，先使用偏保守的参数和电流限幅。
   */
  PI_Controller_Init(&control->speed_pi, FOC_SPEED_PI_KP_DEFAULT,
                     FOC_SPEED_PI_KI_DEFAULT, speed_loop_sample_time,
                     FOC_SPEED_PI_OUTPUT_MIN_DEFAULT,
                     FOC_SPEED_PI_OUTPUT_MAX_DEFAULT);

  control->id_ref = 0.0f;
  control->iq_ref = 0.20f;
  control->speed_ref_rpm = 1500.0f;

  /* 默认仍然保持原工程的电流模式。 */
  control->speed_loop_enable = 1U;
  control->speed_loop_enable_last = 0U;

  control->speed_loop_counter = 0U;

  control->id_feedback = 0.0f;
  control->iq_feedback = 0.0f;
  control->speed_feedback_rpm = 0.0f;

  control->iq_ref_active = control->iq_ref;

  control->ud_output = 0.0f;
  control->uq_output = 0.0f;
}

/**
 * @brief 复位FOC控制器的三个PI环及运行时状态，供停机或重新启动时使用。
 */
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

  control->iq_ref_active = control->iq_ref;

  control->ud_output = 0.0f;
  control->uq_output = 0.0f;
}

/**
 * @brief 在进入闭环前预装电流环和速度环状态，实现无扰切换。
 */
void FOC_Control_PreloadClosedLoop(FOC_Control_t *control, float desired_ud,
                                   float desired_uq, float id_feedback,
                                   float iq_feedback,
                                   float speed_feedback_rpm) {
  if (control == NULL) {
    return;
  }

  control->id_feedback = id_feedback;
  control->iq_feedback = iq_feedback;
  control->speed_feedback_rpm = speed_feedback_rpm;

  /*
   * 若切换时已经选择速度模式，先让速度PI当前输出等于原Iq命令，
   * 防止速度环接管瞬间改变Iq参考值。
   */
  if (control->speed_loop_enable != 0U) {
    PI_Controller_PreloadOutput(&control->speed_pi, control->iq_ref,
                                control->speed_ref_rpm, speed_feedback_rpm);

    control->iq_ref_active = control->speed_pi.output;
  } else {
    control->iq_ref_active = control->iq_ref;
  }

  PI_Controller_PreloadOutput(&control->id_pi, desired_ud, control->id_ref,
                              id_feedback);

  PI_Controller_PreloadOutput(&control->iq_pi, desired_uq,
                              control->iq_ref_active, iq_feedback);

  control->ud_output = control->id_pi.output;
  control->uq_output = control->iq_pi.output;

  control->speed_loop_counter = 0U;
  control->speed_loop_enable_last =
      (control->speed_loop_enable != 0U) ? 1U : 0U;
}

/**
 * @brief 执行一次FOC控制计算；速度环按分频运行，Id/Iq电流环每个PWM周期运行。
 */
void FOC_Control_Run(FOC_Control_t *control, float id_feedback,
                     float iq_feedback, float speed_feedback_rpm,
                     float *ud_output, float *uq_output) {
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
       * 速度PI第一拍输出保持当前有效Iq参考值。
       */
      PI_Controller_PreloadOutput(&control->speed_pi, control->iq_ref_active,
                                  control->speed_ref_rpm, speed_feedback_rpm);
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
    speed_divider = control->speed_loop_divider;

    if (speed_divider == 0U) {
      speed_divider = 1U;
    }

    control->speed_loop_counter++;

    if (control->speed_loop_counter >= speed_divider) {
      control->speed_loop_counter = 0U;

      control->iq_ref_active = PI_Controller_Run(
          &control->speed_pi, control->speed_ref_rpm, speed_feedback_rpm);
    }
  } else {
    control->iq_ref_active = control->iq_ref;
  }

  /* Id、Iq电流环每个电流环周期都运行。 */
  control->ud_output =
      PI_Controller_Run(&control->id_pi, control->id_ref, id_feedback);

  control->uq_output =
      PI_Controller_Run(&control->iq_pi, control->iq_ref_active, iq_feedback);

  if (ud_output != NULL) {
    *ud_output = control->ud_output;
  }

  if (uq_output != NULL) {
    *uq_output = control->uq_output;
  }
}

/**
 * @brief 启用或关闭速度环，实际切换由下一次控制周期完成。
 */
void FOC_Control_EnableSpeedLoop(FOC_Control_t *control, uint8_t enable) {
  if (control == NULL) {
    return;
  }

  control->speed_loop_enable = (enable != 0U) ? 1U : 0U;
}
