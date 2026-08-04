#include "foc_math.h"
#include "adc.h"
#include "bsp_dwt.h"
#include "cordic.h"
#include "main.h"
#include "tim.h"
#include <stdint.h>

FOC_Handle_t foc = {0};
FOC_SIN_COS_t foc_sin_cos = {0};
FOC_SIN_COS_t observer_sin_cos = {0};
FOC_Motor_State_t foc_motor_state = FOC_MOTOR_IDLE;

/* 95%占空比对应的电流重构阈值，在初始化时计算一次。 */
static uint32_t foc_current_rebuild_threshold = 0U;

/**
 * @brief FOC 数据初始化。
 */
void FOC_Data_Init(void) {
  foc.timer.pwm_arr = 3359U;
  foc.timer.adc_trigger = 3358U;
  foc.timer.dead_time = 20U;
  foc.timer.clock_freq = 168000000U;

  /*
   * 中心对齐PWM周期，单位为秒。
   * 当前参数：(3359 + 1) * 2 / 168 MHz = 40 us。
   */
  foc.timer.Ts =
      (foc.timer.pwm_arr + 1.0f) * 2.0f / (float)foc.timer.clock_freq;

  foc_current_rebuild_threshold =
      (foc.timer.pwm_arr * 95U) / 100U;

  foc.calibration.calibrated = 0U;
  foc.calibration.ia_offset = 0.0f;
  foc.calibration.ib_offset = 0.0f;
  foc.calibration.ic_offset = 0.0f;

  /*
   * 4倍硬件过采样后右移量未在ADC硬件中完成，
   * 因此保留原来的0.25比例。
   */
  foc.current.gain_a = 0.0056982421875f;
  foc.current.gain_b = 0.0056982421875f;
  foc.current.gain_c = 0.0056982421875f;
  foc.current.rebuild = CURRENT_REBUILD_A;

  foc_sin_cos.sin = 0.0f;
  foc_sin_cos.cos = 1.0f;

  foc.state.omega = 0.0f;

  Observer_MotorParam_t motor = {
      .Rs = 0.2f, 
      .Ls = 0.000035f, 
      .flux_linkage = 0.0016f, 
      .pole_pairs = 7
    };

  Observer_Config_t observer_cfg = {/*
                                     * VESC经验值
                                     */
                                    .gain =1e9f,
                                    .Ts = 0.00004f,  //25kHz
                                    .psi_min = motor.flux_linkage*0.5f,
                                    .psi_max = motor.flux_linkage*3.0f,
                                    .pll_kp = 3000.0f,
                                    .pll_ki = 20000.0f,
                                    .pll_omega_limit = 5000.0f
                                  };

  Observer_Init(&foc.observer, &motor, &observer_cfg);


}

void FOC_PWM_Start(void) {

  uint32_t init_ccr = (foc.timer.pwm_arr + 1U) / 2U;
  TIM1->CCR1 = init_ccr;
  TIM1->CCR2 = init_ccr;
  TIM1->CCR3 = init_ccr;
  TIM1->CCR4 = foc.timer.adc_trigger;

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void FOC_PWM_Stop(void) {

  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
}

/**
 * @brief FOC电流采样
 * @param obs  FOC数据结构体指针
 * @param adc1 ADC1采样值  IA
 * @param adc2 ADC2采样值  IB
 * @param adc3 ADC1采样值  IC
 */
void FOC_Get_Iabc(FOC_Handle_t *handle, uint16_t adc1, uint16_t adc2,
                  uint16_t adc3) {
  uint32_t ccr_a;
  uint32_t ccr_b;
  uint32_t ccr_c;
  uint32_t ccr_max;

  if (handle == NULL) {
    return;
  }

  ccr_a = TIM1->CCR1;
  ccr_b = TIM1->CCR2;
  ccr_c = TIM1->CCR3;
  ccr_max = ccr_a;

  handle->current.rebuild = CURRENT_REBUILD_A;

  if (ccr_b > ccr_max) {
    ccr_max = ccr_b;
    handle->current.rebuild = CURRENT_REBUILD_B;
  }

  if (ccr_c > ccr_max) {
    ccr_max = ccr_c;
    handle->current.rebuild = CURRENT_REBUILD_C;
  }

  handle->current.adc_a = adc1;
  handle->current.adc_b = adc2;
  handle->current.adc_c = adc3;

  handle->state.i_abc.a =
      (handle->calibration.ia_offset - (float)adc1) * handle->current.gain_a;
  handle->state.i_abc.b =
      (handle->calibration.ib_offset - (float)adc2) * handle->current.gain_b;
  handle->state.i_abc.c =
      (handle->calibration.ic_offset - (float)adc3) * handle->current.gain_c;

  /* 使用初始化时算好的整数阈值，避免ISR中每拍做整型转浮点和乘法。 */
  if (ccr_max > foc_current_rebuild_threshold) {
    switch (handle->current.rebuild) {
    case CURRENT_REBUILD_A:
      handle->state.i_abc.a = -handle->state.i_abc.b - handle->state.i_abc.c;
      break;

    case CURRENT_REBUILD_B:
      handle->state.i_abc.b = -handle->state.i_abc.a - handle->state.i_abc.c;
      break;

    case CURRENT_REBUILD_C:
      handle->state.i_abc.c = -handle->state.i_abc.a - handle->state.i_abc.b;
      break;

    default:
      break;
    }
  }
}


void FOC_Open_Loop(float u_d, float u_q, uint32_t theta_step_q32) {
  foc.state.u_dq.d = u_d;
  foc.state.u_dq.q = u_q;

  foc.state.theta_q31 += theta_step_q32;

  CORDIC_SinCos_FastF32(foc.state.theta_q31, &foc_sin_cos.sin,
                        &foc_sin_cos.cos);

  FOC_InvPark(&foc.state.u_dq, &foc_sin_cos, &foc.state.u_alpha_beta);

  FOC_InvClarke(&foc.state.u_alpha_beta, &foc.state.u_abc);

  FOC_SVPWM_Run(&foc.state.u_abc, foc.state.vbus, &foc.timer, &foc.svpwm);
}


/**
 * @brief 读取ADC的常规转换数据。
 * @return HAL状态。
 */
HAL_StatusTypeDef ADC_Regular_Read_DMA(void) {
  uint32_t start_tick;

  /*
   * 先启动ADC2，再启动ADC1。
   */
  if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)foc.current.adc2_regular_dma_buffer,
                        1U) != HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)foc.current.adc1_regular_dma_buffer,
                        3U) != HAL_OK) {
    HAL_ADC_Stop_DMA(&hadc2);
    return HAL_ERROR;
  }

  /*
   * 等待两个DMA都完成。
   *
   * 不使用HAL_DMA_PollForTransfer，
   * 避免DMA中断先完成后HAL状态发生竞争。
   */
  start_tick = HAL_GetTick();

  while ((__HAL_DMA_GET_COUNTER(&hdma_adc1) != 0U) ||
         (__HAL_DMA_GET_COUNTER(&hdma_adc2) != 0U)) {
    if ((HAL_GetTick() - start_tick) > 10U) {
      HAL_ADC_Stop_DMA(&hadc1);
      HAL_ADC_Stop_DMA(&hadc2);

      return HAL_TIMEOUT;
    }
  }

  HAL_ADC_Stop_DMA(&hadc1);
  HAL_ADC_Stop_DMA(&hadc2);

  return HAL_OK;
}


/**
 * @brief 基于最大值/最小值公共模注入的SVPWM。
 *
 * 输入：
 *   u_abc：三相参考相电压，单位V
 *   vbus ：直流母线电压，单位V
 *   timer：定时器参数，主要使用pwm_arr
 *
 * 输出：
 *   output：三相占空比和CCR计数值
 *
 * @note
 *  该函数只负责计算，不直接修改TIM寄存器。
 *
 * @note
 *  输入Uabc和Vbus必须使用相同的电压单位。
 */
HAL_StatusTypeDef FOC_SVPWM_Run(const FOC_ABC_t *u_abc, float vbus,
                                const FOC_TimerConfig_t *timer,
                                FOC_SVPWM_Output_t *output)
{
  float ua;
  float ub;
  float uc;

  float u_max;
  float u_min;
  float u_span;

  float common_mode;
  float voltage_scale;
  float inv_vbus;

  float duty_a;
  float duty_b;
  float duty_c;

  uint32_t arr;
  uint32_t middle_ccr;

  if (output == NULL)
  {
    return HAL_ERROR;
  }

  /*
   * 错误路径才生成安全的50%占空比。
   * 正常FOC路径不再每拍先写一遍默认值、随后又覆盖。
   */
  if ((u_abc == NULL) || (timer == NULL) || (timer->pwm_arr == 0U))
  {
    output->duty_a = 0.5f;
    output->duty_b = 0.5f;
    output->duty_c = 0.5f;
    output->ccr_a = 0U;
    output->ccr_b = 0U;
    output->ccr_c = 0U;
    output->common_mode = 0.0f;
    output->voltage_scale = 1.0f;
    output->limited = 0U;
    return HAL_ERROR;
  }

  arr = timer->pwm_arr;

  if (vbus <= 0.001f)
  {
    middle_ccr = (arr + 1U) >> 1U;
    if (middle_ccr > arr)
    {
      middle_ccr = arr;
    }

    output->duty_a = 0.5f;
    output->duty_b = 0.5f;
    output->duty_c = 0.5f;
    output->ccr_a = middle_ccr;
    output->ccr_b = middle_ccr;
    output->ccr_c = middle_ccr;
    output->common_mode = 0.0f;
    output->voltage_scale = 1.0f;
    output->limited = 0U;
    return HAL_ERROR;
  }

  /* 三相输入只读一次，减少结构体重复访存。 */
  ua = u_abc->a;
  ub = u_abc->b;
  uc = u_abc->c;

  u_max = ua;
  if (ub > u_max)
  {
    u_max = ub;
  }
  if (uc > u_max)
  {
    u_max = uc;
  }

  u_min = ua;
  if (ub < u_min)
  {
    u_min = ub;
  }
  if (uc < u_min)
  {
    u_min = uc;
  }

  u_span = u_max - u_min;
  voltage_scale = 1.0f;

  if (u_span > vbus)
  {
    voltage_scale = vbus / u_span;
  }

  common_mode = -0.5f * (u_max + u_min);

  /*
   * 每拍只计算一次1/Vbus，三相占空比全部改用乘法。
   * 原代码在正常路径中进行了3次相同的浮点除法。
   */
  inv_vbus = 1.0f / vbus;

  duty_a = 0.5f + (ua + common_mode) * voltage_scale * inv_vbus;
  duty_b = 0.5f + (ub + common_mode) * voltage_scale * inv_vbus;
  duty_c = 0.5f + (uc + common_mode) * voltage_scale * inv_vbus;

  duty_a = FOC_ClampDuty(duty_a);
  duty_b = FOC_ClampDuty(duty_b);
  duty_c = FOC_ClampDuty(duty_c);

  output->duty_a = duty_a;
  output->duty_b = duty_b;
  output->duty_c = duty_c;

  output->ccr_a = FOC_DutyToCCR(duty_a, arr);
  output->ccr_b = FOC_DutyToCCR(duty_b, arr);
  output->ccr_c = FOC_DutyToCCR(duty_c, arr);

  output->common_mode = common_mode * voltage_scale;
  output->voltage_scale = voltage_scale;
  output->limited = (voltage_scale < 1.0f) ? 1U : 0U;

  return HAL_OK;
}

/**
 * @brief 配置 CORDIC 为 FOC 正余弦模式。
 *
 * 运行期间使用寄存器直接写入和读取，因此这里只配置一次。
 */
void CORDIC_SinCos_RegisterConfig(void) {
  CORDIC->CSR = CORDIC_FUNCTION_COSINE | CORDIC_PRECISION_6CYCLES |
                CORDIC_SCALE_0 | CORDIC_NBWRITE_1 | CORDIC_NBREAD_2 |
                CORDIC_INSIZE_32BITS | CORDIC_OUTSIZE_32BITS;
}

int32_t CORDIC_RadToQ31(float angle_rad) {
  while (angle_rad >= CORDIC_PI_F) {
    angle_rad -= CORDIC_TWO_PI_F;
  }

  while (angle_rad < -CORDIC_PI_F) {
    angle_rad += CORDIC_TWO_PI_F;
  }

  float normalized_angle = angle_rad * CORDIC_INV_PI_F;

  /*
   * Q1.31 无法表示正的 +1.0，
   * 防止浮点舍入生成越界值。
   */
  if (normalized_angle >= 1.0f) {
    normalized_angle = 0.99999994f;
  }

  return (int32_t)(normalized_angle * CORDIC_Q31_SCALE_F);
}

HAL_StatusTypeDef CORDIC_SinCos_F32(float angle_rad, float *sin_value,
                                    float *cos_value) {
  int32_t input_q31;
  int32_t output_q31[2];

  HAL_StatusTypeDef status;

  if ((sin_value == NULL) || (cos_value == NULL)) {
    return HAL_ERROR;
  }

  input_q31 = CORDIC_RadToQ31(angle_rad);

  status = HAL_CORDIC_Calculate(&hcordic, &input_q31, output_q31, 1U, 10U);

  if (status != HAL_OK) {
    *sin_value = 0.0f;
    *cos_value = 0.0f;

    return status;
  }

  /*
   * COSINE 模式、两结果输出：
   * output[0] = cos
   * output[1] = sin
   */
  *cos_value = (float)output_q31[0] * CORDIC_Q31_TO_FLOAT_F;

  *sin_value = (float)output_q31[1] * CORDIC_Q31_TO_FLOAT_F;

  return HAL_OK;
}

void CORDIC_SinCos_Q31_Fast(int32_t angle_q31, int32_t *sin_q31,
                            int32_t *cos_q31) {
  /*
   * 写入 Q1.31 电角度后 CORDIC 自动开始计算。
   */
  CORDIC->WDATA = (uint32_t)angle_q31;

  /*
   * COSINE 模式：
   * 第一次读取 cos，第二次读取 sin。
   */
  *cos_q31 = (int32_t)CORDIC->RDATA;

  *sin_q31 = (int32_t)CORDIC->RDATA;
}
