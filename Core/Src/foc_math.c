/**
 * @file foc_math.c
 * @brief FOC板级参数、电流换算、SVPWM和CORDIC辅助接口。
 * 电流统一使用A，电压使用V，角度使用电角度rad；坐标变换内联实现见头文件。
 * SVPWM只写计算结果结构体，应用层负责把比较值送入TIM1。
 */
#include "foc_math.h"
#include "adc.h"
#include "bsp_dwt.h"
#include "cordic.h"
#include "main.h"
#include "tim.h"
#include <stdint.h>

/*
 * 本文件保存 FOC 的板级采样参数、电流换算、SVPWM 数学计算及 CORDIC
 * 辅助接口。算法只产生数据和比较值，PWM 外设的具体初始化仍由 CubeMX
 * 文件负责，避免用户代码被重新生成时覆盖。
 */

FOC_Handle_t foc = {0}; /* 全局 FOC 运行状态、采样值和观测器实例。 */
FOC_SIN_COS_t foc_sin_cos = {0}; /* FOC 角度对应的正余弦缓存。 */
FOC_SIN_COS_t observer_sin_cos = {0}; /* 观测器角度对应的正余弦缓存。 */
FOC_Motor_State_t foc_motor_state = FOC_MOTOR_IDLE; /* 电机当前运行状态。 */

/* 95%占空比对应的电流重构阈值，在初始化时计算一次。 */
static uint32_t foc_current_rebuild_threshold = 0U; /* 触发三相电流重构的 PWM 阈值。 */

/**
 * @brief FOC 数据初始化。
 */
/* 初始化 PWM 定时参数、采样增益、校准状态和观测器参数。 */
void FOC_Data_Init(void) {
  foc.timer.pwm_arr = 3399U;
  foc.timer.adc_trigger = 3398U;
  /* 170 MHz下90个DTG计数约为529 ns，适合作为新功率板的保守起点。 */
  foc.timer.dead_time = 90U;
  foc.timer.clock_freq = 170000000U;

  /*
   * 中心对齐PWM周期，单位为秒。
   * 当前参数：(3399 + 1) * 2 / 170 MHz = 40 us。
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
   * 新板电流采样：5 mOhm分流电阻、约24倍模拟增益。
   * ADC注入组使用单次12位采样，因此每个ADC计数对应：
   * 3.3 / 4096 / (0.005 * 24) = 0.0067138671875 A。
   */
  foc.current.gain_a = 0.0067138671875f;
  foc.current.gain_b = 0.0067138671875f;
  foc.current.gain_c = 0.0067138671875f;
  foc.current.rebuild = CURRENT_REBUILD_A;

  foc_sin_cos.sin = 0.0f;
  foc_sin_cos.cos = 1.0f;

  foc.state.omega = 0.0f;

  /* 新水下电机参数：相电阻0.5 ohm、相电感100 uH、磁链2.84 mWb、7极对。 */
  Observer_MotorParam_t motor = { /* 当前电机的电阻、电感、磁链和极对数。 */
      .Rs = 0.5f,
      .Ls = 0.000100f,
      .flux_linkage = 0.00284f,
      .pole_pairs = 7
    };

  Observer_Config_t observer_cfg = { /* 磁链观测器与 SRF-PLL 的运行参数。 */
      /* 磁链观测器保持现有增益和25 kHz更新周期。 */
      .gain = 5e7f,
      .Ts = 0.00004f,
      .psi_min = motor.flux_linkage * 0.5f,
      .psi_max = motor.flux_linkage * 3.0f,

      /*
       * SRF-PLL按二阶系统配置：wn=sqrt(Ki)=200 rad/s，
       * 阻尼比zeta=Kp/(2*wn)=1。相比原Kp=3000，显著降低
       * 磁链角度噪声直接映射到瞬时转速的幅度；提高Ki则保留
       * 对机械加减速的跟踪能力。
       */
      .pll_kp = 400.0f,
      .pll_ki = 40000.0f,
      .pll_omega_limit = 50000.0f
  };

  Observer_Init(&foc.observer, &motor, &observer_cfg);


}

/**
 * @brief 先设置三相相同的50%比较值，再启用主/互补PWM与CH4采样触发。
 * 相同占空比的理想平均线电压为零，但不代表六个功率开关全部关闭。
 */
/* 将三相 PWM 和 ADC 触发通道置于安全中点后启动互补输出。 */
void FOC_PWM_Start(void) {

  uint32_t init_ccr = (foc.timer.pwm_arr + 1U) / 2U; /* 三相初始 50% 比较值。 */
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

/* 停止三相PWM及CH4；注入ADC仍可处于等待外部触发状态。 */
/* 停止三相主输出、互补输出以及 ADC 触发 PWM 通道。 */
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
 * @param handle FOC数据结构体指针
 * @param adc1 ADC1采样值  IA
 * @param adc2 ADC2采样值  IB
 * @param adc3 ADC1采样值  IC
 */
void FOC_Get_Iabc(FOC_Handle_t *handle, uint16_t adc1, uint16_t adc2,
                  uint16_t adc3) {
  uint32_t ccr_a;   /* A 相 PWM 比较值。 */
  uint32_t ccr_b;   /* B 相 PWM 比较值。 */
  uint32_t ccr_c;   /* C 相 PWM 比较值。 */
  uint32_t ccr_max; /* 三相中最大的比较值。 */

  if (handle == NULL) {
    return;
  }

  /* 最大比较值对应最短的低侧导通窗口，优先作为待重构相。 */
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

  /* 当前电流方向约定为(offset-raw)*gain；零偏单位为ADC计数，增益为A/计数。
   * 保留原始计数用于排查，后面重构只覆盖换算后的电流。
   */
  handle->state.i_abc.a =
      (handle->calibration.ia_offset - (float)adc1) * handle->current.gain_a;
  handle->state.i_abc.b =
      (handle->calibration.ib_offset - (float)adc2) * handle->current.gain_b;
  handle->state.i_abc.c =
      (handle->calibration.ic_offset - (float)adc3) * handle->current.gain_c;

  /* 超过95%时用Ia+Ib+Ic=0重构一相；该方法要求另外两相的采样仍然有效。
   * 阈值预先换成整数计数，避免中断每拍重复计算。
   */
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


/**
 * @brief 读取ADC的常规转换数据。
 * @note 保留的DMA读取接口；当前MotorApp使用BoardAdc_Update轮询路径。
 *       两种路径共用ADC规则组，不能同时启动；此函数阻塞等待DMA完成。
 * @return HAL状态。
 */
/* 使用 DMA 阻塞读取常规 ADC 结果，并在超时后停止两个 DMA 通道。 */
HAL_StatusTypeDef ADC_Regular_Read_DMA(void) {
  uint32_t start_tick; /* 启动 DMA 后的系统 tick，用于超时判断。 */

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
  float ua; /* A 相参考电压。 */
  float ub; /* B 相参考电压。 */
  float uc; /* C 相参考电压。 */

  float u_max; /* 三相参考电压最大值。 */
  float u_min; /* 三相参考电压最小值。 */
  float u_span; /* 三相参考电压跨度。 */

  float common_mode; /* 注入三相的公共模式电压。 */
  float voltage_scale; /* 母线不足时的比例缩放因子。 */
  float inv_vbus; /* 母线电压倒数，避免重复除法。 */

  float duty_a; /* A 相限幅后的占空比。 */
  float duty_b; /* B 相限幅后的占空比。 */
  float duty_c; /* C 相限幅后的占空比。 */

  uint32_t arr; /* PWM 自动重装值。 */
  uint32_t middle_ccr; /* 母线无效时使用的中点比较值。 */

  if (output == NULL)
  {
    return HAL_ERROR;
  }

  /*
   * 参数无效时填充默认结果并报错；无有效定时器时CCR置零。
   * 下方母线过低分支则生成三相相同的中点CCR，不等同于功率桥关断。
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

  /* 三相最大电压跨度超过母线时等比例缩小，保留电压矢量方向。 */
  if (u_span > vbus)
  {
    voltage_scale = vbus / u_span;
  }

  /* 同时平移三相，使最大值与最小值关于零对称；不改变任意两相线电压。 */
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
/* 配置 CORDIC 为一次写入、两次读取的余弦/正弦计算模式。 */
void CORDIC_SinCos_RegisterConfig(void) {
  CORDIC->CSR = CORDIC_FUNCTION_COSINE | CORDIC_PRECISION_6CYCLES |
                CORDIC_SCALE_0 | CORDIC_NBWRITE_1 | CORDIC_NBREAD_2 |
                CORDIC_INSIZE_32BITS | CORDIC_OUTSIZE_32BITS;
}

/* 将弧度角归一化到 [-pi, pi) 并转换为 CORDIC 的 Q1.31 格式。 */
int32_t CORDIC_RadToQ31(float angle_rad) {
  while (angle_rad >= CORDIC_PI_F) {
    angle_rad -= CORDIC_TWO_PI_F;
  }

  while (angle_rad < -CORDIC_PI_F) {
    angle_rad += CORDIC_TWO_PI_F;
  }

  float normalized_angle = angle_rad * CORDIC_INV_PI_F; /* 归一化到 [-1, 1)。 */

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
  int32_t input_q31; /* CORDIC 输入角度的 Q1.31 编码。 */
  int32_t output_q31[2]; /* CORDIC 输出的余弦和正弦 Q1.31 值。 */

  HAL_StatusTypeDef status; /* HAL CORDIC 计算结果状态。 */

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

/* 直接访问 CORDIC 寄存器，快速输出 Q1.31 正弦和余弦结果。 */
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
