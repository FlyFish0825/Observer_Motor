#ifndef __FOC_MATH_H
#define __FOC_MATH_H
#include "main.h"
#include <stdint.h>
#include "observer.h"
#include "arm_math.h"

/* ======================== CORDIC 常量 ======================== */

#define CORDIC_PI_F 3.14159265358979323846f
#define CORDIC_TWO_PI_F 6.28318530717958647692f
#define CORDIC_INV_PI_F 0.31830988618379067154f

#define CORDIC_Q31_SCALE_F 2147483648.0f
#define CORDIC_Q31_TO_FLOAT_F 4.656612873077392578125e-10f

#define Q31_TO_DEG_F 8.381903171539307e-8f
#define RAD_TO_DEG_F 57.29577951308232f
#define Q32_TO_RAD_F 1.4629180792671596e-9f

/* ======================== FOC 数学常量 ======================== */

#define FOC_ONE_THIRD_F   0.33333333333333333333f
#define FOC_INV_SQRT3_F   0.57735026918962576451f
#define FOC_SQRT3_BY_2_F  0.86602540378443864676f
#define FOC_PI            3.14159265358979323846f
#define CURRENT_OFFSET_SAMPLE_NUM 1000 // 校准采样次数

/* ======================== FOC 结构体变量 ======================== */

/*
 * 三相坐标，a/b/c对应U/V/W；电流为A，电压为V
 */
typedef struct {
  float a;
  float b;
  float c;

} FOC_ABC_t;

/*
 * 静止两相坐标
 */
typedef struct {
  float alpha;
  float beta;

} FOC_AlphaBeta_t;

/*
 * 转子同步旋转坐标；d轴沿磁链方向，q轴为转矩电流方向
 */
typedef struct {
  float d;
  float q;

} FOC_DQ_t;

/*
 * sin cos
 */
typedef struct {
  float sin;
  float cos;

} FOC_SIN_COS_t;


/**
 * @brief 三相电流重构相选择
 *
 * 当某一相PWM占空比接近100%时，该相低侧导通窗口过短，
 * 分流电阻采样不可靠，需用另外两相电流重构（Ia+Ib+Ic=0）。
 * 该枚举记录本拍选择重构的是哪一相。
 */
typedef enum {

  CURRENT_REBUILD_A = 0, /**< 重构A相电流（U相） */
  CURRENT_REBUILD_B,     /**< 重构B相电流（V相） */
  CURRENT_REBUILD_C      /**< 重构C相电流（W相） */

} FOC_CurrentRebuild_t;

/**
 * @brief 电机运行状态
 *
 * 控制中断内切换：IDLE时功率桥关闭，CLOSED_LOOP时执行FOC闭环。
 */
typedef enum
{
    FOC_MOTOR_IDLE = 0,      /**< 空闲状态，功率桥关闭，仅保留ADC采样触发 */
    FOC_MOTOR_CLOSED_LOOP,   /**< 闭环运行，执行观测器+双闭环+SVPWM */

} FOC_Motor_State_t;


typedef struct {

  uint16_t adc_a;
  uint16_t adc_b;
  uint16_t adc_c;

  /* 保留的规则组DMA接口缓冲区；当前BoardAdc轮询路径不使用这些数组。 */
  uint16_t adc1_regular_dma_buffer[3] __attribute__((aligned(4)));
  uint16_t adc2_regular_dma_buffer[1] __attribute__((aligned(4)));

  /* 预留零偏字段；当前换算实际使用FOC_Calibration_t的ia/ib/ic_offset。 */
  float offset_a;
  float offset_b;
  float offset_c;

  /* ADC计数到电流的增益，A/计数，需与硬件过采样设置一致。 */
  float gain_a;
  float gain_b;
  float gain_c;

  FOC_CurrentRebuild_t rebuild;

} FOC_CurrentSample_t;

typedef struct {

  // 输入
  float vbus;
  /* MCU内部温度传感器换算值，单位摄氏度。 */
  float temperature_c;
  float theta;
  float omega;

  // 电流
  FOC_ABC_t i_abc;
  FOC_AlphaBeta_t i_alpha_beta;
  FOC_DQ_t i_dq;
  /* 由dq轴电功率估算的母线电流及其滤波值，单位A。 */
  float ibus_est;
  float ibus_filter;

  /// 电压
  FOC_DQ_t u_dq;
  FOC_AlphaBeta_t u_alpha_beta;
  FOC_ABC_t u_abc;

  /* ADC实测端电压，已恢复分压倍率，单位V；a/b/c对应U/V/W对地电压。
   * 上面的u_abc是控制器电压命令，此处独立保存实际采样值。
   */
  FOC_ABC_t u_abc_measured;
  /* 实测端电压经Clarke变换后的静止坐标电压，供低速观测器使用。 */
  FOC_AlphaBeta_t u_alpha_beta_measured;

  // Q31角度
  uint32_t theta_q31;
} FOC_State_t;

typedef struct {

  uint32_t clock_freq; /* TIM1计数时钟，Hz */

  uint32_t pwm_arr; /* 中心对齐计数器顶部ARR */

  uint32_t adc_trigger; /* CH4比较值，决定注入触发相对于载波的位置 */

  uint32_t dead_time; /* TIM死区DTG编码；不是以ns表示的时间 */

  float Ts; /* FOC算法采用的完整控制周期，s */

} FOC_TimerConfig_t;

typedef struct {

  uint8_t calibrated;

  /* 校准阶段先累加ADC计数，完成后除以样本数，保存零电流平均计数。 */
  float ia_offset;
  float ib_offset;
  float ic_offset;

} FOC_Calibration_t;

/*
 * SVPWM计算结果
 */
typedef struct {
  /*
   * 三相占空比，范围0.0~1.0
   */
  float duty_a;
  float duty_b;
  float duty_c;

  /*
   * 三相定时器比较值
   */
  uint32_t ccr_a;
  uint32_t ccr_b;
  uint32_t ccr_c;

  /*
   * 实际加入的公共模电压
   */
  float common_mode;

  /*
   * 电压缩放比例：
   * 1.0表示未限幅；
   * 小于1.0表示输入电压超过母线能力。
   */
  float voltage_scale;

  /*
   * 0：没有限幅
   * 1：发生电压限幅
   */
  uint8_t limited;

} FOC_SVPWM_Output_t;

typedef struct {

  FOC_State_t state;

  FOC_CurrentSample_t current;

  FOC_Calibration_t calibration;

  FOC_TimerConfig_t timer;

  FOC_SVPWM_Output_t svpwm;

   Observer_Handle_t observer;

} FOC_Handle_t;

extern FOC_SIN_COS_t foc_sin_cos;
extern FOC_SIN_COS_t observer_sin_cos;
extern FOC_Handle_t foc;
extern FOC_Motor_State_t foc_motor_state;



/* ======================== FOC 相关函数 ======================== */

/**
 * @brief 初始化FOC全部板级参数、电机模型和观测器配置。
 * @note 必须在 MX_TIM1_Init() 之前调用，因为定时器初始化会读取 foc.timer 参数。
 */
void FOC_Data_Init(void);
/** @brief 按dq轴电功率估算母线电流并进行一阶低通滤波。 */
void FOC_UpdateBusCurrentEstimate(FOC_Handle_t *handle);

/**
 * @brief 启动三相互补PWM和CH4 ADC触发。
 * @note 启动前先将三相CCR置为50%中点，避免跳变产生冲击。
 */
void FOC_PWM_Start(void);

/**
 * @brief 停止三相主/互补PWM和CH4触发通道。
 */
void FOC_PWM_Stop(void);

/**
 * @brief 使用DMA阻塞读取ADC规则组（保留接口，当前未使用）。
 * @return HAL_OK成功，HAL_TIMEOUT超时，HAL_ERROR启动失败。
 */
HAL_StatusTypeDef ADC_Regular_Read_DMA(void);

/**
 * @brief 三相电流ADC零偏校准（已废弃，校准逻辑移入motor_app.c）。
 * @note 保留声明以兼容旧代码；实际校准在MotorApp_OnInjectedConversion中完成。
 */
void FOC_Iabc_Calibration(void);

/**
 * @brief 将三相ADC原始计数换算为电流，并在占空比超限时重构一相。
 * @param handle  FOC全局句柄指针，保存零偏、增益和运行状态。
 * @param adc1    ADC1注入Rank1原始计数（U相电流）。
 * @param adc2    ADC2注入Rank1原始计数（V相电流）。
 * @param adc3    ADC1注入Rank2原始计数（W相电流）。
 */
void FOC_Get_Iabc(FOC_Handle_t *handle, uint16_t adc1, uint16_t adc2,uint16_t adc3);


/* ======================== FOC 计算函数 ======================== */

/**
 * @brief 基于公共模注入的SVPWM计算，将三相电压命令转为占空比和CCR。
 * @param u_abc   三相参考电压（V），来自逆Clarke变换。
 * @param vbus    直流母线电压（V），用于计算占空比和电压限幅。
 * @param timer   定时器参数（使用pwm_arr字段）。
 * @param output  输出结构体，填充三相占空比、CCR和公共模信息。
 * @return HAL_OK正常或发生限幅；HAL_ERROR参数无效或母线过低。
 */
HAL_StatusTypeDef FOC_SVPWM_Run(const FOC_ABC_t *u_abc,float vbus,const FOC_TimerConfig_t *timer,
    FOC_SVPWM_Output_t *output);






/* ======================== CORDIC 接口 ======================== */

/**
 * @brief 将弧度角转换为 CORDIC 使用的 Q1.31 角度。
 * @note  高频 FOC 推荐直接使用 Q31 电角度，不要每周期调用本函数。
 */
int32_t CORDIC_RadToQ31(float angle_rad);

/**
 * @brief HAL 阻塞版本，主要用于调试验证。
 */
HAL_StatusTypeDef CORDIC_SinCos_F32(float angle_rad, float *sin_value,
                                    float *cos_value);

/**
 * @brief 配置 CORDIC 为 COSINE、Q1.31、1写2读、6周期模式。
 * @note  必须在 MX_CORDIC_Init() 之后调用一次。
 */
void CORDIC_SinCos_RegisterConfig(void);

/**
 * @brief 直接读写寄存器的高速 Q31 正余弦函数。
 * @note  当前配置下第一次读出 cos，第二次读出 sin。
 */
void CORDIC_SinCos_Q31_Fast(int32_t angle_q31, int32_t *sin_q31,
                            int32_t *cos_q31);

/**
 *__attribute__((always_inline)) static inline  表示内联函数，不跳转执行，速度快
 * @brief Q31 角度输入，float 正余弦输出。
 * @note  适合当前“Q31角度 + 浮点FOC”的结构。
 */
__STATIC_FORCEINLINE void
CORDIC_SinCos_FastF32(int32_t angle_q31, float *sin_value, float *cos_value) {
  int32_t cos_q31;
  int32_t sin_q31;

  /*
   * 高频路径直接访问CORDIC寄存器，避免再调用
   * CORDIC_SinCos_Q31_Fast()产生函数调用和指针传参开销。
   */
  CORDIC->WDATA = (uint32_t)angle_q31;

  cos_q31 = (int32_t)CORDIC->RDATA;
  sin_q31 = (int32_t)CORDIC->RDATA;

  *sin_value = (float)sin_q31 * CORDIC_Q31_TO_FLOAT_F;
  *cos_value = (float)cos_q31 * CORDIC_Q31_TO_FLOAT_F;
}

/**
 * @brief 已经位于[-pi, pi]范围内的弧度角快速转换为Q1.31。
 * @note 不执行循环归一化，供观测器和闭环ISR的高频路径使用。
 */
__STATIC_FORCEINLINE int32_t
CORDIC_RadToQ31_WrappedFast(float angle_rad)
{
    float normalized_angle = angle_rad * CORDIC_INV_PI_F;

    /* Q1.31不能表示+1.0，保护恰好为+pi的情况。 */
    if (normalized_angle >= 1.0f)
    {
        normalized_angle = 0.99999994f;
    }
    else if (normalized_angle < -1.0f)
    {
        normalized_angle = -1.0f;
    }

    return (int32_t)(normalized_angle * CORDIC_Q31_SCALE_F);
}




/**
 * @brief 快速atan2近似
 * 输入:
 *      y
 *      x
 * 输出:
 *      atan2(y,x)
 * 范围:
 *      -pi ~ pi
 * 误差:
 *      约0.3度
 * 用于:
 *      磁链观测器角度计算
 */
__STATIC_FORCEINLINE float FOC_atan2_Fast(
    float y,
    float x)
{
    const float abs_y =
        fabsf(y) + 1.0e-20f;

    float angle;


    if (x >= 0.0f)
    {
        float r =
            (x - abs_y) /
            (x + abs_y);


        float r_sq =
            r * r;


        angle =
            ((0.1963f * r_sq)
             -0.9817f)
             * r
             +
             (FOC_PI * 0.25f);
    }
    else
    {
        float r =
            (x + abs_y) /
            (abs_y - x);


        float r_sq =
            r * r;


        angle =
            ((0.1963f * r_sq)
             -0.9817f)
             * r
             +
             (FOC_PI * 0.75f);
    }


    return (y < 0.0f) ?
            -angle :
             angle;
}



/**
 * @brief 三相静止坐标 abc -> 两相静止坐标 alpha-beta。
 *
 * 使用三电流完整 Clarke 变换：
 *   alpha = (2Ia - Ib - Ic) / 3
 *   beta  = (Ib - Ic) / sqrt(3)
 *
 * 三路电流存在少量采样误差时，该形式比直接令 alpha=Ia 更稳妥。
 */
__STATIC_FORCEINLINE void FOC_Clarke(const FOC_ABC_t *abc, FOC_AlphaBeta_t *ab) {

  ab->alpha = (2.0f * abc->a - abc->b - abc->c) * (1.0f / 3.0f);

  ab->beta = (abc->b - abc->c) * (0.577350269f);
}

/**
 * @brief 两相静止坐标 alpha-beta -> 旋转坐标 d-q。
 *
 *   d = alpha*cos(theta) + beta*sin(theta)
 *   q = -alpha*sin(theta) + beta*cos(theta)
 */
__STATIC_FORCEINLINE void FOC_Park(const FOC_AlphaBeta_t *ab, const FOC_SIN_COS_t *sc,
                            FOC_DQ_t *dq) {

  dq->d = ab->alpha * sc->cos + ab->beta * sc->sin;

  dq->q = -ab->alpha * sc->sin + ab->beta * sc->cos;
}

/**
 * @brief 旋转坐标 d-q -> 两相静止坐标 alpha-beta。
 *
 *   alpha = d*cos(theta) - q*sin(theta)
 *   beta  = d*sin(theta) + q*cos(theta)
 */
__STATIC_FORCEINLINE void FOC_InvPark(const FOC_DQ_t *dq, const FOC_SIN_COS_t *sc,
                               FOC_AlphaBeta_t *ab) {

  ab->alpha = dq->d * sc->cos - dq->q * sc->sin;

  ab->beta = dq->d * sc->sin + dq->q * sc->cos;
}

/**
 * @brief 两相静止坐标 alpha-beta -> 三相静止坐标 abc。
 *
 * 当前SVPWM接口使用此三相电压输出，通过公共模注入计算占空比。
 */
__STATIC_FORCEINLINE void FOC_InvClarke(const FOC_AlphaBeta_t *input,
                                        FOC_ABC_t *output) {
  output->a = input->alpha;

  output->b = -0.5f * input->alpha + FOC_SQRT3_BY_2_F * input->beta;

  output->c = -0.5f * input->alpha - FOC_SQRT3_BY_2_F * input->beta;
}

/**
 * @brief 将占空比限制在0~1。
 */
__STATIC_FORCEINLINE float FOC_ClampDuty(float duty) {
  if (duty > 1.0f) {
    return 1.0f;
  }

  if (duty < 0.0f) {
    return 0.0f;
  }

  return duty;
}

/**
 * @brief 将占空比转换成CCR计数值。
 *
 * @param duty 占空比，范围0~1
 * @param arr   定时器ARR
 */
__STATIC_FORCEINLINE uint32_t FOC_DutyToCCR(float duty, uint32_t arr) {
  uint32_t ccr;
  float ccr_float;

  duty = FOC_ClampDuty(duty);

  /*
   * 本函数按ARR+1将归一化占空比换成比较值；当前ARR=3399。
   *
   * duty=0.5：
   * CCR=0.5*3400=1700
   */
  ccr_float = duty * (float)(arr + 1U);

  /*
   * 加0.5实现四舍五入。
   */
  ccr = (uint32_t)(ccr_float + 0.5f);

  /*
   * 当前代码不输出ARR+1，最大限制为ARR。
   */
  if (ccr > arr) {
    ccr = arr;
  }

  return ccr;
}



/* 通用归一化：对有限弧度输入反复加减2pi，范围收敛到[-pi,pi]。 */
__STATIC_FORCEINLINE float FOC_WrapToPi(float angle)
{
    while(angle > FOC_PI)
    {
        angle -= 2.0f * FOC_PI;
    }

    while(angle < -FOC_PI)
    {
        angle += 2.0f * FOC_PI;
    }

    return angle;
}

/**
 * @brief 单步角度归一化。
 * @note 仅用于已知输入最多越界一个2pi周期的高频路径。
 */
__STATIC_FORCEINLINE float FOC_WrapToPiFast(float angle)
{
    if (angle > FOC_PI)
    {
        angle -= CORDIC_TWO_PI_F;
    }
    else if (angle < -FOC_PI)
    {
        angle += CORDIC_TWO_PI_F;
    }

    return angle;
}

#endif /* __FOC_MATH_H */
