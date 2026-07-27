#ifndef __FOC_MATH_H
#define __FOC_MATH_H
#include "stdint.h"
#include "main.h"
/* ======================== CORDIC 常量 ======================== */

#define CORDIC_PI_F               3.14159265358979323846f
#define CORDIC_TWO_PI_F           6.28318530717958647692f
#define CORDIC_INV_PI_F           0.31830988618379067154f

#define CORDIC_Q31_SCALE_F        2147483648.0f
#define CORDIC_Q31_TO_FLOAT_F     4.656612873077392578125e-10f

/* ======================== FOC 数学常量 ======================== */

#define FOC_ONE_THIRD_F           0.33333333333333333333f
#define FOC_INV_SQRT3_F           0.57735026918962576451f
#define FOC_SQRT3_BY_2_F          0.86602540378443864676f

/* ======================== FOC 结构体变量 ======================== */

typedef struct
{
    //时钟相关
    float Ts_us; //采样频率
    uint32_t clock_freq; //时钟频率
    uint32_t arr; //定时器重装载值
    uint32_t ccr4;//adc触发通道值
    uint32_t dead_time;//死区时间
   
    







}FOC_DATA;



extern FOC_DATA foc_data;






/* ======================== FOC 数据初始化 ======================== */

void FOC_Data_Init(void);


void FOC_PWM_Start(void);
void FOC_PWM_Stop(void);












/* ======================== CORDIC 接口 ======================== */

/**
 * @brief 将弧度角转换为 CORDIC 使用的 Q1.31 角度。
 * @note  高频 FOC 推荐直接使用 Q31 电角度，不要每周期调用本函数。
 */
int32_t CORDIC_RadToQ31(float angle_rad);

/**
 * @brief HAL 阻塞版本，主要用于调试验证。
 */
HAL_StatusTypeDef CORDIC_SinCos_F32(float angle_rad,
                                    float *sin_value,
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
void CORDIC_SinCos_Q31_Fast(int32_t angle_q31,
                            int32_t *sin_q31,
                            int32_t *cos_q31);

/**
*__attribute__((always_inline)) static inline  表示内联函数，不跳转执行，速度快
 * @brief Q31 角度输入，float 正余弦输出。
 * @note  适合当前“Q31角度 + 浮点FOC”的结构。
 */
__attribute__((always_inline)) static inline void CORDIC_SinCos_FastF32(
    int32_t angle_q31,
    float *sin_value,
    float *cos_value)
{
    int32_t sin_q31;
    int32_t cos_q31;

    CORDIC_SinCos_Q31_Fast(angle_q31,
                           &sin_q31,
                           &cos_q31);

    *sin_value =
        (float)sin_q31 * CORDIC_Q31_TO_FLOAT_F;

    *cos_value =
        (float)cos_q31 * CORDIC_Q31_TO_FLOAT_F;
}
#endif