#include "foc_math.h"
#include "cordic.h"
#include "main.h"
#include "tim.h"




FOC_DATA foc_data ={0};


/**
 * @brief FOC 数据初始化。
 */
void FOC_Data_Init(void)
{
    foc_data.arr = 3359;
    foc_data.ccr4 = 3200;
    foc_data.dead_time = 20;
    foc_data.clock_freq = 168000000;
    foc_data.Ts_us = (foc_data.arr+1.0f)*2.0f*1000000.0f/foc_data.clock_freq;

}


void FOC_PWM_Start(void)
{   

    uint32_t init_ccr = (foc_data.arr+1)/2;
    TIM1->CCR1 = init_ccr;
    TIM1->CCR2 = init_ccr;
    TIM1->CCR3 = init_ccr;
    TIM1->CCR4 = foc_data.ccr4;

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void FOC_PWM_Stop(void)
{

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

}


/**
 * @brief 配置 CORDIC 为 FOC 正余弦模式。
 *
 * 运行期间使用寄存器直接写入和读取，因此这里只配置一次。
 */
void CORDIC_SinCos_RegisterConfig(void)
{
    CORDIC->CSR =
        CORDIC_FUNCTION_COSINE |
        CORDIC_PRECISION_6CYCLES |
        CORDIC_SCALE_0 |
        CORDIC_NBWRITE_1 |
        CORDIC_NBREAD_2 |
        CORDIC_INSIZE_32BITS |
        CORDIC_OUTSIZE_32BITS;
}



int32_t CORDIC_RadToQ31(float angle_rad)
{
    while (angle_rad >= CORDIC_PI_F)
    {
        angle_rad -= CORDIC_TWO_PI_F;
    }

    while (angle_rad < -CORDIC_PI_F)
    {
        angle_rad += CORDIC_TWO_PI_F;
    }

    float normalized_angle =
        angle_rad * CORDIC_INV_PI_F;


         /*
     * Q1.31 无法表示正的 +1.0，
     * 防止浮点舍入生成越界值。
     */
    if (normalized_angle >= 1.0f)
    {
        normalized_angle = 0.99999994f;
    }

    return (int32_t)(
        normalized_angle *
        CORDIC_Q31_SCALE_F);
}


HAL_StatusTypeDef CORDIC_SinCos_F32(float angle_rad,
                                    float *sin_value,
                                    float *cos_value)
{
    int32_t input_q31;
    int32_t output_q31[2];

    HAL_StatusTypeDef status;

    if ((sin_value == NULL) ||
        (cos_value == NULL))
    {
        return HAL_ERROR;
    }

    input_q31 =
        CORDIC_RadToQ31(angle_rad);

    status =
        HAL_CORDIC_Calculate(&hcordic,
                             &input_q31,
                             output_q31,
                             1U,
                             10U);

    if (status != HAL_OK)
    {
        *sin_value = 0.0f;
        *cos_value = 0.0f;

        return status;
    }

    /*
     * COSINE 模式、两结果输出：
     * output[0] = cos
     * output[1] = sin
     */
    *cos_value =
        (float)output_q31[0] *
        CORDIC_Q31_TO_FLOAT_F;

    *sin_value =
        (float)output_q31[1] *
        CORDIC_Q31_TO_FLOAT_F;

    return HAL_OK;
}


void CORDIC_SinCos_Q31_Fast(int32_t angle_q31,
                            int32_t *sin_q31,
                            int32_t *cos_q31)
{
    /*
     * 写入 Q1.31 电角度后 CORDIC 自动开始计算。
     */
    CORDIC->WDATA =
        (uint32_t)angle_q31;

     /*
     * COSINE 模式：
     * 第一次读取 cos，第二次读取 sin。
     */
    *cos_q31 =
        (int32_t)CORDIC->RDATA;

    *sin_q31 =
        (int32_t)CORDIC->RDATA;
}



