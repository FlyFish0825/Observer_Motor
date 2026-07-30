#include "controller.h"

#include <stddef.h>

/**
 * @brief 浮点限幅
 */
static inline float PI_Clamp(
    float value,
    float minimum,
    float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }

    if (value < minimum)
    {
        return minimum;
    }

    return value;
}

/**
 * @brief 修正限幅参数顺序
 */
static void PI_NormalizeLimits(
    float *minimum,
    float *maximum)
{
    float temporary;

    if ((*minimum) > (*maximum))
    {
        temporary = *minimum;
        *minimum = *maximum;
        *maximum = temporary;
    }
}

void PI_Controller_Init(
    PI_Controller_t *pi,
    float kp,
    float ki,
    float sample_time,
    float output_min,
    float output_max)
{
    if (pi == NULL)
    {
        return;
    }

    PI_NormalizeLimits(
        &output_min,
        &output_max);

    if (sample_time < 0.0f)
    {
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

void PI_Controller_Reset(
    PI_Controller_t *pi)
{
    if (pi == NULL)
    {
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

float PI_Controller_Run(
    PI_Controller_t *pi,
    float reference,
    float feedback)
{
    if (pi == NULL)
    {
        return 0.0f;
    }

    pi->reference = reference;
    pi->feedback = feedback;

    return PI_Controller_RunError(
        pi,
        reference - feedback);
}

float PI_Controller_RunError(
    PI_Controller_t *pi,
    float error)
{
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

    uint8_t block_integrator = 0U;

    if (pi == NULL)
    {
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

    integral_candidate =
        integral_old +
        ki * error * sample_time;

    integral_candidate = PI_Clamp(
        integral_candidate,
        integral_min,
        integral_max);

    /*
     * 使用新的积分值计算未限幅输出
     */
    output_unsaturated =
        pi->proportional +
        integral_candidate;

    output = PI_Clamp(
        output_unsaturated,
        output_min,
        output_max);

    /*
     * 条件积分抗饱和：
     *
     * 输出已经达到上限并且误差仍要求继续增大输出，
     * 则停止积分。
     *
     * 输出已经达到下限并且误差仍要求继续减小输出，
     * 则停止积分。
     */
    if ((output_unsaturated > output_max) &&
        (error > 0.0f))
    {
        block_integrator = 1U;
    }
    else if ((output_unsaturated < output_min) &&
             (error < 0.0f))
    {
        block_integrator = 1U;
    }

    if (block_integrator != 0U)
    {
        /*
         * 保持原积分值。
         */
        integral_candidate = integral_old;

        output_unsaturated =
            pi->proportional +
            integral_candidate;

        output = PI_Clamp(
            output_unsaturated,
            output_min,
            output_max);
    }

    pi->integral = integral_candidate;

    pi->output_unsaturated =
        output_unsaturated;

    pi->output = output;

    if (output_unsaturated > output_max)
    {
        pi->saturation = PI_SATURATION_HIGH;
    }
    else if (output_unsaturated < output_min)
    {
        pi->saturation = PI_SATURATION_LOW;
    }
    else
    {
        pi->saturation = PI_SATURATION_NONE;
    }

    return output;
}

void PI_Controller_SetGains(
    PI_Controller_t *pi,
    float kp,
    float ki)
{
    if (pi == NULL)
    {
        return;
    }

    pi->kp = kp;
    pi->ki = ki;
}

void PI_Controller_SetSampleTime(
    PI_Controller_t *pi,
    float sample_time)
{
    if (pi == NULL)
    {
        return;
    }

    if (sample_time < 0.0f)
    {
        sample_time = 0.0f;
    }

    pi->sample_time = sample_time;
}

void PI_Controller_SetLimits(
    PI_Controller_t *pi,
    float minimum,
    float maximum)
{
    if (pi == NULL)
    {
        return;
    }

    PI_NormalizeLimits(
        &minimum,
        &maximum);

    pi->output_min = minimum;
    pi->output_max = maximum;

    pi->integral_min = minimum;
    pi->integral_max = maximum;

    pi->integral = PI_Clamp(
        pi->integral,
        minimum,
        maximum);

    pi->output = PI_Clamp(
        pi->output,
        minimum,
        maximum);
}

void PI_Controller_SetOutputLimits(
    PI_Controller_t *pi,
    float minimum,
    float maximum)
{
    if (pi == NULL)
    {
        return;
    }

    PI_NormalizeLimits(
        &minimum,
        &maximum);

    pi->output_min = minimum;
    pi->output_max = maximum;

    pi->output = PI_Clamp(
        pi->output,
        minimum,
        maximum);
}

void PI_Controller_SetIntegralLimits(
    PI_Controller_t *pi,
    float minimum,
    float maximum)
{
    if (pi == NULL)
    {
        return;
    }

    PI_NormalizeLimits(
        &minimum,
        &maximum);

    pi->integral_min = minimum;
    pi->integral_max = maximum;

    pi->integral = PI_Clamp(
        pi->integral,
        minimum,
        maximum);
}

void PI_Controller_PreloadOutput(
    PI_Controller_t *pi,
    float desired_output,
    float reference,
    float feedback)
{
    float output_min;
    float output_max;

    if (pi == NULL)
    {
        return;
    }

    output_min = pi->output_min;
    output_max = pi->output_max;

    desired_output = PI_Clamp(
        desired_output,
        output_min,
        output_max);

    pi->reference = reference;
    pi->feedback = feedback;
    pi->error = reference - feedback;

    pi->proportional =
        pi->kp * pi->error;

    /*
     * desired_output = proportional + integral
     *
     * 所以：
     * integral = desired_output - proportional
     */
    pi->integral =
        desired_output -
        pi->proportional;

    pi->integral = PI_Clamp(
        pi->integral,
        pi->integral_min,
        pi->integral_max);

    pi->output_unsaturated =
        pi->proportional +
        pi->integral;

    pi->output = PI_Clamp(
        pi->output_unsaturated,
        output_min,
        output_max);

    if (pi->output_unsaturated > output_max)
    {
        pi->saturation = PI_SATURATION_HIGH;
    }
    else if (pi->output_unsaturated < output_min)
    {
        pi->saturation = PI_SATURATION_LOW;
    }
    else
    {
        pi->saturation = PI_SATURATION_NONE;
    }
}