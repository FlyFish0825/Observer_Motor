#include "motor_calibration_ls.h"

#include <math.h>
#include <string.h>

/* 独立离线拟合单元；没有 TIM/GPIO/ADC 写操作，不可能触发功率级。 */
#define LS_MIN_SAMPLES 5U
#define LS_MIN_SPAN_US 8.0f
#define LS_MIN_CURRENT_A 0.02f
#define LS_MAX_RMSE_A 0.02f
#define LS_MAX_RELATIVE_L_DIFF 0.40f

typedef struct {
    double n, sx, sy, sxx, sxy;
    double slope, intercept;
    uint32_t count;
    float start_us, end_us;
} LsRegression_t;

static void LsRegression_Add(LsRegression_t *reg, double x, double y)
{
    if (reg->count == 0U) reg->start_us = (float)x;
    reg->end_us = (float)x;
    reg->count++;
    reg->n += 1.0;
    reg->sx += x;
    reg->sy += y;
    reg->sxx += x * x;
    reg->sxy += x * y;
}

static uint8_t LsRegression_Solve(LsRegression_t *reg)
{
    double denominator = reg->n * reg->sxx - reg->sx * reg->sx;
    if ((reg->count < LS_MIN_SAMPLES) ||
        ((reg->end_us - reg->start_us) < LS_MIN_SPAN_US) ||
        (!isfinite(denominator)) || (denominator <= 1.0e-9)) return 0U;
    reg->slope = (reg->n * reg->sxy - reg->sx * reg->sy) / denominator;
    reg->intercept = (reg->sy - reg->slope * reg->sx) / reg->n;
    return (isfinite(reg->slope) && isfinite(reg->intercept)) ? 1U : 0U;
}

static float LsCurrent(const LsSample_t *sample, const LsFitConfig_t *config)
{
    return ((float)sample->adc_raw - config->adc_offset) *
           config->adc_gain_a_per_code;
}

LsFitStatus_t MotorCalibration_LsFit(const LsSample_t *samples, size_t count,
                                    const LsFitConfig_t *config,
                                    LsFitResult_t *result)
{
    LsRegression_t rise = {0}, decay = {0};
    uint32_t previous = 0U;
    uint8_t saw_rise = 0U, saw_decay = 0U;
    double rise_error = 0.0, decay_error = 0.0;

    if (result == NULL) return LS_FIT_BAD_ARGUMENT;
    memset(result, 0, sizeof(*result));
    result->status = LS_FIT_BAD_ARGUMENT;
    if ((samples == NULL) || (config == NULL) || (count == 0U) ||
        !isfinite(config->timestamp_tick_us) || (config->timestamp_tick_us <= 0.0f) ||
        !isfinite(config->adc_offset) ||
        !isfinite(config->adc_gain_a_per_code) || (config->adc_gain_a_per_code <= 0.0f) ||
        !isfinite(config->applied_voltage_v) || (config->applied_voltage_v <= 0.0f) ||
        !isfinite(config->loop_resistance_ohm) || (config->loop_resistance_ohm <= 0.0f) ||
        !isfinite(config->current_limit_a) || (config->current_limit_a <= 0.0f) ||
        !isfinite(config->pulse_hard_limit_us) || (config->pulse_hard_limit_us <= 0.0f) ||
        !isfinite(config->blanking_us) || (config->blanking_us < 0.0f)) return result->status;
    if (config->adc_overrun) return result->status = LS_FIT_ADC_OVERRUN;
    if ((config->pulse_start_timestamp >= config->pulse_end_timestamp) ||
        (config->pulse_end_timestamp > config->freewheel_start_timestamp) ||
        ((double)(config->pulse_end_timestamp - config->pulse_start_timestamp) *
         config->timestamp_tick_us > config->pulse_hard_limit_us))
        return result->status = LS_FIT_TIMING;

    for (size_t i = 0U; i < count; i++) {
        float current = LsCurrent(&samples[i], config);
        if ((i != 0U) && (samples[i].timestamp <= previous))
            return result->status = LS_FIT_TIMING;
        previous = samples[i].timestamp;
        if (samples[i].state > LS_SAMPLE_INVALID)
            return result->status = LS_FIT_BAD_ARGUMENT;
        if (!isfinite(current)) return result->status = LS_FIT_NUMERIC;
        if (fabsf(current) > config->current_limit_a)
            return result->status = LS_FIT_OVERCURRENT;
        if (samples[i].adc_raw >= 4095U)
            return result->status = LS_FIT_SATURATED;
        if (samples[i].state == LS_SAMPLE_RISE) {
            if (saw_decay) return result->status = LS_FIT_TIMING;
            if ((samples[i].timestamp < config->pulse_start_timestamp) ||
                (samples[i].timestamp > config->pulse_end_timestamp))
                return result->status = LS_FIT_TIMING;
            saw_rise = 1U;
        } else if (samples[i].state == LS_SAMPLE_DECAY) {
            if (!saw_rise) return result->status = LS_FIT_TIMING;
            if (samples[i].timestamp < config->freewheel_start_timestamp)
                return result->status = LS_FIT_TIMING;
            saw_decay = 1U;
        }
    }
    if (!saw_rise || !saw_decay) return result->status = LS_FIT_SAMPLE_SHORT;
    if (count > 1U)
        result->sample_period_us = (float)((double)(samples[count - 1U].timestamp -
            samples[0].timestamp) * config->timestamp_tick_us / (count - 1U));

    for (size_t i = 0U; i < count; i++) {
        float current = LsCurrent(&samples[i], config);
        float t_us = (float)(((double)samples[i].timestamp -
                             config->pulse_start_timestamp) *
                             config->timestamp_tick_us);
        if ((samples[i].state == LS_SAMPLE_RISE) &&
            (t_us >= config->blanking_us) && (current > LS_MIN_CURRENT_A))
            LsRegression_Add(&rise, t_us, current);
        if ((samples[i].state == LS_SAMPLE_DECAY) &&
            (current > LS_MIN_CURRENT_A))
            LsRegression_Add(&decay, t_us, log((double)current));
    }
    result->rise_samples = rise.count;
    result->decay_samples = decay.count;
    result->rise_start_us = rise.start_us;
    result->rise_end_us = rise.end_us;
    result->decay_start_us = decay.start_us;
    result->decay_end_us = decay.end_us;
    if (!LsRegression_Solve(&rise) || !LsRegression_Solve(&decay))
        return result->status = LS_FIT_SAMPLE_SHORT;
    if ((rise.slope <= 0.0) || (decay.slope >= 0.0))
        return result->status = LS_FIT_NUMERIC;

    /* 上升斜率单位 A/us；下降段对 ln(I) 的斜率单位 1/us。 */
    double mean_rise_a = rise.sy / rise.n;
    double l_rise_uh = (config->applied_voltage_v -
                        config->loop_resistance_ohm * mean_rise_a) / rise.slope;
    double tau_decay_us = -1.0 / decay.slope;
    double l_decay_uh = tau_decay_us * config->loop_resistance_ohm;
    if (!isfinite(l_rise_uh) || !isfinite(l_decay_uh) ||
        (l_rise_uh <= 0.0) || (l_decay_uh <= 0.0))
        return result->status = LS_FIT_NUMERIC;

    for (size_t i = 0U; i < count; i++) {
        float current = LsCurrent(&samples[i], config);
        float t_us = (float)(((double)samples[i].timestamp -
                             config->pulse_start_timestamp) *
                             config->timestamp_tick_us);
        if ((samples[i].state == LS_SAMPLE_RISE) &&
            (t_us >= config->blanking_us) && (current > LS_MIN_CURRENT_A)) {
            double error = current - (rise.intercept + rise.slope * t_us);
            rise_error += error * error;
        }
        if ((samples[i].state == LS_SAMPLE_DECAY) &&
            (current > LS_MIN_CURRENT_A)) {
            double error = current - exp(decay.intercept + decay.slope * t_us);
            decay_error += error * error;
        }
    }
    result->tau_rise_us = (float)(l_rise_uh / config->loop_resistance_ohm);
    result->tau_decay_us = (float)tau_decay_us;
    result->l_ab_rise_uh = (float)l_rise_uh;
    result->l_ab_decay_uh = (float)l_decay_uh;
    result->ls_rise_uh = (float)(l_rise_uh / 2.0);
    result->ls_decay_uh = (float)(l_decay_uh / 2.0);
    result->rise_rmse_a = (float)sqrt(rise_error / rise.n);
    result->decay_rmse_a = (float)sqrt(decay_error / decay.n);
    if (!isfinite(result->rise_rmse_a) || !isfinite(result->decay_rmse_a) ||
        (result->rise_rmse_a > LS_MAX_RMSE_A) ||
        (result->decay_rmse_a > LS_MAX_RMSE_A)) {
        result->l_ab_rise_uh = result->l_ab_decay_uh = 0.0f;
        result->ls_rise_uh = result->ls_decay_uh = 0.0f;
        return result->status = LS_FIT_POOR_QUALITY;
    }
    result->rise_valid = 1U;
    result->decay_valid = 1U;
    /* 两段各自残差较小，也可能对应互相矛盾的等效电路。 */
    if (fabs(l_rise_uh - l_decay_uh) / (0.5 * (l_rise_uh + l_decay_uh)) >
        LS_MAX_RELATIVE_L_DIFF)
        return result->status = LS_FIT_INCONSISTENT;
    return result->status = LS_FIT_OK;
}
