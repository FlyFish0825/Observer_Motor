#include "motor_calibration_ls.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define COUNT 61U
static LsSample_t samples[COUNT];

static LsFitConfig_t fixture_config(void)
{
    LsFitConfig_t c = {0};
    c.timestamp_tick_us = 1.0f;
    c.pulse_start_timestamp = 100U;
    c.pulse_end_timestamp = 200U;
    c.freewheel_start_timestamp = 205U;
    c.adc_offset = 2000.0f;
    c.adc_gain_a_per_code = 0.001f;
    c.applied_voltage_v = 1.0f;
    c.loop_resistance_ohm = 1.06f;
    c.current_limit_a = 1.0f;
    c.pulse_hard_limit_us = 110.0f;
    c.blanking_us = 10.0f;
    return c;
}

static void fixture_samples(int noise)
{
    const double tau = 200.0 / 1.06;
    const double peak = (1.0 / 1.06) * (1.0 - exp(-100.0 / tau));
    for (unsigned i = 0; i < COUNT; i++) {
        unsigned t = 100U + i * 5U;
        double current = (t <= 200U) ?
            (1.0 / 1.06) * (1.0 - exp(-(double)(t - 100U) / tau)) :
            peak * exp(-(double)(t - 200U) / tau);
        int code = (int)lround(current * 1000.0);
        if (noise) code += ((int)(i * 17U % 7U) - 3);
        samples[i].adc_raw = (uint16_t)(2000 + code);
        samples[i].timestamp = t;
        samples[i].state = (t <= 200U) ? LS_SAMPLE_RISE : LS_SAMPLE_DECAY;
    }
}

int main(void)
{
    LsFitConfig_t c = fixture_config();
    LsFitResult_t r;
    static LsSample_t short_pulse[256];
    fixture_samples(0);
    assert(MotorCalibration_LsFit(samples, COUNT, &c, &r) == LS_FIT_OK);
    assert(r.rise_valid && r.decay_valid);
    assert(fabsf(r.l_ab_rise_uh - 200.0f) < 15.0f);
    assert(fabsf(r.l_ab_decay_uh - 200.0f) < 3.0f);
    assert(fabsf(r.ls_decay_uh - 100.0f) < 2.0f);
    assert(fabsf(r.sample_period_us - 5.0f) < 0.01f);

    fixture_samples(1); /* ADC 噪声 */
    assert(MotorCalibration_LsFit(samples, COUNT, &c, &r) == LS_FIT_OK);
    assert(fabsf(r.l_ab_decay_uh - 200.0f) < 8.0f);

    fixture_samples(0);
    c.applied_voltage_v = 2.0f; /* Two individually clean fits disagree. */
    assert(MotorCalibration_LsFit(samples, COUNT, &c, &r) == LS_FIT_INCONSISTENT);
    assert(r.ls_rise_uh > r.ls_decay_uh);
    c.applied_voltage_v = 1.0f;

    fixture_samples(0); /* 非零偏置及增益必须参与换算 */
    c.adc_offset = 2000.0f;
    c.adc_gain_a_per_code = 0.001f;
    assert(MotorCalibration_LsFit(samples, COUNT, &c, &r) == LS_FIT_OK);

    assert(MotorCalibration_LsFit(samples, 3U, &c, &r) == LS_FIT_SAMPLE_SHORT);
    c.pulse_end_timestamp = 220U; /* 硬件记录脉冲超限 */
    assert(MotorCalibration_LsFit(samples, COUNT, &c, &r) == LS_FIT_TIMING);
    c.pulse_end_timestamp = 200U;

    fixture_samples(0); /* 脉冲过短，升流有效点不足 */
    c.pulse_end_timestamp = 105U;
    c.freewheel_start_timestamp = 110U;
    for (unsigned i = 2; i < COUNT; i++) samples[i].state = LS_SAMPLE_DECAY;
    assert(MotorCalibration_LsFit(samples, COUNT, &c, &r) == LS_FIT_SAMPLE_SHORT);
    c.pulse_end_timestamp = 200U;
    c.freewheel_start_timestamp = 205U;

    fixture_samples(0);
    for (unsigned i = 10; i < 21; i++) samples[i].adc_raw = 2390U;
    assert(MotorCalibration_LsFit(samples, COUNT, &c, &r) == LS_FIT_POOR_QUALITY);
    assert(!r.rise_valid && r.ls_rise_uh == 0.0f);

    fixture_samples(0);
    c.adc_overrun = 1U;
    assert(MotorCalibration_LsFit(samples, COUNT, &c, &r) == LS_FIT_ADC_OVERRUN);
    c.adc_overrun = 0U;
    samples[10].adc_raw = 4095U;
    assert(MotorCalibration_LsFit(samples, COUNT, &c, &r) == LS_FIT_OVERCURRENT);
    c.current_limit_a = 3.0f;
    assert(MotorCalibration_LsFit(samples, COUNT, &c, &r) == LS_FIT_SATURATED);

    fixture_samples(0);
    samples[25].timestamp = samples[24].timestamp;
    assert(MotorCalibration_LsFit(samples, COUNT, &c, &r) == LS_FIT_TIMING);
    assert(MotorCalibration_LsFit(NULL, COUNT, &c, &r) == LS_FIT_BAD_ARGUMENT);

    /* 接近实际配置：14 us 上升、2 us 采样、约 1 A 峰值、512 us 总窗。 */
    c.timestamp_tick_us = 1.0f;
    c.pulse_start_timestamp = 0U;
    c.pulse_end_timestamp = 14U;
    c.freewheel_start_timestamp = 15U;
    c.adc_offset = 2048.0f;
    c.adc_gain_a_per_code = 0.0056982421875f;
    c.applied_voltage_v = 15.7f;
    c.loop_resistance_ohm = 1.06f;
    c.current_limit_a = 1.5f;
    c.pulse_hard_limit_us = 14.0f;
    c.blanking_us = 2.0f;
    const double short_tau = 200.0 / 1.06;
    const double short_peak = (15.7 / 1.06) * (1.0 - exp(-14.0 / short_tau));
    for (unsigned i = 0U; i < 256U; i++) {
        unsigned t = (i + 1U) * 2U;
        double current = (t < 14U) ?
            (15.7 / 1.06) * (1.0 - exp(-(double)t / short_tau)) :
            short_peak * exp(-((double)t - 14.0) / short_tau);
        short_pulse[i].adc_raw = (uint16_t)lround(2048.0 +
            current / 0.0056982421875);
        short_pulse[i].timestamp = t;
        short_pulse[i].state = (t < 14U) ? LS_SAMPLE_RISE :
            (t >= 15U) ? LS_SAMPLE_DECAY : LS_SAMPLE_INVALID;
    }
    LsFitStatus_t short_status = MotorCalibration_LsFit(short_pulse, 256U, &c, &r);
    assert(short_status == LS_FIT_OK);
    assert(fabsf(r.l_ab_rise_uh - 200.0f) < 25.0f);
    assert(fabsf(r.l_ab_decay_uh - 200.0f) < 8.0f);
    puts("Ls offline fit: PASS");
    return 0;
}
