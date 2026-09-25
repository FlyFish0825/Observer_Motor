#include "motor_calibration.h"

#include "debug_console.h"
#include "tim.h"
#include <math.h>
#include <stdint.h>

#define MOTOR_CALIBRATION_STAGE_COUNT             4U
#define MOTOR_CALIBRATION_SETTLE_CYCLES          5000U
#define MOTOR_CALIBRATION_SAMPLE_CYCLES          1000U
#define MOTOR_CALIBRATION_WINDOW_CYCLES           250U
#define RS_IDENTIFY_MAX_CURRENT_A                 1.80f
#define RS_IDENTIFY_VOLTAGE_VBUS_MAX_V            30.0f
#define RS_IDENTIFY_ADC_SUM_MAX                   16380.0f
#define RS_IDENTIFY_ADC_STALL_TIMEOUT_MS          20U
#define RS_IDENTIFY_STAGE_TIMEOUT_MS             500U
#define MOTOR_CALIBRATION_MIN_CURRENT_A           0.02f
#define MOTOR_CALIBRATION_MIN_CURRENT_SPREAD_A    0.02f
#define MOTOR_CALIBRATION_TARGET_TOLERANCE        0.05f
#define MOTOR_CALIBRATION_STABLE_DELTA_A          0.002f
#define MOTOR_CALIBRATION_VOLTAGE_STABLE_DELTA_A  0.010f
#define MOTOR_CALIBRATION_MAX_DUTY                0.20f
#define MOTOR_CALIBRATION_CURRENT_KP              0.003f
#define MOTOR_CALIBRATION_VOLTAGE_DUTY_STEP       0.0001f
#define MOTOR_CALIBRATION_DUTY_SETTLE_TOLERANCE  0.001f
#define MOTOR_CALIBRATION_DUTY_SETTLE_CYCLES      250U
#define MOTOR_CALIBRATION_FIT_DENOMINATOR_MIN     1.0e-6f

#define MOTOR_CALIBRATION_PHASE_CCER_MASK \
    (TIM_CCER_CC1E | TIM_CCER_CC1NE | \
     TIM_CCER_CC2E | TIM_CCER_CC2NE | \
     TIM_CCER_CC3E | TIM_CCER_CC3NE)

static const float motor_calibration_stage_target_current[
    MOTOR_CALIBRATION_STAGE_COUNT] = {
    0.50f, 0.80f, 1.15f, 1.50f
};

static const float rs_voltage_targets[MOTOR_CALIBRATION_STAGE_COUNT] = {
    0.65f, 0.95f, 1.25f, 1.55f
};

typedef struct
{
    float sum_ib;
    float sum_abs_ib;
    uint64_t sum_adc_b;
    uint64_t sum_ccr1;
    float sum_vbus;
    float sum_u_cmd;
    float sum_start_abs_ib;
    float sum_end_abs_ib;
    float sum_start_duty;
    float sum_end_duty;
    float min_duty;
    float max_duty;
    float min_ib;
    float max_ib;
    float max_abs_ib_seen;
    uint32_t sample_count;
    uint32_t start_window_count;
    uint32_t end_window_count;
    uint32_t settle_start_tick_ms;
    uint32_t settle_elapsed_ms;
    uint32_t duty_target_consecutive_cycles;
    uint8_t settle_timing_valid;
    uint8_t target_reached;
    uint8_t voltage_target_reached;
    uint8_t duty_target_settled;
    uint8_t current_stable;
    uint8_t duty_saturated;
    uint8_t overcurrent_detected;
} MotorCalibrationStageData_t;

typedef struct
{
    volatile MotorCalibrationState_t state;
    volatile uint8_t finish_pending;
    volatile MotorCalibrationState_t terminal_state;
    uint32_t cycle;
    uint8_t stage;
    RsInjectionMode_t mode;
    float duty;
    volatile uint32_t last_adc_step_tick_ms;
    volatile uint32_t stage_start_tick_ms;

    /*
     * The ADC callback runs after the CH4 trigger near the PWM top.
     * A CCR written by that callback is preloaded and transfers at the
     * following center-aligned update at CNT=0, before the next CH4 sample.
     * These fields therefore describe the command paired with the next sample;
     * they are deliberately not the command calculated in the current callback.
     */
    float next_sample_duty;
    uint16_t next_sample_ccr1;

    MotorCalibrationStageData_t stage_data[MOTOR_CALIBRATION_STAGE_COUNT];
    float r_ab_fit;
    float voltage_fit_intercept;
    float rs_all;
    float rs_2_4;
    float ideal_r_ab_2_4;
    float ideal_intercept_2_4;
    float fit_all_current_span;
    float fit_2_4_current_span;
    float fit_all_denominator;
    float fit_2_4_denominator;
    uint8_t fit_all_valid;
    uint8_t fit_2_4_valid;
    uint8_t all_stages_valid;
    uint8_t error_code;
} MotorCalibrationContext_t;

static MotorCalibrationContext_t motor_calibration;

static void MotorCalibration_DisablePhaseOutputs(void)
{
    /* MOE is the immediate power-stage gate; clear it before changing CCER. */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    __DSB();
    TIM1->CCER &= ~MOTOR_CALIBRATION_PHASE_CCER_MASK;
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
}

static void MotorCalibration_Finish(MotorCalibrationState_t terminal_state,
                                    uint8_t error_code)
{
    if ((motor_calibration.state != MOTOR_CALIBRATION_RUNNING) &&
        (motor_calibration.state != MOTOR_CALIBRATION_STARTING))
    {
        return;
    }

    /* Keep the FOC callback excluded until the main loop stops the timer. */
    motor_calibration.state = MOTOR_CALIBRATION_STOPPING;
    MotorCalibration_DisablePhaseOutputs();
    motor_calibration.terminal_state = terminal_state;
    motor_calibration.error_code = error_code;
    motor_calibration.finish_pending = 1U;
    __DMB();
}

static float MotorCalibration_Average(float sum, uint32_t count)
{
    return (count == 0U) ? NAN : (sum / (float)count);
}

static void MotorCalibration_CheckStage(uint8_t index)
{
    MotorCalibrationStageData_t *data = &motor_calibration.stage_data[index];
    float target_i = motor_calibration_stage_target_current[index];
    float target_u = rs_voltage_targets[index];
    float average_i = MotorCalibration_Average(data->sum_abs_ib,
                                                data->sample_count);
    float average_u = MotorCalibration_Average(data->sum_u_cmd,
                                                data->sample_count);
    float start_i = MotorCalibration_Average(data->sum_start_abs_ib,
                                              data->start_window_count);
    float end_i = MotorCalibration_Average(data->sum_end_abs_ib,
                                            data->end_window_count);
    float stable_delta_a = motor_calibration.mode == RS_INJECTION_VOLTAGE ?
        MOTOR_CALIBRATION_VOLTAGE_STABLE_DELTA_A :
        MOTOR_CALIBRATION_STABLE_DELTA_A;

    data->target_reached =
        (data->sample_count == MOTOR_CALIBRATION_SAMPLE_CYCLES) &&
        isfinite(average_i) &&
        (fabsf(average_i - target_i) <=
         target_i * MOTOR_CALIBRATION_TARGET_TOLERANCE);
    data->voltage_target_reached =
        (data->sample_count == MOTOR_CALIBRATION_SAMPLE_CYCLES) &&
        isfinite(average_u) &&
        (fabsf(average_u - target_u) <=
         target_u * MOTOR_CALIBRATION_TARGET_TOLERANCE);
    data->current_stable =
        (data->start_window_count == MOTOR_CALIBRATION_WINDOW_CYCLES) &&
        (data->end_window_count == MOTOR_CALIBRATION_WINDOW_CYCLES) &&
        isfinite(start_i) && isfinite(end_i) &&
        (fabsf(end_i - start_i) < stable_delta_a);
}

static uint8_t MotorCalibration_Fit(uint8_t first_stage,
                                    uint8_t stage_count,
                                    float *slope,
                                    float *intercept,
                                    float *current_span,
                                    float *fit_denominator)
{
    float sum_i = 0.0f;
    float sum_u = 0.0f;
    float sum_ii = 0.0f;
    float sum_iu = 0.0f;
    float min_i = INFINITY;
    float max_i = -INFINITY;
    float denominator;
    uint8_t offset;

    *slope = NAN;
    *intercept = NAN;
    *current_span = NAN;
    *fit_denominator = NAN;

    for (offset = 0U; offset < stage_count; ++offset)
    {
        uint8_t index = (uint8_t)(first_stage + offset);
        const MotorCalibrationStageData_t *data =
            &motor_calibration.stage_data[index];
        float average_i;
        float average_u;

        if (data->sample_count == 0U)
        {
            return 0U;
        }

        average_i = MotorCalibration_Average(
            data->sum_abs_ib, data->sample_count);
        average_u = MotorCalibration_Average(
            data->sum_u_cmd, data->sample_count);
        if ((!isfinite(average_i)) || (!isfinite(average_u)))
        {
            return 0U;
        }

        if (average_i < min_i)
        {
            min_i = average_i;
        }
        if (average_i > max_i)
        {
            max_i = average_i;
        }

        sum_i += average_i;
        sum_u += average_u;
        sum_ii += average_i * average_i;
        sum_iu += average_i * average_u;
    }

    *current_span = max_i - min_i;
    denominator = (float)stage_count * sum_ii - sum_i * sum_i;
    *fit_denominator = denominator;
    if ((!isfinite(*current_span)) || (!isfinite(denominator)) ||
        (*current_span <= MOTOR_CALIBRATION_MIN_CURRENT_SPREAD_A) ||
        (denominator <= MOTOR_CALIBRATION_FIT_DENOMINATOR_MIN))
    {
        return 0U;
    }
    if ((sum_i / (float)stage_count) < MOTOR_CALIBRATION_MIN_CURRENT_A)
    {
        return 0U;
    }

    *slope = ((float)stage_count * sum_iu - sum_i * sum_u) /
             denominator;
    *intercept = (sum_u - (*slope * sum_i)) / (float)stage_count;
    if ((!isfinite(*slope)) || (!isfinite(*intercept)) || (*slope <= 0.0f))
    {
        *slope = NAN;
        *intercept = NAN;
        return 0U;
    }

    return 1U;
}

static void MotorCalibration_ComputeFits(void)
{
    float slope_all;
    float intercept_all;
    float span_all;
    float slope_2_4;
    float intercept_2_4;
    float span_2_4;

    motor_calibration.fit_all_valid = MotorCalibration_Fit(
        0U, MOTOR_CALIBRATION_STAGE_COUNT,
        &slope_all, &intercept_all, &span_all,
        &motor_calibration.fit_all_denominator);
    motor_calibration.fit_2_4_valid = MotorCalibration_Fit(
        1U, MOTOR_CALIBRATION_STAGE_COUNT - 1U,
        &slope_2_4, &intercept_2_4, &span_2_4,
        &motor_calibration.fit_2_4_denominator);

    motor_calibration.r_ab_fit = slope_all;
    motor_calibration.voltage_fit_intercept = intercept_all;
    motor_calibration.fit_all_current_span = span_all;
    motor_calibration.fit_2_4_current_span = span_2_4;
    motor_calibration.rs_all = motor_calibration.fit_all_valid != 0U ?
                               (slope_all * 0.5f) : NAN;
    motor_calibration.rs_2_4 = motor_calibration.fit_2_4_valid != 0U ?
                               (slope_2_4 * 0.5f) : NAN;
    motor_calibration.ideal_r_ab_2_4 = slope_2_4;
    motor_calibration.ideal_intercept_2_4 = intercept_2_4;
}

static void MotorCalibration_ResetData(void)
{
    uint8_t index;

    motor_calibration.cycle = 0U;
    motor_calibration.stage = 0U;
    motor_calibration.duty = 0.0f;
    motor_calibration.last_adc_step_tick_ms = 0U;
    motor_calibration.stage_start_tick_ms = 0U;
    motor_calibration.finish_pending = 0U;
    motor_calibration.next_sample_duty = 0.0f;
    motor_calibration.next_sample_ccr1 = 0U;
    motor_calibration.r_ab_fit = NAN;
    motor_calibration.voltage_fit_intercept = NAN;
    motor_calibration.rs_all = NAN;
    motor_calibration.rs_2_4 = NAN;
    motor_calibration.ideal_r_ab_2_4 = NAN;
    motor_calibration.ideal_intercept_2_4 = NAN;
    motor_calibration.fit_all_valid = 0U;
    motor_calibration.fit_2_4_valid = 0U;
    motor_calibration.all_stages_valid = 0U;
    motor_calibration.fit_all_current_span = NAN;
    motor_calibration.fit_2_4_current_span = NAN;
    motor_calibration.fit_all_denominator = NAN;
    motor_calibration.fit_2_4_denominator = NAN;
    motor_calibration.error_code = 0U;
    motor_calibration.terminal_state = MOTOR_CALIBRATION_ERROR;

    for (index = 0U; index < MOTOR_CALIBRATION_STAGE_COUNT; ++index)
    {
        MotorCalibrationStageData_t *data = &motor_calibration.stage_data[index];
        data->sum_ib = 0.0f;
        data->sum_abs_ib = 0.0f;
        data->sum_adc_b = 0U;
        data->sum_ccr1 = 0U;
        data->sum_vbus = 0.0f;
        data->sum_u_cmd = 0.0f;
        data->sum_start_abs_ib = 0.0f;
        data->sum_end_abs_ib = 0.0f;
        data->sum_start_duty = 0.0f;
        data->sum_end_duty = 0.0f;
        data->min_duty = INFINITY;
        data->max_duty = -INFINITY;
        data->min_ib = INFINITY;
        data->max_ib = -INFINITY;
        data->max_abs_ib_seen = 0.0f;
        data->sample_count = 0U;
        data->start_window_count = 0U;
        data->end_window_count = 0U;
        data->settle_start_tick_ms = 0U;
        data->settle_elapsed_ms = 0U;
        data->duty_target_consecutive_cycles = 0U;
        data->settle_timing_valid = 0U;
        data->target_reached = 0U;
        data->voltage_target_reached = 0U;
        data->duty_target_settled = 0U;
        data->current_stable = 0U;
        data->duty_saturated = 0U;
        data->overcurrent_detected = 0U;
    }
}

static void MotorCalibration_PrintHeader(void)
{
    float pwm_frequency_hz;

    pwm_frequency_hz = (foc.timer.pwm_arr == 0U) ? NAN :
        ((float)foc.timer.clock_freq /
         (2.0f * (float)foc.timer.pwm_arr));

    DebugConsole_Printf("RS_IDENTIFY STARTED\r\n");
    DebugConsole_Printf("RS_DIAG_REV=VOLTAGE_INJECTION_V7\r\n");
    DebugConsole_Printf("INJECTION_MODE=%s DUTY_STEP=%.7f DUTY_SETTLE_CYCLES=%u\r\n",
        motor_calibration.mode == RS_INJECTION_VOLTAGE ? "VOLTAGE" : "CURRENT",
        (double)MOTOR_CALIBRATION_VOLTAGE_DUTY_STEP,
        (unsigned int)MOTOR_CALIBRATION_DUTY_SETTLE_CYCLES);
    DebugConsole_Printf("CURRENT_PI_ACTIVE=%s\r\n",
        motor_calibration.mode == RS_INJECTION_CURRENT ? "YES" : "NO");
    DebugConsole_Printf("CURRENT_STABLE_DELTA_LIMIT_A=%.6f\r\n",
        (double)(motor_calibration.mode == RS_INJECTION_VOLTAGE ?
            MOTOR_CALIBRATION_VOLTAGE_STABLE_DELTA_A :
            MOTOR_CALIBRATION_STABLE_DELTA_A));
    DebugConsole_Printf("RS_IDENTIFY_MAX_CURRENT=%.2f ADC_STALL_TIMEOUT_MS=%u STAGE_TIMEOUT_MS=%u\r\n",
        (double)RS_IDENTIFY_MAX_CURRENT_A,
        (unsigned int)RS_IDENTIFY_ADC_STALL_TIMEOUT_MS,
        (unsigned int)RS_IDENTIFY_STAGE_TIMEOUT_MS);
    DebugConsole_Printf("VOLTAGE_MODE_VBUS_RANGE=[1.0,%.1f] V\r\n",
        (double)RS_IDENTIFY_VOLTAGE_VBUS_MAX_V);
    DebugConsole_Printf("VOLTAGE_MODEL=IDEAL_COMMAND AB_DIFFERENTIAL_VERIFIED=NO\r\n");
    DebugConsole_Printf(
        "CURRENT_KP=%.6f SETTLE_CYCLES=%u SAMPLE_CYCLES=%u WINDOW_CYCLES=%u\r\n",
        (double)MOTOR_CALIBRATION_CURRENT_KP,
        (unsigned int)MOTOR_CALIBRATION_SETTLE_CYCLES,
        (unsigned int)MOTOR_CALIBRATION_SAMPLE_CYCLES,
        (unsigned int)MOTOR_CALIBRATION_WINDOW_CYCLES);
    DebugConsole_Printf(
        "PWM_FREQ=%.3f PWM_ARR=%lu DEADTIME_DTG=%u VBUS=%.6f TARGET_TOL=+/-%.1f%%\r\n",
        (double)pwm_frequency_hz,
        (unsigned long)foc.timer.pwm_arr,
        (unsigned int)foc.timer.dead_time,
        (double)foc.state.vbus,
        (double)(MOTOR_CALIBRATION_TARGET_TOLERANCE * 100.0f));
    DebugConsole_Printf("PWM_CLOCK_HZ=%lu TIM1_ARR_REG=%lu\r\n",
        (unsigned long)foc.timer.clock_freq,
        (unsigned long)TIM1->ARR);
    DebugConsole_Printf(
        "ADC_OFFSET_A=%.3f ADC_OFFSET_B=%.3f ADC_OFFSET_C=%.3f\r\n",
        (double)foc.calibration.ia_offset,
        (double)foc.calibration.ib_offset,
        (double)foc.calibration.ic_offset);
    DebugConsole_Printf(
        "ADC_GAIN_A=%.12g ADC_GAIN_B=%.12g ADC_GAIN_C=%.12g\r\n",
        (double)foc.current.gain_a,
        (double)foc.current.gain_b,
        (double)foc.current.gain_c);
}

static uint8_t MotorCalibration_StageSamplesValid(
    const MotorCalibrationStageData_t *data)
{
    return data->sample_count == MOTOR_CALIBRATION_SAMPLE_CYCLES;
}

static uint8_t MotorCalibration_StageCurrentValid(
    const MotorCalibrationStageData_t *data)
{
    return (MotorCalibration_StageSamplesValid(data) != 0U) &&
           (((motor_calibration.mode == RS_INJECTION_VOLTAGE) &&
             (data->duty_target_settled != 0U) &&
             (data->voltage_target_reached != 0U)) ||
            ((motor_calibration.mode == RS_INJECTION_CURRENT) &&
             (data->target_reached != 0U))) &&
           (data->current_stable != 0U);
}

static uint8_t MotorCalibration_AdcHasCurrentHeadroom(void)
{
    const float gains[3] = {
        foc.current.gain_a, foc.current.gain_b, foc.current.gain_c
    };
    const float offsets[3] = {
        foc.calibration.ia_offset,
        foc.calibration.ib_offset,
        foc.calibration.ic_offset
    };
    uint8_t index;

    /* Four unshifted 12-bit conversions produce a 0..16380 JDR sum.
     * This checks digital headroom only, not analogue linearity or protection. */
    for (index = 0U; index < 3U; ++index)
    {
        float threshold_codes;
        if ((!isfinite(gains[index])) || (gains[index] <= 0.0f) ||
            (!isfinite(offsets[index])))
        {
            return 0U;
        }
        threshold_codes = RS_IDENTIFY_MAX_CURRENT_A / gains[index];
        if ((!isfinite(threshold_codes)) ||
            (offsets[index] <= threshold_codes) ||
            ((RS_IDENTIFY_ADC_SUM_MAX - offsets[index]) <= threshold_codes))
        {
            return 0U;
        }
    }
    return 1U;
}

static const char *MotorCalibration_AbortReason(uint8_t error_code)
{
    switch (error_code)
    {
    case 0U: return "NONE";
    case 1U: return "STOP_REQUESTED";
    case 2U: return "OVERCURRENT";
    case 3U: return "FIT_INVALID";
    case 4U: return "INVALID_SAMPLE";
    case 5U: return "STAGE_INVALID";
    case 6U: return "DUTY_SATURATED";
    case 7U: return "ADC_STALL";
    case 8U: return "STAGE_TIMEOUT";
    case 9U: return "ADC_RANGE_INVALID";
    case 10U: return "DUTY_NOT_SETTLED";
    case 11U: return "VBUS_INVALID";
    case 12U: return "CURRENT_UNSTABLE";
    case 13U: return "VOLTAGE_TARGET_NOT_REACHED";
    default: return "UNKNOWN";
    }
}

static void MotorCalibration_PrintReport(void)
{
    uint8_t index;

    for (index = 0U; index < MOTOR_CALIBRATION_STAGE_COUNT; ++index)
    {
        const MotorCalibrationStageData_t *data =
            &motor_calibration.stage_data[index];
        float average_ib = MotorCalibration_Average(
            data->sum_ib, data->sample_count);
        float average_abs_ib = MotorCalibration_Average(
            data->sum_abs_ib, data->sample_count);
        float average_adc_b = data->sample_count == 0U ? NAN :
            (float)data->sum_adc_b / (float)data->sample_count;
        float average_ccr1 = data->sample_count == 0U ? NAN :
            (float)data->sum_ccr1 / (float)data->sample_count;
        float average_duty = foc.timer.pwm_arr == 0U ? NAN :
            average_ccr1 / (float)(foc.timer.pwm_arr + 1U);
        float average_vbus = MotorCalibration_Average(
            data->sum_vbus, data->sample_count);
        float average_u = MotorCalibration_Average(
            data->sum_u_cmd, data->sample_count);
        float average_start_i = MotorCalibration_Average(
            data->sum_start_abs_ib, data->start_window_count);
        float average_end_i = MotorCalibration_Average(
            data->sum_end_abs_ib, data->end_window_count);
        float average_start_duty = MotorCalibration_Average(
            data->sum_start_duty, data->start_window_count);
        float average_end_duty = MotorCalibration_Average(
            data->sum_end_duty, data->end_window_count);
        float measured_adc_rate_hz =
            (data->settle_timing_valid != 0U) &&
            (data->settle_elapsed_ms != 0U) ?
            (float)MOTOR_CALIBRATION_SETTLE_CYCLES * 1000.0f /
            (float)data->settle_elapsed_ms : NAN;
        float min_ib = data->sample_count == 0U ? NAN : data->min_ib;
        float max_ib = data->sample_count == 0U ? NAN : data->max_ib;
        float min_duty = data->sample_count == 0U ? NAN : data->min_duty;
        float max_duty = data->sample_count == 0U ? NAN : data->max_duty;

        DebugConsole_Printf("STAGE %u\r\n", (unsigned int)(index + 1U));
        if (motor_calibration.mode == RS_INJECTION_VOLTAGE)
        {
            DebugConsole_Printf("TARGET_U=%.6f\r\n",
                (double)rs_voltage_targets[index]);
        }
        else
        {
            DebugConsole_Printf("TARGET_I=%.6f\r\n",
                (double)motor_calibration_stage_target_current[index]);
        }
        DebugConsole_Printf("IB_AVG=%.6f FABS_IB_AVG=%.6f\r\n",
            (double)average_ib, (double)average_abs_ib);
        DebugConsole_Printf(
            "ADC_B_AVG=%.3f DUTY_AVG=%.8f CCR1_AVG=%.3f VBUS_AVG=%.6f U_CMD_AVG=%.6f U_IDEAL_AVG=%.6f\r\n",
            (double)average_adc_b, (double)average_duty,
            (double)average_ccr1, (double)average_vbus,
            (double)average_u, (double)average_u);
        DebugConsole_Printf("DUTY_MIN=%.8f DUTY_MAX=%.8f IB_MIN=%.6f IB_MAX=%.6f\r\n",
            (double)min_duty, (double)max_duty,
            (double)min_ib, (double)max_ib);
        DebugConsole_Printf(
            "I_START_AVG=%.6f I_END_AVG=%.6f DUTY_START_AVG=%.8f DUTY_END_AVG=%.8f\r\n",
            (double)average_start_i, (double)average_end_i,
            (double)average_start_duty, (double)average_end_duty);
        DebugConsole_Printf(
            "TARGET_REACHED=%s VOLTAGE_TARGET_REACHED=%s DUTY_TARGET_SETTLED=%s CURRENT_STABLE=%s STAGE_VALID=%s SAMPLES=%lu\r\n",
            motor_calibration.mode == RS_INJECTION_CURRENT ?
                (data->target_reached != 0U ? "YES" : "NO") : "NA",
            motor_calibration.mode == RS_INJECTION_VOLTAGE ?
                (data->voltage_target_reached != 0U ? "YES" : "NO") : "NA",
            motor_calibration.mode == RS_INJECTION_VOLTAGE ?
                (data->duty_target_settled != 0U ? "YES" : "NO") : "NA",
            data->current_stable != 0U ? "YES" : "NO",
            MotorCalibration_StageCurrentValid(data) != 0U ? "YES" : "NO",
            (unsigned long)data->sample_count);
        DebugConsole_Printf(
            "IB_ABS_PEAK_ADC=%.6f DUTY_SATURATED=%s OVERCURRENT_DETECTED=%s\r\n",
            (double)data->max_abs_ib_seen,
            data->duty_saturated != 0U ? "YES" : "NO",
            data->overcurrent_detected != 0U ? "YES" : "NO");
        DebugConsole_Printf(
            "SETTLE_MEASURED=%s SETTLE_MS=%lu ADC_STEP_HZ_EST=%.1f\r\n",
            data->settle_timing_valid != 0U ? "YES" : "NO",
            (unsigned long)data->settle_elapsed_ms,
            (double)measured_adc_rate_hz);
    }

    DebugConsole_Printf("DATA_VALID=%s RESULT_VALID=%s\r\n",
        motor_calibration.all_stages_valid != 0U ? "YES" : "NO",
        (motor_calibration.all_stages_valid != 0U &&
         motor_calibration.fit_all_valid != 0U) ? "YES" : "NO");
    DebugConsole_Printf(
        "FIT_ALL_VALID=%s FIT_2_4_VALID=%s FIT_SPAN_ALL=%.6f FIT_SPAN_2_4=%.6f\r\n",
        motor_calibration.fit_all_valid != 0U ? "YES" : "NO",
        motor_calibration.fit_2_4_valid != 0U ? "YES" : "NO",
        (double)motor_calibration.fit_all_current_span,
        (double)motor_calibration.fit_2_4_current_span);
    DebugConsole_Printf(
        "FIT_DENOM_ALL=%.9g FIT_DENOM_2_4=%.9g DENOM_MIN=%.9g\r\n",
        (double)motor_calibration.fit_all_denominator,
        (double)motor_calibration.fit_2_4_denominator,
        (double)MOTOR_CALIBRATION_FIT_DENOMINATOR_MIN);
    DebugConsole_Printf(
        "R_AB_FIT=%.6f VOLTAGE_FIT_INTERCEPT=%.6f RS_ALL=%.6f RS_2_4=%.6f\r\n",
        (double)motor_calibration.r_ab_fit,
        (double)motor_calibration.voltage_fit_intercept,
        (double)motor_calibration.rs_all,
        (double)motor_calibration.rs_2_4);
    DebugConsole_Printf("RS_IDENTIFY RESULT\r\n");
    DebugConsole_Printf(
        "RS_IDENTIFY_MAX_CURRENT=%.2f RS_IDENTIFY_ABORT_REASON=%s\r\n",
        (double)RS_IDENTIFY_MAX_CURRENT_A,
        MotorCalibration_AbortReason(motor_calibration.error_code));
    DebugConsole_Printf("RS_IDEAL_ALL=%.6f RS_IDEAL_2_4=%.6f\r\n",
        (double)motor_calibration.rs_all,
        (double)motor_calibration.rs_2_4);
    DebugConsole_Printf(
        "R_AB_IDEAL=%.6f R_AB_IDEAL_2_4=%.6f IDEAL_INTERCEPT=%.6f IDEAL_2_4_INTERCEPT=%.6f\r\n",
        (double)motor_calibration.r_ab_fit,
        (double)motor_calibration.ideal_r_ab_2_4,
        (double)motor_calibration.voltage_fit_intercept,
        (double)motor_calibration.ideal_intercept_2_4);
    DebugConsole_Printf(
        "VOLTAGE_MODEL=IDEAL_COMMAND AB_DIFFERENTIAL_VERIFIED=NO\r\n");
}


HAL_StatusTypeDef MotorCalibration_StartMode(RsInjectionMode_t mode)
{
    HAL_StatusTypeDef status;
    uint32_t adc_trigger_mask = TIM_CCER_CC4E;
    uint32_t primask;

    if ((mode != RS_INJECTION_VOLTAGE) && (mode != RS_INJECTION_CURRENT))
    {
        return HAL_ERROR;
    }

    if ((motor_calibration.state == MOTOR_CALIBRATION_STARTING) ||
        (motor_calibration.state == MOTOR_CALIBRATION_RUNNING) ||
        (motor_calibration.state == MOTOR_CALIBRATION_STOPPING) ||
        (motor_calibration.finish_pending != 0U))
    {
        return HAL_BUSY;
    }

    if ((foc_motor_state != FOC_MOTOR_IDLE) ||
        (foc.calibration.calibrated == 0U) ||
        (!isfinite(foc.state.vbus)) ||
        (foc.state.vbus < 1.0f) ||
        ((mode == RS_INJECTION_VOLTAGE) &&
         (foc.state.vbus > RS_IDENTIFY_VOLTAGE_VBUS_MAX_V)))
    {
        return HAL_ERROR;
    }
    if (MotorCalibration_AdcHasCurrentHeadroom() == 0U)
    {
        motor_calibration.error_code = 9U;
        motor_calibration.state = MOTOR_CALIBRATION_ERROR;
        DebugConsole_Printf("RS_IDENTIFY_ABORT_REASON=ADC_RANGE_INVALID\r\n");
        return HAL_ERROR;
    }

    /* Reserve the motor state before touching PWM; ADC callbacks now bypass FOC. */
    motor_calibration.state = MOTOR_CALIBRATION_STARTING;
    foc_motor_state = FOC_MOTOR_CALIBRATION;
    __DMB();
    MotorCalibration_ResetData();
    motor_calibration.mode = mode;
    motor_calibration.state = MOTOR_CALIBRATION_STARTING;

    /* Disable the power stage before stopping/reconfiguring any timer channel. */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    __DSB();
    TIM1->CCER &= ~MOTOR_CALIBRATION_PHASE_CCER_MASK;

    /* Synchronize HAL channel state as well as hardware state, while MOE is off. */
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER &= ~(MOTOR_CALIBRATION_PHASE_CCER_MASK | adc_trigger_mask);
    TIM1->CR1 &= ~TIM_CR1_CEN;

    /* Latch a zero-current compare image before enabling any phase output. */
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    TIM1->CCR4 = foc.timer.adc_trigger;
    TIM1->CNT = 0U;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR &= ~TIM_SR_UIF;

    if (motor_calibration.state != MOTOR_CALIBRATION_STARTING)
    {
        return HAL_BUSY;
    }

    /* CH4 only restarts the injected ADC; all phase CCER bits remain clear. */
    status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    if (status != HAL_OK)
    {
        MotorCalibration_DisablePhaseOutputs();
        TIM1->CCER &= ~TIM_CCER_CC4E;
        TIM1->CR1 &= ~TIM_CR1_CEN;
        foc_motor_state = FOC_MOTOR_IDLE;
        motor_calibration.state = MOTOR_CALIBRATION_IDLE;
        return status;
    }

    /* HAL_TIM_PWM_Start asserts MOE; with phase CCER clear that is still safe. */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CNT = 0U;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR &= ~TIM_SR_UIF;

    if (motor_calibration.state != MOTOR_CALIBRATION_STARTING)
    {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        return HAL_BUSY;
    }

    /* With CCR2=0, CH2N is complementary only when CH2 and CH2N are enabled. */
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE |
                  TIM_CCER_CC2E | TIM_CCER_CC2NE;
    __DSB();

    /*
     * Make the final enable and STARTING->RUNNING transition atomic against
     * GPIO stop requests. An abort before this point leaves MOE disabled.
     */
    primask = __get_PRIMASK();
    __disable_irq();
    if (motor_calibration.state != MOTOR_CALIBRATION_STARTING)
    {
        MotorCalibration_DisablePhaseOutputs();
        if (primask == 0U)
        {
            __enable_irq();
        }
        return HAL_BUSY;
    }

    /* Enable only after CCR preload, update event, and bridge selection are ready. */
    motor_calibration.last_adc_step_tick_ms = HAL_GetTick();
    motor_calibration.stage_start_tick_ms =
        motor_calibration.last_adc_step_tick_ms;
    TIM1->BDTR |= TIM_BDTR_MOE;
    motor_calibration.state = MOTOR_CALIBRATION_RUNNING;
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }

    MotorCalibration_PrintHeader();
    return HAL_OK;
}

HAL_StatusTypeDef MotorCalibration_Start(void)
{
    return MotorCalibration_StartMode(RS_INJECTION_VOLTAGE);
}

void MotorCalibration_Stop(void)
{
    MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 1U);
}

void MotorCalibration_AdcStep(uint16_t adc_b_raw)
{
    float signed_ia;
    float signed_ib;
    float signed_ic;
    float current_a;
    float target_a;
    float target_duty;
    float error_a;
    float sample_duty;
    float sample_vbus;
    float sample_u_ideal;
    MotorCalibrationStageData_t *data;
    uint16_t sample_ccr1;
    uint16_t next_ccr1;

    if (motor_calibration.state != MOTOR_CALIBRATION_RUNNING)
    {
        return;
    }

    motor_calibration.last_adc_step_tick_ms = HAL_GetTick();
    data = &motor_calibration.stage_data[motor_calibration.stage];
    signed_ia = foc.state.i_abc.a;
    signed_ib = foc.state.i_abc.b;
    signed_ic = foc.state.i_abc.c;
    current_a = fabsf(signed_ib);
    if ((!isfinite(signed_ia)) || (!isfinite(signed_ib)) ||
        (!isfinite(signed_ic)) || (!isfinite(foc.state.vbus)))
    {
        MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 4U);
        return;
    }
    if (current_a > data->max_abs_ib_seen)
    {
        data->max_abs_ib_seen = current_a;
    }
    if ((fabsf(signed_ia) > RS_IDENTIFY_MAX_CURRENT_A) ||
        (current_a > RS_IDENTIFY_MAX_CURRENT_A) ||
        (fabsf(signed_ic) > RS_IDENTIFY_MAX_CURRENT_A))
    {
        data->overcurrent_detected = 1U;
        MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 2U);
        return;
    }
    if ((motor_calibration.last_adc_step_tick_ms -
         motor_calibration.stage_start_tick_ms) >
        RS_IDENTIFY_STAGE_TIMEOUT_MS)
    {
        MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 8U);
        return;
    }

    /* Pair this ADC result with the command applied since the prior trigger. */
    sample_duty = motor_calibration.next_sample_duty;
    sample_ccr1 = motor_calibration.next_sample_ccr1;
    sample_vbus = foc.state.vbus;
    if ((sample_vbus < 1.0f) ||
        ((motor_calibration.mode == RS_INJECTION_VOLTAGE) &&
         (sample_vbus > RS_IDENTIFY_VOLTAGE_VBUS_MAX_V)))
    {
        MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 11U);
        return;
    }
    /* The quantized CCR paired with this ADC sample defines command voltage. */
    if ((!isfinite(sample_duty)) || (sample_duty < 0.0f) ||
        (sample_duty > MOTOR_CALIBRATION_MAX_DUTY))
    {
        MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 4U);
        return;
    }
    sample_u_ideal = sample_duty * sample_vbus;

    if (motor_calibration.cycle == 0U)
    {
        data->settle_start_tick_ms = HAL_GetTick();
    }

    if (motor_calibration.mode == RS_INJECTION_VOLTAGE)
    {
        target_duty = rs_voltage_targets[motor_calibration.stage] / sample_vbus;
        if ((!isfinite(target_duty)) || (target_duty < 0.0f))
        {
            MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 11U);
            return;
        }
        if (target_duty > MOTOR_CALIBRATION_MAX_DUTY)
        {
            data->duty_saturated = 1U;
            MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 6U);
            return;
        }
        /* One bounded CCR step per ADC period, including stage transitions. */
        if (motor_calibration.duty < target_duty)
        {
            motor_calibration.duty = fminf(
                motor_calibration.duty + MOTOR_CALIBRATION_VOLTAGE_DUTY_STEP,
                target_duty);
        }
        else
        {
            motor_calibration.duty = fmaxf(
                motor_calibration.duty - MOTOR_CALIBRATION_VOLTAGE_DUTY_STEP,
                target_duty);
        }
    }
    else
    {
        target_a = motor_calibration_stage_target_current[motor_calibration.stage];
        error_a = target_a - current_a;
        /* Retained V6 current-mode integrator for explicit A/B tests. */
        motor_calibration.duty += MOTOR_CALIBRATION_CURRENT_KP * error_a;
        if (motor_calibration.duty < 0.0f)
        {
            motor_calibration.duty = 0.0f;
        }
        if ((motor_calibration.duty >= MOTOR_CALIBRATION_MAX_DUTY) &&
            (error_a > 0.0f))
        {
            motor_calibration.duty = MOTOR_CALIBRATION_MAX_DUTY;
            data->duty_saturated = 1U;
            MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 6U);
            return;
        }
    }

    next_ccr1 = (uint16_t)FOC_DutyToCCR(
        motor_calibration.duty, foc.timer.pwm_arr);
    TIM1->CCR1 = next_ccr1;
    /* Record the quantized CCR ratio that the next PWM update applies. */
    motor_calibration.next_sample_duty =
        (float)next_ccr1 / (float)(foc.timer.pwm_arr + 1U);
    motor_calibration.next_sample_ccr1 = next_ccr1;

    if (motor_calibration.cycle < MOTOR_CALIBRATION_SETTLE_CYCLES)
    {
        if (motor_calibration.mode == RS_INJECTION_VOLTAGE)
        {
            /* The paired duty was applied before this ADC sample. */
            if (fabsf(sample_duty - target_duty) <=
                MOTOR_CALIBRATION_DUTY_SETTLE_TOLERANCE)
            {
                data->duty_target_consecutive_cycles++;
            }
            else
            {
                data->duty_target_consecutive_cycles = 0U;
            }
        }
        motor_calibration.cycle++;
        return;
    }

    if ((motor_calibration.mode == RS_INJECTION_VOLTAGE) &&
        (data->sample_count == 0U))
    {
        data->duty_target_settled =
            data->duty_target_consecutive_cycles >=
            MOTOR_CALIBRATION_DUTY_SETTLE_CYCLES;
        if (data->duty_target_settled == 0U)
        {
            MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 10U);
            return;
        }
    }

    if (data->sample_count == 0U)
    {
        data->settle_elapsed_ms = HAL_GetTick() -
                                  data->settle_start_tick_ms;
        data->settle_timing_valid = 1U;
    }

    {
        /* The first and last 250 samples are disjoint stable-phase windows. */
        if (data->sample_count < MOTOR_CALIBRATION_WINDOW_CYCLES)
        {
            data->sum_start_abs_ib += current_a;
            data->sum_start_duty += sample_duty;
            data->start_window_count++;
        }
        if (data->sample_count >=
            MOTOR_CALIBRATION_SAMPLE_CYCLES -
            MOTOR_CALIBRATION_WINDOW_CYCLES)
        {
            data->sum_end_abs_ib += current_a;
            data->sum_end_duty += sample_duty;
            data->end_window_count++;
        }
        data->sum_ib += signed_ib;
        data->sum_abs_ib += current_a;
        data->sum_adc_b += (uint64_t)adc_b_raw;
        if (sample_duty < data->min_duty)
        {
            data->min_duty = sample_duty;
        }
        if (sample_duty > data->max_duty)
        {
            data->max_duty = sample_duty;
        }
        data->sum_ccr1 += (uint64_t)sample_ccr1;
        data->sum_vbus += sample_vbus;
        data->sum_u_cmd += sample_u_ideal;
        if (data->sample_count == 0U)
        {
            data->min_ib = signed_ib;
            data->max_ib = signed_ib;
        }
        else
        {
            if (signed_ib < data->min_ib)
            {
                data->min_ib = signed_ib;
            }
            if (signed_ib > data->max_ib)
            {
                data->max_ib = signed_ib;
            }
        }
        data->sample_count++;
    }

    motor_calibration.cycle++;
    if (motor_calibration.stage_data[motor_calibration.stage].sample_count <
        MOTOR_CALIBRATION_SAMPLE_CYCLES)
    {
        return;
    }

    if (motor_calibration.mode == RS_INJECTION_VOLTAGE)
    {
        MotorCalibration_CheckStage(motor_calibration.stage);
        if (data->current_stable == 0U)
        {
            MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 12U);
            return;
        }
        if (data->voltage_target_reached == 0U)
        {
            MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 13U);
            return;
        }
    }

    motor_calibration.cycle = 0U;
    motor_calibration.stage++;
    if (motor_calibration.stage < MOTOR_CALIBRATION_STAGE_COUNT)
    {
        motor_calibration.stage_start_tick_ms = HAL_GetTick();
        return;
    }

    /* Fit and validity checks run in the main loop after outputs are off. */
    MotorCalibration_Finish(MOTOR_CALIBRATION_DONE, 0U);
}

void MotorCalibration_Process(void)
{
    MotorCalibrationState_t terminal_state;
    uint8_t index;

    if (motor_calibration.state == MOTOR_CALIBRATION_RUNNING)
    {
        uint32_t now = HAL_GetTick();
        if ((now - motor_calibration.last_adc_step_tick_ms) >
            RS_IDENTIFY_ADC_STALL_TIMEOUT_MS)
        {
            MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 7U);
        }
        else if ((now - motor_calibration.stage_start_tick_ms) >
                 RS_IDENTIFY_STAGE_TIMEOUT_MS)
        {
            MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 8U);
        }
    }

    if (motor_calibration.finish_pending == 0U)
    {
        return;
    }

    terminal_state = motor_calibration.terminal_state;
    motor_calibration.finish_pending = 0U;

    /* Power off first; stop CH4/timer only after the phase outputs are gated. */
    MotorCalibration_DisablePhaseOutputs();
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER &= ~(MOTOR_CALIBRATION_PHASE_CCER_MASK | TIM_CCER_CC4E);
    TIM1->CR1 &= ~TIM_CR1_CEN;
    foc.state.ibus_est = 0.0f;
    foc.state.ibus_filter = 0.0f;

    if (terminal_state == MOTOR_CALIBRATION_DONE)
    {
        motor_calibration.all_stages_valid = 1U;
        for (index = 0U; index < MOTOR_CALIBRATION_STAGE_COUNT; ++index)
        {
            MotorCalibration_CheckStage(index);
            if (MotorCalibration_StageCurrentValid(
                    &motor_calibration.stage_data[index]) == 0U)
            {
                motor_calibration.all_stages_valid = 0U;
            }
        }

        MotorCalibration_ComputeFits();
        if (motor_calibration.fit_all_valid == 0U)
        {
            terminal_state = MOTOR_CALIBRATION_ERROR;
            motor_calibration.error_code = 3U;
        }
        else if (motor_calibration.all_stages_valid == 0U)
        {
            terminal_state = MOTOR_CALIBRATION_ERROR;
            motor_calibration.error_code = 5U;
        }
        motor_calibration.terminal_state = terminal_state;
    }

    /* No saved CCER is restored: leave every power output disabled. */
    foc_motor_state = FOC_MOTOR_IDLE;
    motor_calibration.state = terminal_state;
    __DMB();

    MotorCalibration_PrintReport();
    if (terminal_state == MOTOR_CALIBRATION_DONE)
    {
        DebugConsole_Printf("RS_IDENTIFY DONE Rs=%.6f ohm\r\n",
                            (double)motor_calibration.rs_all);
    }
    else
    {
        DebugConsole_Printf("RS_IDENTIFY ERROR code=%u\r\n",
                            (unsigned int)motor_calibration.error_code);
    }
}

uint8_t MotorCalibration_IsActive(void)
{
    return ((motor_calibration.state == MOTOR_CALIBRATION_STARTING) ||
            (motor_calibration.state == MOTOR_CALIBRATION_RUNNING) ||
            (motor_calibration.state == MOTOR_CALIBRATION_STOPPING)) ? 1U : 0U;
}

MotorCalibrationState_t MotorCalibration_GetState(void)
{
    return motor_calibration.state;
}

float MotorCalibration_GetResultOhm(void)
{
    return (motor_calibration.state == MOTOR_CALIBRATION_DONE) ?
           motor_calibration.rs_all : NAN;
}

uint8_t MotorCalibration_GetErrorCode(void)
{
    return motor_calibration.error_code;
}
