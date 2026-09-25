#ifndef MOTOR_CALIBRATION_LS_H
#define MOTOR_CALIBRATION_LS_H

#include <stddef.h>
#include <stdint.h>

/* Ls 瞬态拟合接口；硬件采集和拟合分离，便于离线验证计算结果。 */
typedef enum {
    LS_SAMPLE_IDLE = 0,
    LS_SAMPLE_RISE,
    LS_SAMPLE_DECAY,
    LS_SAMPLE_INVALID
} LsSampleState_t;

typedef struct {
    uint16_t adc_raw;
    uint16_t state;
    uint32_t timestamp; /* 同一硬件时基的计数值。 */
} LsSample_t;

typedef enum {
    LS_FIT_OK = 0,
    LS_FIT_BAD_ARGUMENT,
    LS_FIT_SAMPLE_SHORT,
    LS_FIT_TIMING,
    LS_FIT_ADC_OVERRUN,
    LS_FIT_OVERCURRENT,
    LS_FIT_SATURATED,
    LS_FIT_POOR_QUALITY,
    LS_FIT_NUMERIC,
    LS_FIT_INCONSISTENT
} LsFitStatus_t;

typedef struct {
    float timestamp_tick_us; /* 硬件时间戳每计数的微秒数。 */
    uint32_t pulse_start_timestamp; /* 硬件记录的电压阶跃起点。 */
    uint32_t pulse_end_timestamp;   /* 硬件记录的高侧关断时刻。 */
    uint32_t freewheel_start_timestamp; /* 硬件记录的下桥臂续流起点。 */
    float adc_offset;        /* 原始 ADC 零电流码。 */
    float adc_gain_a_per_code;
    float applied_voltage_v; /* 实测或明确估计的 A-B 有效电压，不能直接代入 Vbus。 */
    float loop_resistance_ohm; /* 含绕组、MOS、采样及通路的估计回路电阻。 */
    float current_limit_a;
    float pulse_hard_limit_us;
    float blanking_us;       /* 上升段开关建立时间，排除此范围内样本。 */
    uint8_t adc_overrun;
} LsFitConfig_t;

typedef struct {
    LsFitStatus_t status;
    uint32_t rise_samples;
    uint32_t decay_samples;
    float rise_start_us;
    float rise_end_us;
    float decay_start_us;
    float decay_end_us;
    float sample_period_us; /* 全部相邻采样的平均周期。 */
    float tau_rise_us;
    float tau_decay_us;
    float l_ab_rise_uh;
    float l_ab_decay_uh;
    float ls_rise_uh;
    float ls_decay_uh;
    float rise_rmse_a;
    float decay_rmse_a;
    uint8_t rise_valid;
    uint8_t decay_valid;
} LsFitResult_t;

LsFitStatus_t MotorCalibration_LsFit(const LsSample_t *samples, size_t count,
                                    const LsFitConfig_t *config,
                                    LsFitResult_t *result);

#endif
