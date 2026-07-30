#ifndef VESC_SENSORLESS_FOC_H
#define VESC_SENSORLESS_FOC_H

#include "vesc_current_controller.h"
#include "vesc_observer.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*VESC_SinCosFn_t)(float angle_rad,
                                float *sin_value,
                                float *cos_value);

typedef enum {
    VESC_FOC_STATE_IDLE = 0,
    VESC_FOC_STATE_ALIGN,
    VESC_FOC_STATE_OPEN_LOOP_RAMP,
    VESC_FOC_STATE_OPEN_LOOP_HOLD,
    VESC_FOC_STATE_TRANSITION,
    VESC_FOC_STATE_CLOSED_LOOP,
    VESC_FOC_STATE_FAULT
} VESC_FOC_State_t;

typedef struct {
    VESC_ObserverConfig_t observer;
    VESC_CurrentControllerConfig_t current;

    float align_time_s;
    float align_current_a;
    float align_current_ramp_a_per_s;
    float align_max_voltage_v;

    float open_loop_ramp_time_s;
    float open_loop_hold_time_s;
    float open_loop_timeout_s;
    float open_loop_speed_start_rad_s;
    float open_loop_speed_end_rad_s;
    float startup_iq_a;
    float startup_max_voltage_v;

    float transition_time_s;
    float transition_min_omega_rad_s;
    float transition_speed_ratio_min;
    float transition_speed_ratio_max;
    uint32_t transition_valid_samples;
    uint32_t transition_loss_samples; /* consecutive bad samples before fault */
    uint32_t closed_loop_loss_samples;

    float iq_ramp_a_per_s;
    float max_current_a;

    VESC_SinCosFn_t sincos_fn; /* NULL selects sinf/cosf */
} VESC_SensorlessFOC_Config_t;

typedef struct {
    uint8_t enable;
    float iq_ref_a;
    float i_alpha;
    float i_beta;
    float duty_a;       /* previous PWM cycle */
    float duty_b;
    float duty_c;
    float vbus;
} VESC_SensorlessFOC_Input_t;

typedef struct {
    VESC_FOC_State_t state;

    float phase_control;
    float phase_observer;
    float phase_pll;
    float phase_open_loop;
    float transition_offset;
    float omega_e;

    float id_ref;
    float iq_ref;
    float id;
    float iq;

    float ud;
    float uq;
    float u_alpha;
    float u_beta;

    float observer_flux_mag;
    float observer_error;
    float observer_duty_est;
    uint8_t observer_valid;
    uint8_t voltage_saturated;
    uint8_t fault_code;
} VESC_SensorlessFOC_Output_t;

typedef struct {
    VESC_SensorlessFOC_Config_t config;
    VESC_Observer_t observer;
    VESC_CurrentController_t current;

    VESC_FOC_State_t state;
    float state_time_s;
    float total_open_loop_time_s;
    float open_loop_phase;
    float open_loop_omega;
    float transition_offset;
    float id_ref_ramped;
    float iq_ref_ramped;
    uint32_t transition_valid_count;
    uint32_t observer_invalid_count;
    uint32_t observer_reverse_count;
    uint8_t was_enabled;
    uint8_t fault_code;

    /* Latched at the instant of a fault for post-stop debugging. */
    float fault_omega_e;
    float fault_flux_mag;
    float fault_phase_observer;
    float fault_phase_open_loop;
    uint8_t fault_observer_valid;
} VESC_SensorlessFOC_t;

void VESC_SensorlessFOC_Init(
    VESC_SensorlessFOC_t *foc,
    const VESC_SensorlessFOC_Config_t *config);

void VESC_SensorlessFOC_Reset(VESC_SensorlessFOC_t *foc);

void VESC_SensorlessFOC_Run(
    VESC_SensorlessFOC_t *foc,
    const VESC_SensorlessFOC_Input_t *input,
    VESC_SensorlessFOC_Output_t *output);

void VESC_ReconstructVoltageFromDuty(
    float duty_a, float duty_b, float duty_c, float vbus,
    float *v_alpha, float *v_beta);

#ifdef __cplusplus
}
#endif

#endif /* VESC_SENSORLESS_FOC_H */
