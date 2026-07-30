#ifndef VESC_PROJECT_CONFIG_H
#define VESC_PROJECT_CONFIG_H

#include "vesc_sensorless_foc.h"

/*
 * Moderate-start values for the user's motor/project:
 * R = 2.55 ohm, L = 0.86 mH, lambda = 0.0035 Wb, ISR = 25 kHz.
 * Adjust startup current and direction before raising any limits.
 */

static inline VESC_SensorlessFOC_Config_t VESC_Project_DefaultConfig(void) {
    VESC_SensorlessFOC_Config_t cfg = {0};

    cfg.observer.motor_r = 2.55f;
    cfg.observer.motor_l = 0.86e-3f;
    cfg.observer.flux_linkage = 0.0035f;
    cfg.observer.observer_gain = 1.0e8f;
    cfg.observer.observer_gain_slow = 0.05f;
    cfg.observer.pll_kp = 444.0f;
    cfg.observer.pll_ki = 98700.0f;
    cfg.observer.dt = 40.0e-6f;
    cfg.observer.phase_advance_cycles = 0.0f;
    cfg.observer.atan2_fn = 0;

    cfg.current.motor_r = cfg.observer.motor_r;
    cfg.current.motor_ld = cfg.observer.motor_l;
    cfg.current.motor_lq = cfg.observer.motor_l;
    cfg.current.flux_linkage = cfg.observer.flux_linkage;
    cfg.current.kp = 0.430f;      /* L * 500 rad/s: moderate current-loop bandwidth */
    cfg.current.ki = 1275.0f;      /* R * 500 rad/s */
    cfg.current.dt = cfg.observer.dt;
    cfg.current.max_duty = 0.5f; /* global ceiling; startup has a lower voltage clamp */
    cfg.current.d_axis_priority = 0.5f;
    cfg.current.decoupling_enable = 0U; /* enable only after basic loop works */

    cfg.align_time_s = 0.45f;
    cfg.align_current_a = 0.10f;
    cfg.align_current_ramp_a_per_s = 0.50f;
    cfg.align_max_voltage_v = 0.80f;

    cfg.open_loop_ramp_time_s = 2.00f;
    cfg.open_loop_hold_time_s = 0.20f;
    cfg.open_loop_timeout_s = 3.00f;
    cfg.open_loop_speed_start_rad_s = 5.0f;
    cfg.open_loop_speed_end_rad_s = 180.0f;
    cfg.startup_iq_a = 0.15f;
    cfg.startup_max_voltage_v = 3.00f;

    cfg.transition_time_s = 0.30f;
    cfg.transition_min_omega_rad_s = 70.0f;
    cfg.transition_speed_ratio_min = 0.45f;
    cfg.transition_speed_ratio_max = 1.80f;
    cfg.transition_valid_samples = 500U; /* 20 ms at 25 kHz */
    cfg.transition_loss_samples = 125U; /* 5 ms debounce during handover */
    cfg.closed_loop_loss_samples = 250U; /* 10 ms debounce in closed loop */

    cfg.iq_ramp_a_per_s = 0.75f;
    cfg.max_current_a = 0.50f;
    cfg.sincos_fn = 0;

    return cfg;
}

#endif /* VESC_PROJECT_CONFIG_H */
