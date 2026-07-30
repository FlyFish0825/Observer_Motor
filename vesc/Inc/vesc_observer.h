#ifndef VESC_OBSERVER_H
#define VESC_OBSERVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Minimal MCU-independent adaptation of the VESC Ortega flux observer and PLL.
 * The observer equations and PLL structure follow the official VESC firmware.
 * Original VESC firmware: Copyright Benjamin Vedder, GPLv3.
 */

typedef float (*VESC_Atan2Fn_t)(float y, float x);

typedef struct {
    float motor_r;                 /* phase resistance, ohm */
    float motor_l;                 /* phase inductance, H */
    float flux_linkage;            /* permanent-magnet flux linkage, Wb */
    float observer_gain;           /* base Ortega gain */
    float observer_gain_slow;      /* minimum gain ratio, normally 0.02...0.10 */
    float pll_kp;                  /* PLL proportional gain */
    float pll_ki;                  /* PLL integral gain */
    float dt;                      /* observer period, s */
    float phase_advance_cycles;    /* extra PWM-cycle phase advance; start at 0 */
    VESC_Atan2Fn_t atan2_fn;       /* NULL selects atan2f */
} VESC_ObserverConfig_t;

typedef struct {
    float x1;
    float x2;

    float psi_alpha;
    float psi_beta;
    float psi_mag;
    float error;
    float gamma_now;
    float duty_est;

    float phase_raw;
    float phase_compensated;
    float pll_phase;
    float omega_e;
    float pll_error;

    uint8_t valid;
} VESC_ObserverState_t;

typedef struct {
    VESC_ObserverConfig_t config;
    VESC_ObserverState_t state;
} VESC_Observer_t;

void VESC_Observer_Init(VESC_Observer_t *observer,
                        const VESC_ObserverConfig_t *config);

void VESC_Observer_Reset(VESC_Observer_t *observer, float initial_phase_rad);

void VESC_Observer_Seed(VESC_Observer_t *observer, float phase_rad,
                        float i_alpha, float i_beta);

void VESC_Observer_Run(VESC_Observer_t *observer,
                       float v_alpha, float v_beta,
                       float i_alpha, float i_beta,
                       float vbus);

float VESC_WrapPi(float angle_rad);
float VESC_AngleDifference(float target_rad, float actual_rad);
float VESC_Observer_GainFromBandwidth(float flux_linkage,
                                      float observer_bandwidth_rad_s);

#ifdef __cplusplus
}
#endif

#endif /* VESC_OBSERVER_H */
