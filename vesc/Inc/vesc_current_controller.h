#ifndef VESC_CURRENT_CONTROLLER_H
#define VESC_CURRENT_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Minimal VESC-style d/q current PI controller with decoupling and anti-windup. */

typedef struct {
    float motor_r;          /* ohm */
    float motor_ld;         /* H */
    float motor_lq;         /* H */
    float flux_linkage;     /* Wb */
    float kp;               /* V/A */
    float ki;               /* V/(A*s) */
    float dt;               /* s */
    float max_duty;         /* 0...1 */
    float d_axis_priority;  /* 0...1, normally 1 */
    uint8_t decoupling_enable;
} VESC_CurrentControllerConfig_t;

typedef struct {
    float vd_integrator;
    float vq_integrator;

    float error_d;
    float error_q;
    float vd;
    float vq;
    float max_voltage;
    float max_vq;
    uint8_t saturated;
} VESC_CurrentControllerState_t;

typedef struct {
    VESC_CurrentControllerConfig_t config;
    VESC_CurrentControllerState_t state;
} VESC_CurrentController_t;

void VESC_CurrentController_Init(
    VESC_CurrentController_t *controller,
    const VESC_CurrentControllerConfig_t *config);

void VESC_CurrentController_Reset(VESC_CurrentController_t *controller);

void VESC_CurrentController_SetBandwidth(
    VESC_CurrentController_t *controller,
    float bandwidth_rad_s);

void VESC_CurrentController_Preload(
    VESC_CurrentController_t *controller,
    float desired_vd, float desired_vq,
    float id_ref, float iq_ref,
    float id, float iq,
    float omega_e);

void VESC_CurrentController_Run(
    VESC_CurrentController_t *controller,
    float id_ref, float iq_ref,
    float id, float iq,
    float omega_e,
    float vbus);

#ifdef __cplusplus
}
#endif

#endif /* VESC_CURRENT_CONTROLLER_H */
