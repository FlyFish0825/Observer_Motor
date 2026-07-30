#ifndef VESC_SPEED_CONTROLLER_H
#define VESC_SPEED_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Optional slow speed PI. Run at a fixed 500 Hz to 2 kHz, not every PWM sample. */

typedef struct {
    float kp;                 /* A/rpm */
    float ki;                 /* A/(rpm*s) */
    float dt;                 /* s */
    float iq_limit_a;         /* absolute output limit */
    float speed_ramp_rpm_s;   /* command ramp */
    uint8_t allow_braking;
} VESC_SpeedControllerConfig_t;

typedef struct {
    float target_rpm_ramped;
    float error_rpm;
    float integrator_a;
    float iq_ref_a;
    uint8_t saturated;
} VESC_SpeedControllerState_t;

typedef struct {
    VESC_SpeedControllerConfig_t config;
    VESC_SpeedControllerState_t state;
} VESC_SpeedController_t;

void VESC_SpeedController_Init(
    VESC_SpeedController_t *controller,
    const VESC_SpeedControllerConfig_t *config);

void VESC_SpeedController_Reset(VESC_SpeedController_t *controller,
                                float measured_rpm);

float VESC_SpeedController_Run(VESC_SpeedController_t *controller,
                               float target_rpm,
                               float measured_rpm);

float VESC_ElectricalRadPerSec_ToMechanicalRpm(float omega_e,
                                               uint32_t pole_pairs);

#ifdef __cplusplus
}
#endif

#endif /* VESC_SPEED_CONTROLLER_H */
