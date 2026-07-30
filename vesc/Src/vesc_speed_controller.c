#include "vesc_speed_controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define VESC_RAD_S_TO_RPM_F (9.54929658551372014613f)

static float vesc_clampf(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float vesc_move_towards(float value, float target, float step) {
    if (step <= 0.0f) {
        return target;
    }
    if (value < target) {
        value += step;
        return value > target ? target : value;
    }
    if (value > target) {
        value -= step;
        return value < target ? target : value;
    }
    return value;
}

void VESC_SpeedController_Init(
    VESC_SpeedController_t *controller,
    const VESC_SpeedControllerConfig_t *config) {
    if ((controller == NULL) || (config == NULL)) {
        return;
    }
    memset(controller, 0, sizeof(*controller));
    controller->config = *config;
    controller->config.iq_limit_a = fabsf(controller->config.iq_limit_a);
}

void VESC_SpeedController_Reset(VESC_SpeedController_t *controller,
                                float measured_rpm) {
    if (controller == NULL) {
        return;
    }
    memset(&controller->state, 0, sizeof(controller->state));
    controller->state.target_rpm_ramped = measured_rpm;
}

float VESC_SpeedController_Run(VESC_SpeedController_t *controller,
                               float target_rpm,
                               float measured_rpm) {
    if ((controller == NULL) || (controller->config.dt <= 0.0f)) {
        return 0.0f;
    }

    const VESC_SpeedControllerConfig_t *cfg = &controller->config;
    VESC_SpeedControllerState_t *st = &controller->state;

    st->target_rpm_ramped = vesc_move_towards(
        st->target_rpm_ramped,
        target_rpm,
        cfg->speed_ramp_rpm_s * cfg->dt);

    st->error_rpm = st->target_rpm_ramped - measured_rpm;
    const float p_term = cfg->kp * st->error_rpm;

    st->integrator_a += cfg->ki * st->error_rpm * cfg->dt;
    st->integrator_a = vesc_clampf(st->integrator_a,
                                  -cfg->iq_limit_a,
                                  cfg->iq_limit_a);

    const float unsaturated = p_term + st->integrator_a;
    st->iq_ref_a = vesc_clampf(unsaturated,
                               -cfg->iq_limit_a,
                               cfg->iq_limit_a);
    st->saturated = (uint8_t)(fabsf(st->iq_ref_a - unsaturated) > 1.0e-6f);

    /* Conditional integration anti-windup. */
    if ((st->saturated != 0U) &&
        ((st->iq_ref_a * st->error_rpm) > 0.0f)) {
        st->integrator_a -= cfg->ki * st->error_rpm * cfg->dt;
    }

    if (cfg->allow_braking == 0U) {
        if ((measured_rpm > 5.0f) && (st->iq_ref_a < 0.0f)) {
            st->iq_ref_a = 0.0f;
        } else if ((measured_rpm < -5.0f) && (st->iq_ref_a > 0.0f)) {
            st->iq_ref_a = 0.0f;
        }
    }

    return st->iq_ref_a;
}

float VESC_ElectricalRadPerSec_ToMechanicalRpm(float omega_e,
                                               uint32_t pole_pairs) {
    if (pole_pairs == 0U) {
        return 0.0f;
    }
    return omega_e * VESC_RAD_S_TO_RPM_F / (float)pole_pairs;
}
