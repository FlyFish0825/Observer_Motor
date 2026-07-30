#include "vesc_current_controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define VESC_ONE_BY_SQRT3_F (0.57735026918962576451f)

static float vesc_clampf(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float vesc_clamp_abs(float value, float limit) {
    if (limit < 0.0f) {
        limit = -limit;
    }
    return vesc_clampf(value, -limit, limit);
}

void VESC_CurrentController_Init(
    VESC_CurrentController_t *controller,
    const VESC_CurrentControllerConfig_t *config) {
    if ((controller == NULL) || (config == NULL)) {
        return;
    }

    memset(controller, 0, sizeof(*controller));
    controller->config = *config;

    controller->config.max_duty =
        vesc_clampf(controller->config.max_duty, 0.0f, 0.98f);
    controller->config.d_axis_priority =
        vesc_clampf(controller->config.d_axis_priority, 0.0f, 1.0f);
}

void VESC_CurrentController_Reset(VESC_CurrentController_t *controller) {
    if (controller == NULL) {
        return;
    }
    memset(&controller->state, 0, sizeof(controller->state));
}

void VESC_CurrentController_SetBandwidth(
    VESC_CurrentController_t *controller,
    float bandwidth_rad_s) {
    if ((controller == NULL) || (bandwidth_rad_s <= 0.0f)) {
        return;
    }

    /* Pole-zero cancellation for plant 1/(L*s + R): Kp=L*w, Ki=R*w. */
    controller->config.kp = controller->config.motor_lq * bandwidth_rad_s;
    controller->config.ki = controller->config.motor_r * bandwidth_rad_s;
}

void VESC_CurrentController_Preload(
    VESC_CurrentController_t *controller,
    float desired_vd, float desired_vq,
    float id_ref, float iq_ref,
    float id, float iq,
    float omega_e) {
    if (controller == NULL) {
        return;
    }

    const VESC_CurrentControllerConfig_t *cfg = &controller->config;
    VESC_CurrentControllerState_t *st = &controller->state;
    const float error_d = id_ref - id;
    const float error_q = iq_ref - iq;

    float dec_vd = 0.0f;
    float dec_vq = 0.0f;
    if (cfg->decoupling_enable != 0U) {
        dec_vd = omega_e * cfg->motor_lq * iq;
        dec_vq = omega_e * cfg->motor_ld * id +
                 omega_e * cfg->flux_linkage;
    }

    st->vd_integrator = desired_vd - cfg->kp * error_d + dec_vd;
    st->vq_integrator = desired_vq - cfg->kp * error_q - dec_vq;
}

void VESC_CurrentController_Run(
    VESC_CurrentController_t *controller,
    float id_ref, float iq_ref,
    float id, float iq,
    float omega_e,
    float vbus) {
    if (controller == NULL) {
        return;
    }

    const VESC_CurrentControllerConfig_t *cfg = &controller->config;
    VESC_CurrentControllerState_t *st = &controller->state;

    if ((cfg->dt <= 0.0f) || (vbus <= 0.5f)) {
        st->vd = 0.0f;
        st->vq = 0.0f;
        st->saturated = 1U;
        return;
    }

    st->error_d = id_ref - id;
    st->error_q = iq_ref - iq;

    st->vd_integrator += st->error_d * cfg->ki * cfg->dt;
    st->vq_integrator += st->error_q * cfg->ki * cfg->dt;

    st->vd = st->vd_integrator + cfg->kp * st->error_d;
    st->vq = st->vq_integrator + cfg->kp * st->error_q;

    if (cfg->decoupling_enable != 0U) {
        /* PMSM cross-coupling and back-EMF feed-forward, same signs as VESC. */
        st->vd -= omega_e * cfg->motor_lq * iq;
        st->vq += omega_e * cfg->motor_ld * id +
                  omega_e * cfg->flux_linkage;
    }

    st->max_voltage = VESC_ONE_BY_SQRT3_F * cfg->max_duty * vbus;
    const float max_vd = st->max_voltage * cfg->d_axis_priority;

    const float vd_before = st->vd;
    const float vq_before = st->vq;

    st->vd = vesc_clamp_abs(st->vd, max_vd);
    st->vd_integrator = vesc_clamp_abs(st->vd_integrator, max_vd);

    float vq_sq = st->max_voltage * st->max_voltage - st->vd * st->vd;
    if (vq_sq < 0.0f) {
        vq_sq = 0.0f;
    }
    st->max_vq = sqrtf(vq_sq);

    st->vq = vesc_clamp_abs(st->vq, st->max_vq);
    st->vq_integrator = vesc_clamp_abs(st->vq_integrator, st->max_vq);

    st->saturated = (uint8_t)((fabsf(st->vd - vd_before) > 1.0e-6f) ||
                              (fabsf(st->vq - vq_before) > 1.0e-6f));

    if (!isfinite(st->vd) || !isfinite(st->vq) ||
        !isfinite(st->vd_integrator) || !isfinite(st->vq_integrator)) {
        VESC_CurrentController_Reset(controller);
        st->saturated = 1U;
    }
}
