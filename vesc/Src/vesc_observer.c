#include "vesc_observer.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define VESC_PI_F          (3.14159265358979323846f)
#define VESC_TWO_PI_F      (6.28318530717958647692f)
#define VESC_SQRT3_F       (1.73205080756887729353f)
#define VESC_EPSILON_F     (1.0e-12f)

static float vesc_clampf(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float vesc_atan2(const VESC_Observer_t *observer, float y, float x) {
    if (observer->config.atan2_fn != NULL) {
        return observer->config.atan2_fn(y, x);
    }
    return atan2f(y, x);
}

float VESC_WrapPi(float angle_rad) {
    while (angle_rad > VESC_PI_F) {
        angle_rad -= VESC_TWO_PI_F;
    }
    while (angle_rad < -VESC_PI_F) {
        angle_rad += VESC_TWO_PI_F;
    }
    return angle_rad;
}

float VESC_AngleDifference(float target_rad, float actual_rad) {
    return VESC_WrapPi(target_rad - actual_rad);
}

float VESC_Observer_GainFromBandwidth(float flux_linkage,
                                      float observer_bandwidth_rad_s) {
    const float lambda_sq = flux_linkage * flux_linkage;
    if ((lambda_sq < VESC_EPSILON_F) ||
        (observer_bandwidth_rad_s <= 0.0f)) {
        return 0.0f;
    }

    /* Ortega correction has an approximate rate gamma*lambda^2/2. */
    return (2.0f * observer_bandwidth_rad_s) / lambda_sq;
}

void VESC_Observer_Init(VESC_Observer_t *observer,
                        const VESC_ObserverConfig_t *config) {
    if ((observer == NULL) || (config == NULL)) {
        return;
    }

    memset(observer, 0, sizeof(*observer));
    observer->config = *config;

    if (observer->config.observer_gain_slow <= 0.0f) {
        observer->config.observer_gain_slow = 0.05f;
    }
    observer->config.observer_gain_slow =
        vesc_clampf(observer->config.observer_gain_slow, 0.001f, 1.0f);

    VESC_Observer_Reset(observer, 0.0f);
}

void VESC_Observer_Reset(VESC_Observer_t *observer, float initial_phase_rad) {
    if (observer == NULL) {
        return;
    }

    const VESC_ObserverConfig_t config = observer->config;
    memset(&observer->state, 0, sizeof(observer->state));
    observer->config = config;

    observer->state.x1 = config.flux_linkage * cosf(initial_phase_rad);
    observer->state.x2 = config.flux_linkage * sinf(initial_phase_rad);
    observer->state.psi_alpha = observer->state.x1;
    observer->state.psi_beta = observer->state.x2;
    observer->state.psi_mag = fabsf(config.flux_linkage);
    observer->state.phase_raw = VESC_WrapPi(initial_phase_rad);
    observer->state.phase_compensated = observer->state.phase_raw;
    observer->state.pll_phase = observer->state.phase_raw;
}

void VESC_Observer_Seed(VESC_Observer_t *observer, float phase_rad,
                        float i_alpha, float i_beta) {
    if (observer == NULL) {
        return;
    }

    const float lambda = observer->config.flux_linkage;
    const float inductance = observer->config.motor_l;

    /* x = psi_pm + L*i, because phase is atan2(x2-L*i_beta, x1-L*i_alpha). */
    observer->state.x1 = lambda * cosf(phase_rad) + inductance * i_alpha;
    observer->state.x2 = lambda * sinf(phase_rad) + inductance * i_beta;
    observer->state.phase_raw = VESC_WrapPi(phase_rad);
    observer->state.phase_compensated = observer->state.phase_raw;
    observer->state.pll_phase = observer->state.phase_raw;
    observer->state.omega_e = 0.0f;
}

void VESC_Observer_Run(VESC_Observer_t *observer,
                       float v_alpha, float v_beta,
                       float i_alpha, float i_beta,
                       float vbus) {
    if (observer == NULL) {
        return;
    }

    VESC_ObserverConfig_t *cfg = &observer->config;
    VESC_ObserverState_t *st = &observer->state;

    if ((cfg->dt <= 0.0f) || (cfg->motor_l <= 0.0f) ||
        (cfg->flux_linkage <= 0.0f) || (cfg->motor_r < 0.0f)) {
        st->valid = 0U;
        return;
    }

    const float l_ia = cfg->motor_l * i_alpha;
    const float l_ib = cfg->motor_l * i_beta;
    const float r_ia = cfg->motor_r * i_alpha;
    const float r_ib = cfg->motor_r * i_beta;
    const float lambda_sq = cfg->flux_linkage * cfg->flux_linkage;

    const float v_mag = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    if (vbus > 0.5f) {
        st->duty_est = v_mag * VESC_SQRT3_F / vbus;
    } else {
        st->duty_est = 0.0f;
    }

    /*
     * VESC scales the observer gain with duty and bus voltage, reaching full
     * configured gain around 40 V of effective modulation, then multiplies by 4.
     */
    float full_gain_duty = 1.0f;
    if (vbus > 0.5f) {
        full_gain_duty = 40.0f / vbus;
    }
    if (full_gain_duty < 0.01f) {
        full_gain_duty = 0.01f;
    }

    float gain_scale = st->duty_est / full_gain_duty;
    gain_scale = vesc_clampf(gain_scale,
                             cfg->observer_gain_slow,
                             1.0f);
    st->gamma_now = cfg->observer_gain * gain_scale * 4.0f;

    const float psi_alpha_before = st->x1 - l_ia;
    const float psi_beta_before = st->x2 - l_ib;
    float error = lambda_sq -
                  (psi_alpha_before * psi_alpha_before +
                   psi_beta_before * psi_beta_before);

    /* This one-sided correction is used by the original VESC Ortega observer. */
    if (error > 0.0f) {
        error = 0.0f;
    }
    st->error = error;

    const float gamma_half = 0.5f * st->gamma_now;
    const float x1_dot = v_alpha - r_ia +
                         gamma_half * psi_alpha_before * error;
    const float x2_dot = v_beta - r_ib +
                         gamma_half * psi_beta_before * error;

    st->x1 += x1_dot * cfg->dt;
    st->x2 += x2_dot * cfg->dt;

    if (!isfinite(st->x1) || !isfinite(st->x2)) {
        VESC_Observer_Reset(observer, st->pll_phase);
        st->valid = 0U;
        return;
    }

    /* Same low-magnitude assistance used by VESC before angle extraction. */
    const float x_mag = sqrtf(st->x1 * st->x1 + st->x2 * st->x2);
    if (x_mag < (0.5f * cfg->flux_linkage)) {
        st->x1 *= 1.1f;
        st->x2 *= 1.1f;
    }

    st->psi_alpha = st->x1 - l_ia;
    st->psi_beta = st->x2 - l_ib;
    st->psi_mag = sqrtf(st->psi_alpha * st->psi_alpha +
                        st->psi_beta * st->psi_beta);

    /* Hold the previous angle only when the flux vector is almost zero. */
    if (st->psi_mag >= (0.02f * cfg->flux_linkage)) {
        st->phase_raw = VESC_WrapPi(vesc_atan2(observer,
                                               st->psi_beta,
                                               st->psi_alpha));
    }

    /* VESC PLL: phase error drives both phase correction and speed integrator. */
    st->pll_error = VESC_AngleDifference(st->phase_raw, st->pll_phase);
    st->pll_phase +=
        (st->omega_e + cfg->pll_kp * st->pll_error) * cfg->dt;
    st->pll_phase = VESC_WrapPi(st->pll_phase);
    st->omega_e += cfg->pll_ki * st->pll_error * cfg->dt;

    if (!isfinite(st->omega_e)) {
        st->omega_e = 0.0f;
    }

    st->phase_compensated = VESC_WrapPi(
        st->phase_raw +
        st->omega_e * cfg->dt * (0.5f + cfg->phase_advance_cycles));

    const float flux_ratio = st->psi_mag / cfg->flux_linkage;
    st->valid = (uint8_t)(isfinite(st->phase_compensated) &&
                          isfinite(st->omega_e) &&
                          (flux_ratio > 0.20f) &&
                          (flux_ratio < 3.0f));
}
