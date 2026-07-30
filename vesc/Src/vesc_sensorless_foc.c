#include "vesc_sensorless_foc.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define VESC_ONE_BY_SQRT3_F (0.57735026918962576451f)
#define VESC_SQRT3_F        (1.73205080756887729353f)

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
        return (value > target) ? target : value;
    }
    if (value > target) {
        value -= step;
        return (value < target) ? target : value;
    }
    return value;
}

static void vesc_sincos(const VESC_SensorlessFOC_t *foc,
                        float angle, float *s, float *c) {
    if (foc->config.sincos_fn != NULL) {
        foc->config.sincos_fn(angle, s, c);
    } else {
        *s = sinf(angle);
        *c = cosf(angle);
    }
}


static void vesc_enter_state(VESC_SensorlessFOC_t *foc,
                             VESC_FOC_State_t state) {
    foc->state = state;
    foc->state_time_s = 0.0f;
}

static void vesc_latch_fault(VESC_SensorlessFOC_t *foc, uint8_t code) {
    foc->fault_code = code;
    foc->fault_omega_e = foc->observer.state.omega_e;
    foc->fault_flux_mag = foc->observer.state.psi_mag;
    foc->fault_phase_observer = foc->observer.state.phase_compensated;
    foc->fault_phase_open_loop = foc->open_loop_phase;
    foc->fault_observer_valid = foc->observer.state.valid;
    vesc_enter_state(foc, VESC_FOC_STATE_FAULT);
}

void VESC_ReconstructVoltageFromDuty(
    float duty_a, float duty_b, float duty_c, float vbus,
    float *v_alpha, float *v_beta) {
    if ((v_alpha == NULL) || (v_beta == NULL)) {
        return;
    }

    duty_a = vesc_clampf(duty_a, 0.0f, 1.0f);
    duty_b = vesc_clampf(duty_b, 0.0f, 1.0f);
    duty_c = vesc_clampf(duty_c, 0.0f, 1.0f);

    *v_alpha = vbus * (2.0f * duty_a - duty_b - duty_c) / 3.0f;
    *v_beta = vbus * (duty_b - duty_c) * VESC_ONE_BY_SQRT3_F;
}

void VESC_SensorlessFOC_Init(
    VESC_SensorlessFOC_t *foc,
    const VESC_SensorlessFOC_Config_t *config) {
    if ((foc == NULL) || (config == NULL)) {
        return;
    }

    memset(foc, 0, sizeof(*foc));
    foc->config = *config;

    /* Keep all three modules on exactly the same interrupt period. */
    foc->config.current.dt = foc->config.observer.dt;

    VESC_Observer_Init(&foc->observer, &foc->config.observer);
    VESC_CurrentController_Init(&foc->current, &foc->config.current);
    VESC_SensorlessFOC_Reset(foc);
}

void VESC_SensorlessFOC_Reset(VESC_SensorlessFOC_t *foc) {
    if (foc == NULL) {
        return;
    }

    foc->state = VESC_FOC_STATE_IDLE;
    foc->state_time_s = 0.0f;
    foc->total_open_loop_time_s = 0.0f;
    foc->open_loop_phase = 0.0f;
    foc->open_loop_omega = 0.0f;
    foc->transition_offset = 0.0f;
    foc->id_ref_ramped = 0.0f;
    foc->iq_ref_ramped = 0.0f;
    foc->transition_valid_count = 0U;
    foc->observer_invalid_count = 0U;
    foc->observer_reverse_count = 0U;
    foc->was_enabled = 0U;
    foc->fault_code = 0U;

    VESC_Observer_Reset(&foc->observer, 0.0f);
    VESC_CurrentController_Reset(&foc->current);
}

static uint8_t vesc_observer_ready(VESC_SensorlessFOC_t *foc) {
    const VESC_ObserverState_t *obs = &foc->observer.state;
    const VESC_SensorlessFOC_Config_t *cfg = &foc->config;

    const float open_abs = fabsf(foc->open_loop_omega);
    const float obs_abs = fabsf(obs->omega_e);
    if ((obs->valid == 0U) ||
        (obs_abs < cfg->transition_min_omega_rad_s) ||
        (open_abs < 1.0f)) {
        return 0U;
    }

    if ((obs->omega_e * foc->open_loop_omega) <= 0.0f) {
        return 0U;
    }

    const float ratio = obs_abs / open_abs;
    if ((ratio < cfg->transition_speed_ratio_min) ||
        (ratio > cfg->transition_speed_ratio_max)) {
        return 0U;
    }

    return 1U;
}

void VESC_SensorlessFOC_Run(
    VESC_SensorlessFOC_t *foc,
    const VESC_SensorlessFOC_Input_t *input,
    VESC_SensorlessFOC_Output_t *output) {
    if ((foc == NULL) || (input == NULL) || (output == NULL)) {
        return;
    }

    memset(output, 0, sizeof(*output));

    const float dt = foc->config.observer.dt;
    float measured_v_alpha = 0.0f;
    float measured_v_beta = 0.0f;
    VESC_ReconstructVoltageFromDuty(input->duty_a,
                                    input->duty_b,
                                    input->duty_c,
                                    input->vbus,
                                    &measured_v_alpha,
                                    &measured_v_beta);

    VESC_Observer_Run(&foc->observer,
                      measured_v_alpha,
                      measured_v_beta,
                      input->i_alpha,
                      input->i_beta,
                      input->vbus);

    if (input->enable == 0U) {
        if (foc->was_enabled != 0U) {
            VESC_SensorlessFOC_Reset(foc);
        }
        output->state = VESC_FOC_STATE_IDLE;
        output->phase_observer = foc->observer.state.phase_compensated;
        output->phase_pll = foc->observer.state.pll_phase;
        output->omega_e = foc->observer.state.omega_e;
        output->observer_flux_mag = foc->observer.state.psi_mag;
        output->observer_error = foc->observer.state.error;
        output->observer_duty_est = foc->observer.state.duty_est;
        output->observer_valid = foc->observer.state.valid;
        return;
    }

    if (foc->was_enabled == 0U) {
        foc->was_enabled = 1U;
        foc->open_loop_phase = 0.0f;
        foc->open_loop_omega = 0.0f;
        foc->total_open_loop_time_s = 0.0f;
        foc->transition_valid_count = 0U;
        VESC_Observer_Seed(&foc->observer, foc->open_loop_phase,
                           input->i_alpha, input->i_beta);
        VESC_CurrentController_Reset(&foc->current);
        vesc_enter_state(foc, VESC_FOC_STATE_ALIGN);
    }

    if ((foc->state == VESC_FOC_STATE_FAULT) ||
        (input->vbus <= 1.0f) || !isfinite(input->vbus)) {
        foc->fault_code = (input->vbus <= 1.0f) ? 1U : foc->fault_code;
        foc->state = VESC_FOC_STATE_FAULT;
        output->state = foc->state;
        output->fault_code = foc->fault_code;
        return;
    }

    foc->state_time_s += dt;

    float phase_control = foc->open_loop_phase;
    float id_target = 0.0f;
    float iq_target = 0.0f;
    float active_voltage_limit_v = 0.0f;

    switch (foc->state) {
    case VESC_FOC_STATE_IDLE:
        vesc_enter_state(foc, VESC_FOC_STATE_ALIGN);
        break;

    case VESC_FOC_STATE_ALIGN:
        phase_control = foc->open_loop_phase;
        id_target = foc->config.align_current_a;
        iq_target = 0.0f;
        active_voltage_limit_v = foc->config.align_max_voltage_v;

        if (foc->state_time_s >= foc->config.align_time_s) {
            VESC_Observer_Seed(&foc->observer, foc->open_loop_phase,
                               input->i_alpha, input->i_beta);
            vesc_enter_state(foc, VESC_FOC_STATE_OPEN_LOOP_RAMP);
        }
        break;

    case VESC_FOC_STATE_OPEN_LOOP_RAMP: {
        foc->total_open_loop_time_s += dt;
        float progress = 1.0f;
        if (foc->config.open_loop_ramp_time_s > 0.0f) {
            progress = vesc_clampf(
                foc->state_time_s / foc->config.open_loop_ramp_time_s,
                0.0f, 1.0f);
        }

        foc->open_loop_omega =
            foc->config.open_loop_speed_start_rad_s +
            progress * (foc->config.open_loop_speed_end_rad_s -
                        foc->config.open_loop_speed_start_rad_s);
        foc->open_loop_phase = VESC_WrapPi(
            foc->open_loop_phase + foc->open_loop_omega * dt);

        phase_control = foc->open_loop_phase;
        id_target = 0.0f;
        iq_target = foc->config.startup_iq_a;
        active_voltage_limit_v = foc->config.startup_max_voltage_v;

        if (progress >= 1.0f) {
            foc->transition_valid_count = 0U;
            vesc_enter_state(foc, VESC_FOC_STATE_OPEN_LOOP_HOLD);
        }
        break;
    }

    case VESC_FOC_STATE_OPEN_LOOP_HOLD:
        foc->total_open_loop_time_s += dt;
        foc->open_loop_omega = foc->config.open_loop_speed_end_rad_s;
        foc->open_loop_phase = VESC_WrapPi(
            foc->open_loop_phase + foc->open_loop_omega * dt);
        phase_control = foc->open_loop_phase;
        id_target = 0.0f;
        iq_target = foc->config.startup_iq_a;
        active_voltage_limit_v = foc->config.startup_max_voltage_v;

        if (vesc_observer_ready(foc) != 0U) {
            if (foc->transition_valid_count < UINT32_MAX) {
                foc->transition_valid_count++;
            }
        } else {
            foc->transition_valid_count = 0U;
        }

        if ((foc->state_time_s >= foc->config.open_loop_hold_time_s) &&
            (foc->transition_valid_count >=
             foc->config.transition_valid_samples)) {
            foc->transition_offset = VESC_AngleDifference(
                foc->open_loop_phase,
                foc->observer.state.phase_compensated);
            foc->observer_invalid_count = 0U;
            foc->observer_reverse_count = 0U;
            vesc_enter_state(foc, VESC_FOC_STATE_TRANSITION);
        } else if ((foc->config.open_loop_timeout_s > 0.0f) &&
                   (foc->total_open_loop_time_s >=
                    foc->config.open_loop_timeout_s)) {
            foc->fault_code = 2U; /* observer did not lock */
            vesc_enter_state(foc, VESC_FOC_STATE_FAULT);
        }
        break;

    case VESC_FOC_STATE_TRANSITION: {
        float progress = 1.0f;
        if (foc->config.transition_time_s > 0.0f) {
            progress = vesc_clampf(
                foc->state_time_s / foc->config.transition_time_s,
                0.0f, 1.0f);
        }

        phase_control = VESC_WrapPi(
    foc->observer.state.phase_compensated +
    foc->transition_offset);
        id_target = 0.0f;
        iq_target = foc->config.startup_iq_a;
        active_voltage_limit_v = foc->config.startup_max_voltage_v;

        /*
         * Do not trip on one noisy observer sample. The observer had already
         * been valid for transition_valid_samples before entering this state.
         * Split the old fault 3 into two diagnosable causes:
         *   31: flux/observer validity lost continuously
         *   32: estimated electrical speed reversed continuously
         */
        if (foc->observer.state.valid == 0U) {
            if (foc->observer_invalid_count < UINT32_MAX) {
                foc->observer_invalid_count++;
            }
        } else {
            foc->observer_invalid_count = 0U;
        }

        const uint8_t speed_reversed = (uint8_t)(
            (fabsf(foc->observer.state.omega_e) > 20.0f) &&
            ((foc->observer.state.omega_e * foc->open_loop_omega) <= 0.0f));

        if (speed_reversed != 0U) {
            if (foc->observer_reverse_count < UINT32_MAX) {
                foc->observer_reverse_count++;
            }
        } else {
            foc->observer_reverse_count = 0U;
        }

        if ((foc->config.transition_loss_samples > 0U) &&
            (foc->observer_invalid_count >=
             foc->config.transition_loss_samples)) {
            vesc_latch_fault(foc, 31U);
        } else if ((foc->config.transition_loss_samples > 0U) &&
                   (foc->observer_reverse_count >=
                    foc->config.transition_loss_samples)) {
            vesc_latch_fault(foc, 32U);
        } else if (progress >= 1.0f) {
            foc->observer_invalid_count = 0U;
            foc->observer_reverse_count = 0U;
            vesc_enter_state(foc, VESC_FOC_STATE_CLOSED_LOOP);
        }
        break;
    }

    case VESC_FOC_STATE_CLOSED_LOOP:

    phase_control = VESC_WrapPi(
        foc->observer.state.phase_compensated +
        foc->transition_offset);

    id_target = 0.0f;
    iq_target = input->iq_ref_a;

    active_voltage_limit_v = 0.0f;

    if (foc->observer.state.valid == 0U) {
        if (foc->observer_invalid_count < UINT32_MAX) {
            foc->observer_invalid_count++;
        }
    } else {
        foc->observer_invalid_count = 0U;
    }

    if ((foc->config.closed_loop_loss_samples > 0U) &&
        (foc->observer_invalid_count >=
         foc->config.closed_loop_loss_samples)) {

        vesc_latch_fault(foc, 4U);
    }

    break;
    case VESC_FOC_STATE_FAULT:
    default:
        output->state = foc->state;
        output->fault_code = foc->fault_code;
        output->omega_e = foc->fault_omega_e;
        output->observer_flux_mag = foc->fault_flux_mag;
        output->phase_observer = foc->fault_phase_observer;
        output->phase_open_loop = foc->fault_phase_open_loop;
        output->observer_valid = foc->fault_observer_valid;
        return;
    }

    id_target = vesc_clampf(id_target,
                            -foc->config.max_current_a,
                            foc->config.max_current_a);
    iq_target = vesc_clampf(iq_target,
                            -foc->config.max_current_a,
                            foc->config.max_current_a);

    foc->id_ref_ramped = vesc_move_towards(
        foc->id_ref_ramped,
        id_target,
        foc->config.align_current_ramp_a_per_s * dt);

    foc->iq_ref_ramped = vesc_move_towards(
        foc->iq_ref_ramped,
        iq_target,
        foc->config.iq_ramp_a_per_s * dt);

    /*
     * First-run safety clamp. During ALIGN and open-loop startup, the PI is
     * not allowed to use the full DC-bus voltage. This limit is converted to
     * an equivalent SVPWM duty limit before running the controller, so the
     * integrators are clamped too and cannot wind up behind an output clamp.
     */
    float active_max_duty = foc->config.current.max_duty;
    if ((active_voltage_limit_v > 0.0f) && (input->vbus > 0.5f)) {
        const float duty_from_voltage =
            active_voltage_limit_v * VESC_SQRT3_F / input->vbus;
        if (duty_from_voltage < active_max_duty) {
            active_max_duty = duty_from_voltage;
        }
    }
    foc->current.config.max_duty =
        vesc_clampf(active_max_duty, 0.0f, 0.98f);

    float sin_phase = 0.0f;
    float cos_phase = 1.0f;
    vesc_sincos(foc, phase_control, &sin_phase, &cos_phase);

    /* Park and inverse Park use the exact same phase on every sample. */
    const float id = cos_phase * input->i_alpha +
                     sin_phase * input->i_beta;
    const float iq = cos_phase * input->i_beta -
                     sin_phase * input->i_alpha;

    VESC_CurrentController_Run(&foc->current,
                               foc->id_ref_ramped,
                               foc->iq_ref_ramped,
                               id,
                               iq,
                               foc->observer.state.omega_e,
                               input->vbus);

    const float ud = foc->current.state.vd;
    const float uq = foc->current.state.vq;
    const float u_alpha = cos_phase * ud - sin_phase * uq;
    const float u_beta = sin_phase * ud + cos_phase * uq;

    output->state = foc->state;
    output->phase_control = phase_control;
    output->phase_observer = foc->observer.state.phase_compensated;
    output->phase_pll = foc->observer.state.pll_phase;
    output->phase_open_loop = foc->open_loop_phase;
    output->transition_offset = foc->transition_offset;
    output->omega_e = foc->observer.state.omega_e;
    output->id_ref = foc->id_ref_ramped;
    output->iq_ref = foc->iq_ref_ramped;
    output->id = id;
    output->iq = iq;
    output->ud = ud;
    output->uq = uq;
    output->u_alpha = u_alpha;
    output->u_beta = u_beta;
    output->observer_flux_mag = foc->observer.state.psi_mag;
    output->observer_error = foc->observer.state.error;
    output->observer_duty_est = foc->observer.state.duty_est;
    output->observer_valid = foc->observer.state.valid;
    output->voltage_saturated = foc->current.state.saturated;
    output->fault_code = foc->fault_code;

}
