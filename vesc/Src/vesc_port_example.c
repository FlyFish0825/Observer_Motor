/*
 * Copy the relevant fragments into main.c. This file is an integration example,
 * not a second main.c and should not be compiled unchanged.
 */

#include "vesc_project_config.h"
#include "vesc_sensorless_foc.h"

/* Existing project headers. */
#include "cordic.h"
#include "foc_math.h"

static VESC_SensorlessFOC_t g_vesc_foc;
static VESC_SensorlessFOC_Output_t g_vesc_out;
static uint8_t g_motor_enable = 0U;
static float g_iq_command_a = 0.30f;

static float VESC_Port_Atan2(float y, float x) {
    return FOC_Atan2_Fast(y, x);
}

static void VESC_Port_SinCos(float angle_rad, float *s, float *c) {
    const uint32_t angle_q31 = (uint32_t)CORDIC_RadToQ31(angle_rad);
    CORDIC_SinCos_FastF32(angle_q31, s, c);
}

static void VESC_Port_Init(void) {
    VESC_SensorlessFOC_Config_t cfg = VESC_Project_DefaultConfig();
    cfg.observer.atan2_fn = VESC_Port_Atan2;
    cfg.sincos_fn = VESC_Port_SinCos;
    VESC_SensorlessFOC_Init(&g_vesc_foc, &cfg);
}

/*
 * Call this after FOC_Get_Iabc() and FOC_Clarke() inside the ADC1 injected ISR.
 * duty_a/b/c must still be the duty ratios applied during the previous PWM cycle.
 */
static void VESC_Port_RunFromAdcIsr(void) {
    VESC_SensorlessFOC_Input_t in = {
        .enable = g_motor_enable,
        .iq_ref_a = g_iq_command_a,
        .i_alpha = foc.state.i_alpha_beta.alpha,
        .i_beta = foc.state.i_alpha_beta.beta,
        .duty_a = foc.svpwm.duty_a,
        .duty_b = foc.svpwm.duty_b,
        .duty_c = foc.svpwm.duty_c,
        .vbus = foc.state.vbus,
    };

    VESC_SensorlessFOC_Run(&g_vesc_foc, &in, &g_vesc_out);

    if ((g_vesc_out.state == VESC_FOC_STATE_IDLE) ||
        (g_vesc_out.state == VESC_FOC_STATE_FAULT)) {
        /* Stop PWM here or write 50% duty according to the power-stage policy. */
        return;
    }

    /* Keep your existing data structure and SVPWM implementation. */
    foc.state.i_dq.d = g_vesc_out.id;
    foc.state.i_dq.q = g_vesc_out.iq;
    foc.state.u_dq.d = g_vesc_out.ud;
    foc.state.u_dq.q = g_vesc_out.uq;
    foc.state.u_alpha_beta.alpha = g_vesc_out.u_alpha;
    foc.state.u_alpha_beta.beta = g_vesc_out.u_beta;

    FOC_InvClarke(&foc.state.u_alpha_beta, &foc.state.u_abc);
    FOC_SVPWM_Run(&foc.state.u_abc,
                  foc.state.vbus,
                  &foc.timer,
                  &foc.svpwm);

    TIM1->CCR1 = foc.svpwm.ccr_a;
    TIM1->CCR2 = foc.svpwm.ccr_b;
    TIM1->CCR3 = foc.svpwm.ccr_c;
}
