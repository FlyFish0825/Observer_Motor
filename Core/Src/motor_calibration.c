#include "motor_calibration.h"
#include "debug_console.h"
#include "tim.h"

#include <math.h>
#include <string.h>

volatile MotorCalibration_t motor_cal;

typedef enum {
    MOTOR_AH = 1,
    MOTOR_AL,
    MOTOR_BH,
    MOTOR_BL,
    MOTOR_CH,
    MOTOR_CL
} Motor_Pin_t;

typedef enum {
    MOTOR_GPIO_LOW = 0,
    MOTOR_GPIO_HIGH,
    MOTOR_PWM
} Motor_Mode_t;

static void Motor_PinMode(Motor_Pin_t pin, Motor_Mode_t mode)
{
    GPIO_InitTypeDef gpio = {0};
    GPIO_TypeDef *port;
    uint16_t pin_num;
    uint32_t af;
    uint32_t ccer;

    switch (pin) {
    case MOTOR_AH:
        port = GPIOA; pin_num = GPIO_PIN_8;
        af = GPIO_AF6_TIM1; ccer = TIM_CCER_CC1E;
        break;
    case MOTOR_AL:
        port = GPIOB; pin_num = GPIO_PIN_13;
        af = GPIO_AF6_TIM1; ccer = TIM_CCER_CC1NE;
        break;
    case MOTOR_BH:
        port = GPIOA; pin_num = GPIO_PIN_9;
        af = GPIO_AF6_TIM1; ccer = TIM_CCER_CC2E;
        break;
    case MOTOR_BL:
        port = GPIOB; pin_num = GPIO_PIN_14;
        af = GPIO_AF6_TIM1; ccer = TIM_CCER_CC2NE;
        break;
    case MOTOR_CH:
        port = GPIOA; pin_num = GPIO_PIN_10;
        af = GPIO_AF6_TIM1; ccer = TIM_CCER_CC3E;
        break;
    case MOTOR_CL:
        port = GPIOB; pin_num = GPIO_PIN_15;
        af = GPIO_AF4_TIM1; ccer = TIM_CCER_CC3NE;
        break;
    default:
        return;
    }

    gpio.Pin = pin_num;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    if (mode == MOTOR_PWM) {
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Alternate = af;
        HAL_GPIO_Init(port, &gpio);
        TIM1->CCER |= ccer;
    } else {
        TIM1->CCER &= ~ccer;
        HAL_GPIO_WritePin(port, pin_num,
            (mode == MOTOR_GPIO_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        gpio.Mode = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(port, &gpio);
    }
}

/* A 桥臂互补 PWM，B 下管常导通，C 全关。实际测 A-B 回路。 */
static void Cal_ConfigPhaseA(void)
{
    __HAL_TIM_MOE_DISABLE(&htim1);

    Motor_PinMode(MOTOR_AH, MOTOR_PWM);
    Motor_PinMode(MOTOR_AL, MOTOR_GPIO_LOW);

    Motor_PinMode(MOTOR_BH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_BL, MOTOR_GPIO_HIGH);

    Motor_PinMode(MOTOR_CH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_CL, MOTOR_GPIO_LOW);
}

static void Cal_ConfigPhaseB(void)
{
    Motor_PinMode(MOTOR_AH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_AL, MOTOR_GPIO_LOW);

    Motor_PinMode(MOTOR_BH, MOTOR_PWM);
    Motor_PinMode(MOTOR_BL, MOTOR_PWM);

    Motor_PinMode(MOTOR_CH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_CL, MOTOR_GPIO_HIGH);
}

static void Cal_ConfigPhaseC(void)
{
    Motor_PinMode(MOTOR_AH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_AL, MOTOR_GPIO_HIGH);

    Motor_PinMode(MOTOR_BH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_BL, MOTOR_GPIO_LOW);

    Motor_PinMode(MOTOR_CH, MOTOR_PWM);
    Motor_PinMode(MOTOR_CL, MOTOR_PWM);
}

static void Cal_OutputOff(void)
{
    __HAL_TIM_MOE_DISABLE(&htim1);
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
}

static void Cal_AllOff(void)
{
    Cal_OutputOff();

    Motor_PinMode(MOTOR_AH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_AL, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_BH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_BL, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_CH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_CL, MOTOR_GPIO_LOW);
}

static void Cal_SetDuty(float duty)
{
    uint32_t ccr = (uint32_t)((float)TIM1->ARR * duty);

    if (ccr < 1U)
        ccr = 1U;
    if (ccr >= TIM1->ARR)
        ccr = TIM1->ARR - 1U;

    if (motor_cal.phase == CAL_PHASE_AB)
        TIM1->CCR1 = ccr;
    else if (motor_cal.phase == CAL_PHASE_BC)
        TIM1->CCR2 = ccr;
    else
        TIM1->CCR3 = ccr;
    __HAL_TIM_MOE_ENABLE(&htim1);
}

static float Cal_LineCurrent(float ia, float ib, float ic)
{
    if (motor_cal.phase == CAL_PHASE_AB)
        return 0.5f * (fabsf(ia) + fabsf(ib));

    if (motor_cal.phase == CAL_PHASE_BC)
        return 0.5f * (fabsf(ib) + fabsf(ic));

    return 0.5f * (fabsf(ic) + fabsf(ia));
}

/* 用最后几组点拟合 V = R*I + b，斜率就是 R_AB。 */
static float Cal_FitResistance(void)
{
    uint16_t end = motor_cal.point_count;
    uint16_t start = (end > CAL_FIT_POINTS) ? (end - CAL_FIT_POINTS) : 0U;
    uint16_t n = end - start;

    float si = 0.0f, sv = 0.0f;
    float sii = 0.0f, siv = 0.0f;

    if (n < 2U)
        return 0.0f;

    for (uint16_t k = start; k < end; k++) {
        float i = motor_cal.point[k].current;
        float v = motor_cal.point[k].voltage;

        si += i;
        sv += v;
        sii += i * i;
        siv += i * v;
    }

    float den = (float)n * sii - si * si;
    if (fabsf(den) < 1e-6f)
        return 0.0f;

    return ((float)n * siv - si * sv) / den;
}

void MotorCalibration_Start(void)
{
    Cal_AllOff();
    memset((void *)&motor_cal, 0, sizeof(motor_cal));

    motor_cal.phase = CAL_PHASE_AB;
    motor_cal.duty = CAL_DUTY_START;
    Cal_ConfigPhaseA();
    motor_cal.state = CAL_SET_DUTY;
}

void MotorCalibration_Stop(void)
{
    Cal_AllOff();
    motor_cal.state = CAL_IDLE;
}

void MotorCalibration_Run(float ia, float ib, float ic, float vbus)
{
    float current = Cal_LineCurrent(ia, ib, ic);

    /* 瞬时硬保护。 */
    if ((fabsf(ia) >= CAL_HARD_CURRENT) ||
        (fabsf(ib) >= CAL_HARD_CURRENT) ||
        (fabsf(ic) >= CAL_HARD_CURRENT)) {
        Cal_AllOff();
        motor_cal.state = CAL_ERROR;
        return;
    }

    switch (motor_cal.state) {
    case CAL_IDLE:
    case CAL_DONE:
    case CAL_ERROR:
        break;

    case CAL_SET_DUTY:
        motor_cal.count = 0U;
        motor_cal.current_sum = 0.0f;
        motor_cal.vbus_sum = 0.0f;
        Cal_SetDuty(motor_cal.duty);
        motor_cal.state = CAL_SETTLE;
        break;

    case CAL_SETTLE:
        if (++motor_cal.count >= CAL_SETTLE_COUNT) {
            motor_cal.count = 0U;
            motor_cal.current_sum = 0.0f;
            motor_cal.vbus_sum = 0.0f;
            motor_cal.state = CAL_SAMPLE;
        }
        break;

    case CAL_SAMPLE:
        motor_cal.current_sum += current;
        motor_cal.vbus_sum += vbus;

        if (++motor_cal.count >= CAL_SAMPLE_COUNT) {
            float avg_i = motor_cal.current_sum / (float)CAL_SAMPLE_COUNT;
            float avg_vbus = motor_cal.vbus_sum / (float)CAL_SAMPLE_COUNT;

            if (motor_cal.point_count >= CAL_MAX_POINTS) {
                Cal_OutputOff();
                if (motor_cal.phase == CAL_PHASE_AB) {
                    motor_cal.r_ab = Cal_FitResistance();
                    motor_cal.phase = CAL_PHASE_BC;
                    motor_cal.duty = CAL_DUTY_START;
                    motor_cal.point_count = 0U;
                    Cal_ConfigPhaseB();
                    motor_cal.state = CAL_SET_DUTY;
                } else if (motor_cal.phase == CAL_PHASE_BC) {
                    motor_cal.r_bc = Cal_FitResistance();
                    motor_cal.phase = CAL_PHASE_CA;
                    motor_cal.duty = CAL_DUTY_START;
                    motor_cal.point_count = 0U;
                    Cal_ConfigPhaseC();
                    motor_cal.state = CAL_SET_DUTY;
                } else {
                    motor_cal.r_ca = Cal_FitResistance();
                    motor_cal.r_a = (motor_cal.r_ab + motor_cal.r_ca - motor_cal.r_bc) * 0.5f;
                    motor_cal.r_b = (motor_cal.r_ab + motor_cal.r_bc - motor_cal.r_ca) * 0.5f;
                    motor_cal.r_c = (motor_cal.r_bc + motor_cal.r_ca - motor_cal.r_ab) * 0.5f;
                    motor_cal.rs = (motor_cal.r_a + motor_cal.r_b + motor_cal.r_c) / 3.0f;
                    motor_cal.state = CAL_DONE;
                }
                break;
            }

            volatile MotorCalPoint_t *p =
                &motor_cal.point[motor_cal.point_count++];

            p->duty = motor_cal.duty;
            p->voltage = motor_cal.duty * avg_vbus;
            p->current = avg_i;

            /* 到 5A，或 duty 到上限，停止并用最后几组点拟合。 */
            if ((avg_i >= CAL_TARGET_CURRENT) ||
                (motor_cal.duty + CAL_DUTY_STEP > CAL_DUTY_MAX)) {
                Cal_OutputOff();
                if (motor_cal.phase == CAL_PHASE_AB) {
                    motor_cal.r_ab = Cal_FitResistance();
                    motor_cal.phase = CAL_PHASE_BC;
                    motor_cal.duty = CAL_DUTY_START;
                    motor_cal.point_count = 0U;
                    Cal_ConfigPhaseB();
                    motor_cal.state = CAL_SET_DUTY;
                } else if (motor_cal.phase == CAL_PHASE_BC) {
                    motor_cal.r_bc = Cal_FitResistance();
                    motor_cal.phase = CAL_PHASE_CA;
                    motor_cal.duty = CAL_DUTY_START;
                    motor_cal.point_count = 0U;
                    Cal_ConfigPhaseC();
                    motor_cal.state = CAL_SET_DUTY;
                } else {
                    motor_cal.r_ca = Cal_FitResistance();
                    motor_cal.r_a = (motor_cal.r_ab + motor_cal.r_ca - motor_cal.r_bc) * 0.5f;
                    motor_cal.r_b = (motor_cal.r_ab + motor_cal.r_bc - motor_cal.r_ca) * 0.5f;
                    motor_cal.r_c = (motor_cal.r_bc + motor_cal.r_ca - motor_cal.r_ab) * 0.5f;
                    motor_cal.rs = (motor_cal.r_a + motor_cal.r_b + motor_cal.r_c) / 3.0f;
                    motor_cal.state = CAL_DONE;
                }
                break;
            }

            motor_cal.duty += CAL_DUTY_STEP;
            motor_cal.state = CAL_SET_DUTY;
        }
        break;

    default:
        Cal_AllOff();
        motor_cal.state = CAL_ERROR;
        break;
    }
}

void MotorCalibration_DebugProcess(void)
{
    static MotorCalState_t last_state = CAL_IDLE;

    if ((motor_cal.state == CAL_DONE) && (last_state != CAL_DONE)) {
        DebugConsole_Printf("\r\n===== Rs Calibration Result =====\r\n");

        for (uint16_t i = 0U; i < motor_cal.point_count; i++) {
            DebugConsole_Printf(
                "%02u: duty=%.2f%%  V=%.4fV  I=%.4fA\r\n",
                (unsigned)(i + 1U),
                motor_cal.point[i].duty * 100.0f,
                motor_cal.point[i].voltage,
                motor_cal.point[i].current);
        }

        DebugConsole_Printf("R_AB = %.6f ohm\r\n"
                           "R_BC = %.6f ohm\r\n"
                           "R_CA = %.6f ohm\r\n"
                           "R_A  = %.6f ohm\r\n"
                           "R_B  = %.6f ohm\r\n"
                           "R_C  = %.6f ohm\r\n"
                           "R_S  = %.6f ohm\r\n",
                           motor_cal.r_ab, motor_cal.r_bc, motor_cal.r_ca,
                           motor_cal.r_a, motor_cal.r_b, motor_cal.r_c,
                           motor_cal.rs);
    }

    if ((motor_cal.state == CAL_ERROR) && (last_state != CAL_ERROR)) {
        DebugConsole_Printf(
            "\r\nRs calibration ERROR: duty=%.2f%%\r\n",
            motor_cal.duty * 100.0f);
    }

    last_state = motor_cal.state;
}
