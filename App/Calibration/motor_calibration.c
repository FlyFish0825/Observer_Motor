#include "motor_calibration.h"
#include "debug_console.h"
#include "foc_math.h"
#include "tim.h"

#include "arm_math.h"
#include <string.h>

volatile MotorCalibration_t motor_cal;

/* ADC 中断只写数值记录，主循环负责格式化；32 项环形队列。 */
#define CAL_LOG_SIZE 32U
typedef struct {
    MotorCalPoint_t point;
    float resistance;
    uint8_t phase;
    uint8_t index;
    uint8_t is_result;
} CalLog_t;
static volatile CalLog_t cal_log[CAL_LOG_SIZE];
static volatile uint8_t cal_log_head, cal_log_tail;
static volatile uint16_t cal_log_dropped;

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
        return 0.5f * (__builtin_fabsf(ia) + __builtin_fabsf(ib));

    if (motor_cal.phase == CAL_PHASE_BC)
        return 0.5f * (__builtin_fabsf(ib) + __builtin_fabsf(ic));

    return 0.5f * (__builtin_fabsf(ic) + __builtin_fabsf(ia));
}


/**
 * @brief 使用最后 CAL_FIT_POINTS 组数据拟合电机线间电阻 R_AB
 *
 * 拟合模型：
 *      V = R * I + b
 *
 * 其中：
 *      V：采样电压
 *      I：采样电流
 *      R：拟合直线斜率，即线间电阻 R_AB
 *      b：电压偏置（截距）
 *
 * 最小二乘法计算斜率：
 *
 *           n*Σ(I*V) - ΣI*ΣV
 *      R = --------------------
 *             n*Σ(I²) - (ΣI)²
 *
 * n 为参与拟合的数据点数量。
 *
 * @return 拟合得到的线间电阻 R_AB，数据不足或分母过小时返回 0
 */
static float Cal_FitResistance(void)
{
    /* 选取最后 CAL_FIT_POINTS 组数据，不足时使用全部数据 */
    uint16_t end = motor_cal.point_count;
    uint16_t start = (end > CAL_FIT_POINTS) ? (end - CAL_FIT_POINTS) : 0U;
    uint16_t n = end - start;

    /* 最小二乘法所需的四个累加量 */
    float si = 0.0f, sv = 0.0f;   // ΣI、ΣV
    float sii = 0.0f, siv = 0.0f; // Σ(I²)、Σ(I*V)

    /* 至少需要两个数据点才能拟合直线 */
    if (n < 2U)
        return 0.0f;

    /* 累加参与拟合的电流、电压及其乘积 */
    for (uint16_t k = start; k < end; k++) {
        float i = motor_cal.point[k].current;
        float v = motor_cal.point[k].voltage;

        si += i;       // ΣI
        sv += v;       // ΣV
        sii += i * i;  // Σ(I²)
        siv += i * v;  // Σ(I*V)
    }

    /* 计算斜率分母：n*Σ(I²) - (ΣI)² */
    float den = (float)n * sii - si * si;

    /* 分母过小意味着电流变化不足，无法可靠计算斜率 */
    if (__builtin_fabsf(den) < 1e-6f)
        return 0.0f;

    /* 计算最小二乘斜率 R_AB */
    return ((float)n * siv - si * sv) / den;
}


void MotorCalibration_Start(void)
{
    MotorCalibration_Stop(); // 先恢复安全停机状态，再配置辨识桥臂。
    memset((void *)&motor_cal, 0, sizeof(motor_cal));
    cal_log_head = cal_log_tail = 0U;
    cal_log_dropped = 0U;

    motor_cal.phase = CAL_PHASE_AB;
    motor_cal.duty = CAL_DUTY_START;
    Cal_ConfigPhaseA();
    motor_cal.state = CAL_SET_DUTY;
}

/* 统一退出：关闭驱动和ADC触发，恢复六路TIM1复用引脚，保持停机。 */
void MotorCalibration_Stop(void)
{
    motor_cal.state = CAL_IDLE;

    __HAL_TIM_MOE_DISABLE(&htim1);
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;

    Motor_PinMode(MOTOR_AH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_AL, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_BH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_BL, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_CH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_CL, MOTOR_GPIO_LOW);

    FOC_PWM_Stop();
    HAL_TIM_MspPostInit(&htim1);
    foc_motor_state = FOC_MOTOR_IDLE; // 硬件恢复后再切换状态。
}

void MotorCalibration_Run(float ia, float ib, float ic, float vbus)
{
    float current = Cal_LineCurrent(ia, ib, ic);

    /* 瞬时硬保护。 __builtin_fabsf();绝对值函数   */
    if ((__builtin_fabsf(ia) >= CAL_HARD_CURRENT) ||
        (__builtin_fabsf(ib) >= CAL_HARD_CURRENT) ||
        (__builtin_fabsf(ic) >= CAL_HARD_CURRENT)) {
        MotorCalibration_Stop();
        motor_cal.state = CAL_ERROR; // 保留错误状态供主循环打印。
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

    case CAL_SAMPLE: {
        /* 按 ADC 回调累加本档采样数据，未满窗口就等待下一次中断。 */
        motor_cal.current_sum += current;
        motor_cal.vbus_sum += vbus;

        if (++motor_cal.count < CAL_SAMPLE_COUNT)
            break;

        /* 采样窗口结束：计算本档平均线电流和平均母线电压。 */
        float avg_i = motor_cal.current_sum / (float)CAL_SAMPLE_COUNT;
        float avg_vbus = motor_cal.vbus_sum / (float)CAL_SAMPLE_COUNT;

        /* 未达到点数上限时保存本档数据；未触发结束条件则继续升压。 */
        if (motor_cal.point_count < CAL_MAX_POINTS) {
            volatile MotorCalPoint_t *p =
                &motor_cal.point[motor_cal.point_count++];

            p->duty = motor_cal.duty;
            p->voltage = motor_cal.duty * avg_vbus;
            p->current = avg_i;

            /* 将本档数值交给主循环，ISR 不格式化或等待串口。 */
            uint8_t next = (uint8_t)((cal_log_head + 1U) & (CAL_LOG_SIZE - 1U));
            if (next != cal_log_tail) {
                cal_log[cal_log_head].point = *p;
                cal_log[cal_log_head].phase = (uint8_t)motor_cal.phase;
                cal_log[cal_log_head].index = (uint8_t)motor_cal.point_count;
                cal_log[cal_log_head].is_result = 0U;
                cal_log_head = next;
            } else {
                cal_log_dropped++;
            }

            /* 电流未达目标且下一档不超限，继续增加占空比。 */
            if ((avg_i < CAL_TARGET_CURRENT) &&
                (motor_cal.duty + CAL_DUTY_STEP <= CAL_DUTY_MAX)) {
                motor_cal.duty += CAL_DUTY_STEP;
                motor_cal.state = CAL_SET_DUTY;
                break;
            }
        }

        /* 达到电流、占空比或点数上限：只关 PWM，保留 ADC 触发用于下一相。 */
        __HAL_TIM_MOE_DISABLE(&htim1);
        TIM1->CCR1 = 0U;
        TIM1->CCR2 = 0U;
        TIM1->CCR3 = 0U;

        /* 按当前回路拟合电阻；AB、BC 依次换相，CA 完成后计算 Rs。 */
        CalPhase_t finished_phase = motor_cal.phase;
        float line_r = Cal_FitResistance();
        switch (finished_phase) {
        case CAL_PHASE_AB:
            motor_cal.r_ab = line_r;
            motor_cal.phase = CAL_PHASE_BC;
            Cal_ConfigPhaseB();
            break;

        case CAL_PHASE_BC:
            motor_cal.r_bc = line_r;
            motor_cal.phase = CAL_PHASE_CA;
            Cal_ConfigPhaseC();
            break;

        case CAL_PHASE_CA:
        default:
            motor_cal.r_ca = line_r;
            motor_cal.r_a = (motor_cal.r_ab + motor_cal.r_ca - motor_cal.r_bc) * 0.5f;
            motor_cal.r_b = (motor_cal.r_ab + motor_cal.r_bc - motor_cal.r_ca) * 0.5f;
            motor_cal.r_c = (motor_cal.r_bc + motor_cal.r_ca - motor_cal.r_ab) * 0.5f;
            motor_cal.rs = (motor_cal.r_a + motor_cal.r_b + motor_cal.r_c) / 3.0f;
            MotorCalibration_Stop();
            motor_cal.state = CAL_DONE; // 保留结果状态供主循环打印。
            break;
        }

        /* 每相结束单独入队电阻结果，避免等三相全部完成。 */
        uint8_t next = (uint8_t)((cal_log_head + 1U) & (CAL_LOG_SIZE - 1U));
        if (next != cal_log_tail) {
            cal_log[cal_log_head].phase = (uint8_t)finished_phase;
            cal_log[cal_log_head].resistance = line_r;
            cal_log[cal_log_head].is_result = 1U;
            cal_log_head = next;
        } else {
            cal_log_dropped++;
        }

        if (finished_phase == CAL_PHASE_CA)
            return;

        /* 换相后从初始占空比重新采样，并复用测量点缓存。 */
        motor_cal.duty = CAL_DUTY_START;
        motor_cal.point_count = 0U;
        motor_cal.state = CAL_SET_DUTY;
        break;
    }

    default:
        MotorCalibration_Stop();
        motor_cal.state = CAL_ERROR; // 保留错误状态供主循环打印。
        break;
    }
}

void MotorCalibration_DebugProcess(void)
{
    static MotorCalState_t last_state = CAL_IDLE;

    /* 每轮最多处理 4 条数值记录，不在 ADC 中断里格式化文本。 */
    for (uint8_t n = 0U; (n < 4U) && (cal_log_tail != cal_log_head); n++) {
        CalLog_t item = cal_log[cal_log_tail];
        cal_log_tail = (uint8_t)((cal_log_tail + 1U) & (CAL_LOG_SIZE - 1U));
        const char *phase = (item.phase == CAL_PHASE_AB) ? "AB" :
                            (item.phase == CAL_PHASE_BC) ? "BC" : "CA";

        if (item.is_result) {
            DebugConsole_Printf("R_%s = %.6f ohm\r\n", phase, item.resistance);
        } else {
            DebugConsole_Printf(
                "%s %02u: duty=%.2f%%  V=%.4fV  I=%.4fA\r\n",
                phase, (unsigned)item.index, item.point.duty * 100.0f,
                item.point.voltage, item.point.current);
        }
    }

    /* 排空测量记录后再打印最终汇总，确保输出先后顺序。 */
    if (cal_log_tail == cal_log_head) {
        if ((motor_cal.state == CAL_DONE) && (last_state != CAL_DONE)) {
            DebugConsole_Printf("\r\n===== Rs Calibration Result =====\r\n"
                                "R_A  = %.6f ohm\r\n"
                                "R_B  = %.6f ohm\r\n"
                                "R_C  = %.6f ohm\r\n"
                                "R_S  = %.6f ohm\r\n",
                                motor_cal.r_a, motor_cal.r_b,
                                motor_cal.r_c, motor_cal.rs);
            if (cal_log_dropped)
                DebugConsole_Printf("WARN: %u calibration logs dropped\r\n",
                                    (unsigned)cal_log_dropped);
            last_state = CAL_DONE;
        } else if ((motor_cal.state == CAL_ERROR) && (last_state != CAL_ERROR)) {
            DebugConsole_Printf("\r\nRs calibration ERROR: duty=%.2f%%\r\n",
                                motor_cal.duty * 100.0f);
            if (cal_log_dropped)
                DebugConsole_Printf("WARN: %u calibration logs dropped\r\n",
                                    (unsigned)cal_log_dropped);
            last_state = CAL_ERROR;
        }
    }

    if ((motor_cal.state != CAL_DONE) && (motor_cal.state != CAL_ERROR))
        last_state = motor_cal.state;
}
