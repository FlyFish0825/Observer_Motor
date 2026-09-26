#include "motor_calibration.h"

#include "debug_console.h"
#include "adc.h"
#include "tim.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

/* 辨识档位数；阶段数组、日志和拟合均以此为边界。 */
#define MOTOR_CALIBRATION_STAGE_COUNT             4U
/* 每档先等待的 ADC 回调数；25 kHz 下约为 200 ms，不参与采样平均。 */
#define MOTOR_CALIBRATION_SETTLE_CYCLES          5000U
/* 每档正式采样点数；25 kHz 下约为 40 ms。 */
#define MOTOR_CALIBRATION_SAMPLE_CYCLES          1000U
/* 正式采样首尾各自用于稳定性判定的窗口长度。 */
#define MOTOR_CALIBRATION_WINDOW_CYCLES           250U
/* 任一被监测相超过此软件电流阈值时立即清 MOE；单位 A。 */
#define RS_IDENTIFY_MAX_CURRENT_A                 1.80f
/* 定电压模式允许的母线电压上限；下限在启动/采样时检查为 1 V。 */
#define RS_IDENTIFY_VOLTAGE_VBUS_MAX_V            30.0f
/* 4 次未右移的 12 位 ADC 采样累加值最大码值：4 * 4095。 */
#define RS_IDENTIFY_ADC_SUM_MAX                   16380.0f
/* 主循环发现 ADC 回调超过此时长未更新即故障退出；单位 ms。 */
#define RS_IDENTIFY_ADC_STALL_TIMEOUT_MS          20U
/* 单个档位从开始到结束的最长时间；单位 ms。 */
#define RS_IDENTIFY_STAGE_TIMEOUT_MS             500U
/* 线性拟合要求的最低平均电流；单位 A。 */
#define MOTOR_CALIBRATION_MIN_CURRENT_A           0.02f
/* 多档拟合的最小电流跨度；单位 A。 */
#define MOTOR_CALIBRATION_MIN_CURRENT_SPREAD_A    0.02f
/* 默认定电压模式首尾窗口允许的电流幅值均值差；单位 A。 */
#define MOTOR_CALIBRATION_VOLTAGE_STABLE_DELTA_A  0.010f
#define MOTOR_CALIBRATION_TARGET_TOLERANCE        0.05f
/* 标定期间允许的最大 PWM 占空比，范围 0..1。 */
#define MOTOR_CALIBRATION_MAX_DUTY                0.20f
/* 定电压模式每个 ADC 周期最大 Duty 变化量，限制电压建立斜率。 */
#define MOTOR_CALIBRATION_VOLTAGE_DUTY_STEP       0.0001f
/* 判断实际量化 Duty 已到定电压目标的容差；Duty 比例。 */
#define MOTOR_CALIBRATION_DUTY_SETTLE_TOLERANCE  0.001f
/* 正式采样前要求连续达到目标 Duty 的回调数。 */
#define MOTOR_CALIBRATION_DUTY_SETTLE_CYCLES      250U
/* 最小二乘正规方程分母下限；用于拒绝数值退化的拟合输入。 */
#define MOTOR_CALIBRATION_FIT_DENOMINATOR_MIN     1.0e-6f

/* 一次清零全部三相主输出与互补输出的 TIM1 CCER 位掩码；不包含 ADC 触发 CH4。 */
#define MOTOR_CALIBRATION_PHASE_CCER_MASK \
    (TIM_CCER_CC1E | TIM_CCER_CC1NE | \
     TIM_CCER_CC2E | TIM_CCER_CC2NE | \
     TIM_CCER_CC3E | TIM_CCER_CC3NE)

/* 默认定电压模式的四档命令平均电压，单位 V，不代表实测绕组电压。 */
/**
 * @brief 默认定电压模式的四档命令目标；按阶段索引访问，单位 V。
 * @note 这是控制器目标值，不是绕组端实测 AB 差分电压。
 */
static const float rs_voltage_targets[MOTOR_CALIBRATION_STAGE_COUNT] = {
    0.65f, 0.95f, 1.25f, 1.55f
};

/** @brief 单一注入档的采样累计值、稳定性判定和故障标志。 */
typedef struct {
    float sum_abs_ib, sum_u_cmd, sum_start_abs_ib, sum_end_abs_ib;
    uint32_t sample_count, start_window_count, end_window_count;
    uint32_t duty_target_consecutive_cycles;
    uint8_t voltage_target_reached, duty_target_settled, current_stable;
} MotorCalibrationStageData_t;

typedef struct {
    volatile MotorCalibrationState_t state;
    volatile uint8_t finish_pending;
    volatile MotorCalibrationState_t terminal_state;
    uint32_t cycle;
    uint8_t stage;
    float duty;
    volatile uint32_t last_adc_step_tick_ms, stage_start_tick_ms;
    float next_sample_duty;
    MotorCalibrationStageData_t stage_data[MOTOR_CALIBRATION_STAGE_COUNT];
    float rs_all;
    uint8_t error_code;
} MotorCalibrationContext_t;

static MotorCalibrationContext_t motor_calibration;

/**
 * @brief 按安全次序关闭全部三相功率输出并清零比较寄存器。
 * @return 无；TIM1 MOE、三相 CCER 和 CCR1～CCR3 均被清除。
 * @note 先清 MOE 并执行屏障，再改 CCER，避免改变通道配置时仍驱动桥臂。
 */
static void MotorCalibration_DisablePhaseOutputs(void)
{
    /* MOE 是桥臂输出的总门控；先清 MOE 并等待写入完成，再改通道使能。 */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    __DSB();
    TIM1->CCER &= ~MOTOR_CALIBRATION_PHASE_CCER_MASK;
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
}

/**
 * @brief 从 ISR 或主循环发起辨识收尾。
 * @param terminal_state 请求的最终状态，通常为 DONE 或 ERROR。
 * @param error_code 错误原因码；成功传 0，中止及故障使用对应非零值。
 * @return 无；若处于活动状态则立即关断相输出，并通知主循环继续收尾。
 */
static void MotorCalibration_Finish(MotorCalibrationState_t terminal_state,
                                    uint8_t error_code)
{
    if ((motor_calibration.state != MOTOR_CALIBRATION_RUNNING) &&
        (motor_calibration.state != MOTOR_CALIBRATION_STARTING))
    {
        return;
    }

    /* 保持 FOC 回调处于辨识独占状态，直到主循环完成定时器停机。 */
    motor_calibration.state = MOTOR_CALIBRATION_STOPPING;
    MotorCalibration_DisablePhaseOutputs();
    motor_calibration.terminal_state = terminal_state;
    motor_calibration.error_code = error_code;
    motor_calibration.finish_pending = 1U;
    __DMB();
}

/**
 * @brief 安全计算一组累计值的算术平均数。
 * @param sum 样本总和，单位由调用方决定。
 * @param count 样本数。
 * @return sum/count；count 为 0 时返回 NAN，避免把空数据伪装为 0。
 */
static float MotorCalibration_Average(float sum, uint32_t count)
{
    return (count == 0U) ? NAN : (sum / (float)count);
}

/**
 * @brief 根据阶段累计数据计算目标到位及电流稳定标志。
 * @param index 阶段数组索引，调用方须保证小于 MOTOR_CALIBRATION_STAGE_COUNT。
 * @return 无；结果写入电压目标和电流稳定标志。
 */
static void MotorCalibration_CheckStage(uint8_t index)
{
    /* 下列量均由当前阶段样本计算；电流使用幅值，电压使用实际配对命令。 */
    MotorCalibrationStageData_t *data = &motor_calibration.stage_data[index]; /* 当前阶段累计记录。 */
    float target_u = rs_voltage_targets[index]; /* 默认模式命令电压目标，V。 */
    float average_u = MotorCalibration_Average(data->sum_u_cmd,
                                                data->sample_count); /* 命令电压全窗均值，V。 */
    float start_i = MotorCalibration_Average(data->sum_start_abs_ib,
                                              data->start_window_count); /* 首窗口 |Ib| 均值，A。 */
    float end_i = MotorCalibration_Average(data->sum_end_abs_ib,
                                            data->end_window_count); /* 末窗口 |Ib| 均值，A。 */
    data->voltage_target_reached =
        (data->sample_count == MOTOR_CALIBRATION_SAMPLE_CYCLES) &&
        isfinite(average_u) &&
        (fabsf(average_u - target_u) <=
         target_u * MOTOR_CALIBRATION_TARGET_TOLERANCE);
    data->current_stable =
        (data->start_window_count == MOTOR_CALIBRATION_WINDOW_CYCLES) &&
        (data->end_window_count == MOTOR_CALIBRATION_WINDOW_CYCLES) &&
        isfinite(start_i) && isfinite(end_i) &&
        (fabsf(end_i - start_i) < MOTOR_CALIBRATION_VOLTAGE_STABLE_DELTA_A);
}

/* 使用四档平均电压/电流拟合 U=R_AB*I+b，再换算单相 Rs=R_AB/2。 */
static uint8_t MotorCalibration_Fit(float *rs)
{
    float sum_i = 0.0f, sum_u = 0.0f, sum_ii = 0.0f, sum_iu = 0.0f;
    float min_i = INFINITY, max_i = -INFINITY;
    const uint8_t n = MOTOR_CALIBRATION_STAGE_COUNT;

    for (uint8_t i = 0U; i < n; ++i) {
        const MotorCalibrationStageData_t *stage = &motor_calibration.stage_data[i];
        float current = MotorCalibration_Average(stage->sum_abs_ib, stage->sample_count);
        float voltage = MotorCalibration_Average(stage->sum_u_cmd, stage->sample_count);
        if (!isfinite(current) || !isfinite(voltage)) return 0U;
        if (current < min_i) min_i = current;
        if (current > max_i) max_i = current;
        sum_i += current;
        sum_u += voltage;
        sum_ii += current * current;
        sum_iu += current * voltage;
    }

    float denominator = (float)n * sum_ii - sum_i * sum_i;
    if ((max_i - min_i) <= MOTOR_CALIBRATION_MIN_CURRENT_SPREAD_A ||
        (sum_i / (float)n) < MOTOR_CALIBRATION_MIN_CURRENT_A ||
        !isfinite(denominator) || denominator <= MOTOR_CALIBRATION_FIT_DENOMINATOR_MIN)
        return 0U;

    float slope = ((float)n * sum_iu - sum_i * sum_u) / denominator;
    if (!isfinite(slope) || slope <= 0.0f) return 0U;
    *rs = slope * 0.5f;
    return 1U;
}

static uint8_t MotorCalibration_ComputeFits(void)
{
    return MotorCalibration_Fit(&motor_calibration.rs_all);
}

static void MotorCalibration_ResetData(void)
{
    motor_calibration.cycle = 0U;
    motor_calibration.stage = 0U;
    motor_calibration.duty = 0.0f;
    motor_calibration.last_adc_step_tick_ms = 0U;
    motor_calibration.stage_start_tick_ms = 0U;
    motor_calibration.finish_pending = 0U;
    motor_calibration.next_sample_duty = 0.0f;
    motor_calibration.rs_all = NAN;
    motor_calibration.error_code = 0U;
    motor_calibration.terminal_state = MOTOR_CALIBRATION_ERROR;
    memset(motor_calibration.stage_data, 0, sizeof(motor_calibration.stage_data));
}

static void MotorCalibration_PrintHeader(void)
{
    DebugConsole_Printf("RS_IDENTIFY STARTED VBUS=%.3f CURRENT_LIMIT=%.2f\r\n",
                        (double)foc.state.vbus,
                        (double)RS_IDENTIFY_MAX_CURRENT_A);
}

static uint8_t MotorCalibration_AdcHasCurrentHeadroom(void)
{
    /* 三相各自拥有独立增益和零偏；统一用相同的电流阈值估算码值余量。 */
    const float gains[3] = { /* 各相安培/ADC 码换算系数，A/code。 */
        foc.current.gain_a, foc.current.gain_b, foc.current.gain_c
    };
    const float offsets[3] = { /* 各相 ADC 零偏码值。 */
        foc.calibration.ia_offset,
        foc.calibration.ib_offset,
        foc.calibration.ic_offset
    };
    uint8_t index; /* 遍历 A、B、C 三相。 */

    /* 四次未右移的 12 位转换累加为 0..16380 码；这里只核对数字余量，
     * 不验证模拟前端线性度，也不能代替独立硬件过流保护。 */
    for (index = 0U; index < 3U; ++index)
    {
        float threshold_codes; /* 软件过流安培值换算到 ADC 码值。 */
        if ((!isfinite(gains[index])) || (gains[index] <= 0.0f) ||
            (!isfinite(offsets[index])))
        {
            return 0U;
        }
        threshold_codes = RS_IDENTIFY_MAX_CURRENT_A / gains[index];
        if ((!isfinite(threshold_codes)) ||
            (offsets[index] <= threshold_codes) ||
            ((RS_IDENTIFY_ADC_SUM_MAX - offsets[index]) <= threshold_codes))
        {
            return 0U;
        }
    }
    return 1U;
}

/**
 * @brief 把数值错误码转换为日志使用的短文本原因。
 * @param error_code 当前错误码。
 * @return 指向静态字符串的只读指针；未知码返回 UNKNOWN。
 */
static const char *MotorCalibration_AbortReason(uint8_t error_code)
{
    switch (error_code)
    {
    case 0U: return "NONE"; /* 无错误；辨识已完成。 */
    case 1U: return "STOP_REQUESTED"; /* 用户请求停止。 */
    case 2U: return "OVERCURRENT"; /* 任一被监测相超过软件限流值。 */
    case 3U: return "FIT_INVALID"; /* 最终电阻拟合不满足数值条件。 */
    case 4U: return "INVALID_SAMPLE"; /* 电流或配对 Duty 非有限/非法。 */
    case 5U: return "STAGE_INVALID"; /* 采样完成但至少一档未通过有效性检查。 */
    case 6U: return "DUTY_SATURATED"; /* 目标 Duty 超限或电流模式触及 Duty 上限。 */
    case 7U: return "ADC_STALL"; /* 超过看门狗时间没有新的 ADC 回调。 */
    case 8U: return "STAGE_TIMEOUT"; /* 当前档运行时长超过上限。 */
    case 9U: return "ADC_RANGE_INVALID"; /* ADC 参数或数字量程余量不足。 */
    case 10U: return "DUTY_NOT_SETTLED"; /* 正式采样前 Duty 连续到位次数不足。 */
    case 11U: return "VBUS_INVALID"; /* 母线电压超出允许范围。 */
    case 12U: return "CURRENT_UNSTABLE"; /* 首尾电流窗口差超过模式阈值。 */
    case 13U: return "VOLTAGE_TARGET_NOT_REACHED"; /* 定电压档平均命令电压误差过大。 */
    default: return "UNKNOWN"; /* 尚未定义的错误码。 */
    }
}

/**
 * @brief 输出四档采样、有效性、拟合及退出原因报告。
 * @return 无；仅由主循环在功率输出关闭后调用，不在 ADC ISR 打印。
 */
HAL_StatusTypeDef MotorCalibration_Start(void)
{
    HAL_StatusTypeDef status; /* HAL 定时器启动状态，传回调用方。 */
    uint32_t adc_trigger_mask = TIM_CCER_CC4E; /* CH4 ADC 触发输出位。 */
    uint32_t primask; /* 保存中断屏蔽状态，确保只恢复本函数改变的状态。 */

    if (MotorCalibration_LsIsActive() ||
        (motor_calibration.state == MOTOR_CALIBRATION_STARTING) ||
        (motor_calibration.state == MOTOR_CALIBRATION_RUNNING) ||
        (motor_calibration.state == MOTOR_CALIBRATION_STOPPING) ||
        (motor_calibration.finish_pending != 0U))
    {
        return HAL_BUSY;
    }

    if ((foc_motor_state != FOC_MOTOR_IDLE) ||
        (foc.calibration.calibrated == 0U) ||
        (!isfinite(foc.state.vbus)) ||
        (foc.state.vbus < 1.0f) ||
        (foc.state.vbus > RS_IDENTIFY_VOLTAGE_VBUS_MAX_V))
    {
        return HAL_ERROR;
    }
    if (MotorCalibration_AdcHasCurrentHeadroom() == 0U)
    {
        motor_calibration.error_code = 9U;
        motor_calibration.state = MOTOR_CALIBRATION_ERROR;
        DebugConsole_Printf("RS_IDENTIFY_ABORT_REASON=ADC_RANGE_INVALID\r\n");
        return HAL_ERROR;
    }

    /* 启动过程的阶段顺序：占用状态 -> 关 MOE -> HAL/寄存器同步 ->
     * 清零比较值并产生更新事件 -> 仅启 CH4 -> 配置桥臂 -> 最终开 MOE。 */
    /* 先独占电机状态；后续 ADC 回调将绕过正常 FOC 控制路径。 */
    motor_calibration.state = MOTOR_CALIBRATION_STARTING;
    foc_motor_state = FOC_MOTOR_CALIBRATION;
    __DMB();
    MotorCalibration_ResetData();
    motor_calibration.state = MOTOR_CALIBRATION_STARTING;

    /* 修改定时器前先关总门控并关闭三相通道。 */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    __DSB();
    TIM1->CCER &= ~MOTOR_CALIBRATION_PHASE_CCER_MASK;

    /* MOE 关闭时同步 HAL 通道状态与硬件寄存器状态。 */
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER &= ~(MOTOR_CALIBRATION_PHASE_CCER_MASK | adc_trigger_mask);
    TIM1->CR1 &= ~TIM_CR1_CEN;

    /* 先写入零比较值，再通过更新事件装载预装载寄存器。 */
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    TIM1->CCR4 = foc.timer.adc_trigger;
    TIM1->CNT = 0U;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR &= ~TIM_SR_UIF;

    if (motor_calibration.state != MOTOR_CALIBRATION_STARTING)
    {
        return HAL_BUSY;
    }

    /* 仅启动 CH4 恢复注入 ADC 触发，此时相输出 CCER 仍关闭。 */
    status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    if (status != HAL_OK)
    {
        MotorCalibration_DisablePhaseOutputs();
        TIM1->CCER &= ~TIM_CCER_CC4E;
        TIM1->CR1 &= ~TIM_CR1_CEN;
        foc_motor_state = FOC_MOTOR_IDLE;
        motor_calibration.state = MOTOR_CALIBRATION_IDLE;
        return status;
    }

    /* HAL 启动 CH4 会置 MOE，因此立即清除；相输出通道仍被 CCER 隔离。 */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CNT = 0U;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR &= ~TIM_SR_UIF;

    if (motor_calibration.state != MOTOR_CALIBRATION_STARTING)
    {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        return HAL_BUSY;
    }

    /* CCR2=0 时要让 B 下管 CH2N 导通，必须同时使能 CH2 与 CH2N。 */
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE |
                  TIM_CCER_CC2E | TIM_CCER_CC2NE;
    __DSB();

    /*
     * 临界区内原子完成最终 MOE 使能和 STARTING->RUNNING 状态转换，
     * 防止 GPIO 停止请求与启动末段交错；若已请求停止则保持输出关闭。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    if (motor_calibration.state != MOTOR_CALIBRATION_STARTING)
    {
        MotorCalibration_DisablePhaseOutputs();
        if (primask == 0U)
        {
            __enable_irq();
        }
        return HAL_BUSY;
    }

    /* CCR 预装载、更新事件和桥臂通道均就绪后，才最终打开 MOE。 */
    motor_calibration.last_adc_step_tick_ms = HAL_GetTick();
    motor_calibration.stage_start_tick_ms =
        motor_calibration.last_adc_step_tick_ms;
    TIM1->BDTR |= TIM_BDTR_MOE;
    motor_calibration.state = MOTOR_CALIBRATION_RUNNING;
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }

    MotorCalibration_PrintHeader();
    return HAL_OK;
}

/**
 * @brief 以“用户请求停止”错误码进入安全关断流程。
 * @return 无；若模块正处于 STARTING/RUNNING，则立即关断相输出。
 */
void MotorCalibration_Stop(void)
{
    MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 1U);
}

/**
 * @brief 消费本次 ADC 数据、检查保护、更新 Duty 或累计一个辨识样本。
 * @param adc_b_raw 当前 ADC_B 过采样原始码值，单位 ADC code。
 * @return 无；任何数据异常、过流或超时都会请求安全退出。
 * @note 按当前触发与预装载时序，先以“上次实际施加”的 CCR/Duty 配对本样本，
 *       再计算并写入下一次 PWM 命令；ISR 中严禁格式化或打印日志。
 */
void MotorCalibration_AdcStep(void)
{
    float signed_ia;       /* 当前采样换算的有符号 A 相电流，A。 */
    float signed_ib;       /* 当前采样换算的有符号 B 相电流，A。 */
    float signed_ic;       /* 当前采样换算的有符号 C 相电流，A。 */
    float current_a;       /* 电流幅值 |Ib|，供保护及控制/辨识使用，A。 */
    float target_duty;     /* 定电压目标除以当前母线后的目标 Duty。 */
    float sample_duty;     /* 与当前 ADC 样本对应的已施加 Duty。 */
    float sample_vbus;     /* 当前样本对应的母线电压，V。 */
    float sample_u_ideal;  /* 已施加 Duty * Vbus 的理想命令电压，V。 */
    MotorCalibrationStageData_t *data; /* 当前阶段的累计数据记录。 */
    uint16_t next_ccr1;    /* 本次计算后写给预装载寄存器的下一 CCR1。 */


    if (motor_calibration.state != MOTOR_CALIBRATION_RUNNING)
    {
        return;
    }

    motor_calibration.last_adc_step_tick_ms = HAL_GetTick();
    data = &motor_calibration.stage_data[motor_calibration.stage];
    signed_ia = foc.state.i_abc.a;
    signed_ib = foc.state.i_abc.b;
    signed_ic = foc.state.i_abc.c;
    current_a = fabsf(signed_ib);
    /* 首先拒绝 NaN/Inf，再执行相电流保护；保护检查位于 Duty 更新与累计之前。 */
    if ((!isfinite(signed_ia)) || (!isfinite(signed_ib)) ||
        (!isfinite(signed_ic)) || (!isfinite(foc.state.vbus)))
    {
        MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 4U);
        return;
    }
    if ((fabsf(signed_ia) > RS_IDENTIFY_MAX_CURRENT_A) ||
        (current_a > RS_IDENTIFY_MAX_CURRENT_A) ||
        (fabsf(signed_ic) > RS_IDENTIFY_MAX_CURRENT_A))
    {
        MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 2U);
        return;
    }
    if ((motor_calibration.last_adc_step_tick_ms -
         motor_calibration.stage_start_tick_ms) >
        RS_IDENTIFY_STAGE_TIMEOUT_MS)
    {
        MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 8U);
        return;
    }

    /* 先将本次 ADC 样本与上次回调之后已生效的 PWM 命令配对。 */
    sample_duty = motor_calibration.next_sample_duty;
    sample_vbus = foc.state.vbus;
    if ((sample_vbus < 1.0f) ||
        (sample_vbus > RS_IDENTIFY_VOLTAGE_VBUS_MAX_V))
    {
        MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 11U);
        return;
    }
    /* 电压累计使用对应样本的量化 CCR 换算 Duty，而非当前新写的目标值。 */
    if ((!isfinite(sample_duty)) || (sample_duty < 0.0f) ||
        (sample_duty > MOTOR_CALIBRATION_MAX_DUTY))
    {
        MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 4U);
        return;
    }
    sample_u_ideal = sample_duty * sample_vbus;

    target_duty = rs_voltage_targets[motor_calibration.stage] / sample_vbus;
    if ((!isfinite(target_duty)) || (target_duty < 0.0f))
    {
        MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 11U);
        return;
    }
    if (target_duty > MOTOR_CALIBRATION_MAX_DUTY)
    {
        MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 6U);
        return;
    }
    /* 每个 ADC 周期只改变有限 Duty；切档时也以相同步长平滑过渡。 */
    if (motor_calibration.duty < target_duty)
    {
        motor_calibration.duty = fminf(
            motor_calibration.duty + MOTOR_CALIBRATION_VOLTAGE_DUTY_STEP,
            target_duty);
    }
    else
    {
        motor_calibration.duty = fmaxf(
            motor_calibration.duty - MOTOR_CALIBRATION_VOLTAGE_DUTY_STEP,
            target_duty);
    }

    next_ccr1 = (uint16_t)FOC_DutyToCCR(
        motor_calibration.duty, foc.timer.pwm_arr);
    TIM1->CCR1 = next_ccr1;
    /* 记录量化后的下一 PWM 命令，供下一次 ADC 样本正确配对。 */
    motor_calibration.next_sample_duty =
        (float)next_ccr1 / (float)(foc.timer.pwm_arr + 1U);

    /* 等待阶段仅计数并确认 Duty 到位，不把爬升样本混入正式平均。 */
    if (motor_calibration.cycle < MOTOR_CALIBRATION_SETTLE_CYCLES)
    {
        /* 只统计本次采样之前已生效的 Duty 是否到达目标。 */
        if (fabsf(sample_duty - target_duty) <=
            MOTOR_CALIBRATION_DUTY_SETTLE_TOLERANCE)
            data->duty_target_consecutive_cycles++;
        else
            data->duty_target_consecutive_cycles = 0U;
        motor_calibration.cycle++;
        return;
    }

    if (data->sample_count == 0U)
    {
        data->duty_target_settled =
            data->duty_target_consecutive_cycles >=
            MOTOR_CALIBRATION_DUTY_SETTLE_CYCLES;
        if (data->duty_target_settled == 0U)
        {
            MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 10U);
            return;
        }
    }

    {
        /* 首尾 250 点为不重叠窗口，用于判断正式采样期间电流是否仍漂移。 */
        if (data->sample_count < MOTOR_CALIBRATION_WINDOW_CYCLES)
        {
            data->sum_start_abs_ib += current_a;
            data->start_window_count++;
        }
        if (data->sample_count >=
            MOTOR_CALIBRATION_SAMPLE_CYCLES -
            MOTOR_CALIBRATION_WINDOW_CYCLES)
        {
            data->sum_end_abs_ib += current_a;
            data->end_window_count++;
        }
        data->sum_abs_ib += current_a;
        data->sum_u_cmd += sample_u_ideal;
        data->sample_count++;
    }

    motor_calibration.cycle++;
    if (motor_calibration.stage_data[motor_calibration.stage].sample_count <
        MOTOR_CALIBRATION_SAMPLE_CYCLES)
    {
        return;
    }

    MotorCalibration_CheckStage(motor_calibration.stage);
    if (data->current_stable == 0U)
    {
        MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 12U);
        return;
    }
    if (data->voltage_target_reached == 0U)
    {
        MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 13U);
        return;
    }

    motor_calibration.cycle = 0U;
    motor_calibration.stage++;
    if (motor_calibration.stage < MOTOR_CALIBRATION_STAGE_COUNT)
    {
        motor_calibration.stage_start_tick_ms = HAL_GetTick();
        return;
    }

    /* 拟合及最终有效性判定交由主循环，在相输出关断后执行。 */
    MotorCalibration_Finish(MOTOR_CALIBRATION_DONE, 0U);
}

/**
 * @brief 主循环轮询 ADC/阶段看门狗，并执行中断请求的完整关断和结果计算。
 * @return 无；辨识结束时输出报告，成功才公开有效的 Rs。
 * @note 先关 MOE 和相输出，再停止 CH4/TIM1，最后校验阶段并做拟合。
 */
void MotorCalibration_Process(void)
{
    MotorCalibrationState_t terminal_state; /* 最终需要发布的成功/失败状态。 */

    if (motor_calibration.state == MOTOR_CALIBRATION_RUNNING)
    {
        uint32_t now = HAL_GetTick(); /* 当前毫秒 tick，用于两种看门狗。 */
        if ((now - motor_calibration.last_adc_step_tick_ms) >
            RS_IDENTIFY_ADC_STALL_TIMEOUT_MS)
        {
            MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 7U);
        }
        else if ((now - motor_calibration.stage_start_tick_ms) >
                 RS_IDENTIFY_STAGE_TIMEOUT_MS)
        {
            MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 8U);
        }
    }

    if (motor_calibration.finish_pending == 0U)
    {
        return;
    }

    terminal_state = motor_calibration.terminal_state;
    motor_calibration.finish_pending = 0U;

    /* 先切断相输出，再停 CH4 和计数器，避免停机过程中桥臂仍被驱动。 */
    MotorCalibration_DisablePhaseOutputs();
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER &= ~(MOTOR_CALIBRATION_PHASE_CCER_MASK | TIM_CCER_CC4E);
    TIM1->CR1 &= ~TIM_CR1_CEN;
    foc.state.ibus_est = 0.0f;
    foc.state.ibus_filter = 0.0f;

    if (terminal_state == MOTOR_CALIBRATION_DONE)
    {
        if (MotorCalibration_ComputeFits() == 0U)
        {
            terminal_state = MOTOR_CALIBRATION_ERROR;
            motor_calibration.error_code = 3U;
        }
        motor_calibration.terminal_state = terminal_state;
    }

    /* 不恢复保存的 CCER；辨识结束后所有相功率输出都保持关闭。 */
    foc_motor_state = FOC_MOTOR_IDLE;
    motor_calibration.state = terminal_state;
    __DMB();

    if (terminal_state == MOTOR_CALIBRATION_DONE)
    {
        DebugConsole_Printf("RS_IDENTIFY DONE Rs=%.6f ohm\r\n",
                            (double)motor_calibration.rs_all);
    }
    else
    {
        DebugConsole_Printf("RS_IDENTIFY ERROR reason=%s\r\n",
                            MotorCalibration_AbortReason(motor_calibration.error_code));
    }
}

/**
 * @brief 判断启动、采样或停止收尾是否仍由辨识模块占有。
 * @return 活动状态返回 1；IDLE、DONE、ERROR 等稳定终态返回 0。
 */
uint8_t MotorCalibration_IsActive(void)
{
    return ((motor_calibration.state == MOTOR_CALIBRATION_STARTING) ||
            (motor_calibration.state == MOTOR_CALIBRATION_RUNNING) ||
            (motor_calibration.state == MOTOR_CALIBRATION_STOPPING)) ? 1U : 0U;
}

/**
 * @brief 获取模块当前状态快照。
 * @return MotorCalibrationState_t 枚举状态。
 */
MotorCalibrationState_t MotorCalibration_GetState(void)
{
    return motor_calibration.state;
}

/**
 * @brief 获取四档完整拟合所得单相 Rs。
 * @return 完成且有效时返回欧姆；其他状态返回 NAN，防止误用失败结果。
 */
float MotorCalibration_GetResultOhm(void)
{
    return (motor_calibration.state == MOTOR_CALIBRATION_DONE) ?
           motor_calibration.rs_all : NAN;
}

/**
 * @brief 获取当前保存的辨识错误码。
 * @return 0 表示无错误，非零错误码与 MotorCalibration_AbortReason 对应。
 */
uint8_t MotorCalibration_GetErrorCode(void)
{
    return motor_calibration.error_code;
}

/* Ls pulse sampling and curve fitting. */

#define LS_MIN_SAMPLES 5U
#define LS_MIN_SPAN_US 8.0f
#define LS_MIN_CURRENT_A 0.02f
#define LS_MAX_RMSE_A 0.02f
#define LS_MAX_RELATIVE_L_DIFF 0.40f

typedef struct {
    double n, sx, sy, sxx, sxy;
    double slope, intercept;
    uint32_t count;
    float start_us, end_us;
} LsRegression_t;

static void LsRegression_Add(LsRegression_t *reg, double x, double y)
{
    if (reg->count == 0U) reg->start_us = (float)x;
    reg->end_us = (float)x;
    reg->count++;
    reg->n += 1.0;
    reg->sx += x;
    reg->sy += y;
    reg->sxx += x * x;
    reg->sxy += x * y;
}

static uint8_t LsRegression_Solve(LsRegression_t *reg)
{
    double denominator = reg->n * reg->sxx - reg->sx * reg->sx;
    if ((reg->count < LS_MIN_SAMPLES) ||
        ((reg->end_us - reg->start_us) < LS_MIN_SPAN_US) ||
        (!isfinite(denominator)) || (denominator <= 1.0e-9)) return 0U;
    reg->slope = (reg->n * reg->sxy - reg->sx * reg->sy) / denominator;
    reg->intercept = (reg->sy - reg->slope * reg->sx) / reg->n;
    return (isfinite(reg->slope) && isfinite(reg->intercept)) ? 1U : 0U;
}

static float LsCurrent(const LsSample_t *sample, const LsFitConfig_t *config)
{
    return ((float)sample->adc_raw - config->adc_offset) *
           config->adc_gain_a_per_code;
}

LsFitStatus_t MotorCalibration_LsFit(const LsSample_t *samples, size_t count,
                                    const LsFitConfig_t *config,
                                    LsFitResult_t *result)
{
    LsRegression_t rise = {0}, decay = {0};
    uint32_t previous = 0U;
    uint8_t saw_rise = 0U, saw_decay = 0U;
    double rise_error = 0.0, decay_error = 0.0;

    if (result == NULL) return LS_FIT_BAD_ARGUMENT;
    memset(result, 0, sizeof(*result));
    result->status = LS_FIT_BAD_ARGUMENT;
    if ((samples == NULL) || (config == NULL) || (count == 0U) ||
        !isfinite(config->timestamp_tick_us) || (config->timestamp_tick_us <= 0.0f) ||
        !isfinite(config->adc_offset) ||
        !isfinite(config->adc_gain_a_per_code) || (config->adc_gain_a_per_code <= 0.0f) ||
        !isfinite(config->applied_voltage_v) || (config->applied_voltage_v <= 0.0f) ||
        !isfinite(config->loop_resistance_ohm) || (config->loop_resistance_ohm <= 0.0f) ||
        !isfinite(config->current_limit_a) || (config->current_limit_a <= 0.0f) ||
        !isfinite(config->pulse_hard_limit_us) || (config->pulse_hard_limit_us <= 0.0f) ||
        !isfinite(config->blanking_us) || (config->blanking_us < 0.0f)) return result->status;
    if (config->adc_overrun) return result->status = LS_FIT_ADC_OVERRUN;
    if ((config->pulse_start_timestamp >= config->pulse_end_timestamp) ||
        (config->pulse_end_timestamp > config->freewheel_start_timestamp) ||
        ((double)(config->pulse_end_timestamp - config->pulse_start_timestamp) *
         config->timestamp_tick_us > config->pulse_hard_limit_us))
        return result->status = LS_FIT_TIMING;

    for (size_t i = 0U; i < count; i++) {
        float current = LsCurrent(&samples[i], config);
        if ((i != 0U) && (samples[i].timestamp <= previous))
            return result->status = LS_FIT_TIMING;
        previous = samples[i].timestamp;
        if (samples[i].state > LS_SAMPLE_INVALID)
            return result->status = LS_FIT_BAD_ARGUMENT;
        if (!isfinite(current)) return result->status = LS_FIT_NUMERIC;
        if (fabsf(current) > config->current_limit_a)
            return result->status = LS_FIT_OVERCURRENT;
        if (samples[i].adc_raw >= 4095U)
            return result->status = LS_FIT_SATURATED;
        if (samples[i].state == LS_SAMPLE_RISE) {
            if (saw_decay) return result->status = LS_FIT_TIMING;
            if ((samples[i].timestamp < config->pulse_start_timestamp) ||
                (samples[i].timestamp > config->pulse_end_timestamp))
                return result->status = LS_FIT_TIMING;
            saw_rise = 1U;
        } else if (samples[i].state == LS_SAMPLE_DECAY) {
            if (!saw_rise) return result->status = LS_FIT_TIMING;
            if (samples[i].timestamp < config->freewheel_start_timestamp)
                return result->status = LS_FIT_TIMING;
            saw_decay = 1U;
        }
    }
    if (!saw_rise || !saw_decay) return result->status = LS_FIT_SAMPLE_SHORT;
    if (count > 1U)
        result->sample_period_us = (float)((double)(samples[count - 1U].timestamp -
            samples[0].timestamp) * config->timestamp_tick_us / (count - 1U));

    for (size_t i = 0U; i < count; i++) {
        float current = LsCurrent(&samples[i], config);
        float t_us = (float)(((double)samples[i].timestamp -
                             config->pulse_start_timestamp) *
                             config->timestamp_tick_us);
        if ((samples[i].state == LS_SAMPLE_RISE) &&
            (t_us >= config->blanking_us) && (current > LS_MIN_CURRENT_A))
            LsRegression_Add(&rise, t_us, current);
        if ((samples[i].state == LS_SAMPLE_DECAY) &&
            (current > LS_MIN_CURRENT_A))
            LsRegression_Add(&decay, t_us, log((double)current));
    }
    result->rise_samples = rise.count;
    result->decay_samples = decay.count;
    result->rise_start_us = rise.start_us;
    result->rise_end_us = rise.end_us;
    result->decay_start_us = decay.start_us;
    result->decay_end_us = decay.end_us;
    if (!LsRegression_Solve(&rise) || !LsRegression_Solve(&decay))
        return result->status = LS_FIT_SAMPLE_SHORT;
    if ((rise.slope <= 0.0) || (decay.slope >= 0.0))
        return result->status = LS_FIT_NUMERIC;

    /* 上升斜率单位 A/us；下降段对 ln(I) 的斜率单位 1/us。 */
    double mean_rise_a = rise.sy / rise.n;
    double l_rise_uh = (config->applied_voltage_v -
                        config->loop_resistance_ohm * mean_rise_a) / rise.slope;
    double tau_decay_us = -1.0 / decay.slope;
    double l_decay_uh = tau_decay_us * config->loop_resistance_ohm;
    if (!isfinite(l_rise_uh) || !isfinite(l_decay_uh) ||
        (l_rise_uh <= 0.0) || (l_decay_uh <= 0.0))
        return result->status = LS_FIT_NUMERIC;

    for (size_t i = 0U; i < count; i++) {
        float current = LsCurrent(&samples[i], config);
        float t_us = (float)(((double)samples[i].timestamp -
                             config->pulse_start_timestamp) *
                             config->timestamp_tick_us);
        if ((samples[i].state == LS_SAMPLE_RISE) &&
            (t_us >= config->blanking_us) && (current > LS_MIN_CURRENT_A)) {
            double error = current - (rise.intercept + rise.slope * t_us);
            rise_error += error * error;
        }
        if ((samples[i].state == LS_SAMPLE_DECAY) &&
            (current > LS_MIN_CURRENT_A)) {
            double error = current - exp(decay.intercept + decay.slope * t_us);
            decay_error += error * error;
        }
    }
    result->tau_rise_us = (float)(l_rise_uh / config->loop_resistance_ohm);
    result->tau_decay_us = (float)tau_decay_us;
    result->l_ab_rise_uh = (float)l_rise_uh;
    result->l_ab_decay_uh = (float)l_decay_uh;
    result->ls_rise_uh = (float)(l_rise_uh / 2.0);
    result->ls_decay_uh = (float)(l_decay_uh / 2.0);
    result->rise_rmse_a = (float)sqrt(rise_error / rise.n);
    result->decay_rmse_a = (float)sqrt(decay_error / decay.n);
    if (!isfinite(result->rise_rmse_a) || !isfinite(result->decay_rmse_a) ||
        (result->rise_rmse_a > LS_MAX_RMSE_A) ||
        (result->decay_rmse_a > LS_MAX_RMSE_A)) {
        result->l_ab_rise_uh = result->l_ab_decay_uh = 0.0f;
        result->ls_rise_uh = result->ls_decay_uh = 0.0f;
        return result->status = LS_FIT_POOR_QUALITY;
    }
    result->rise_valid = 1U;
    result->decay_valid = 1U;
    /* 两段各自残差较小，也可能对应互相矛盾的等效电路。 */
    if (fabs(l_rise_uh - l_decay_uh) / (0.5 * (l_rise_uh + l_decay_uh)) >
        LS_MAX_RELATIVE_L_DIFF)
        return result->status = LS_FIT_INCONSISTENT;
    return result->status = LS_FIT_OK;
}

#define LS_TIMER_HZ             168000000UL
#define LS_SAMPLE_PERIOD_US     2U
#define LS_SAMPLE_TICKS         336U
#define LS_PREPULSE_US          4U
#define LS_BOOTSTRAP_CHARGE_MS  2U
#define LS_PULSE_WIDTH_US       60U
#define LS_PULSE_HARD_LIMIT_US  60U
#define LS_CAPTURE_COUNT        256U
#define LS_CURRENT_LIMIT_A      1.50f
#define LS_VBUS_MIN_V           10.0f
#define LS_VBUS_MAX_V           18.0f
#define LS_TIMEOUT_MS           5U
#define LS_PHASE_CCER_MASK (TIM_CCER_CC1E | TIM_CCER_CC1NE | \
                            TIM_CCER_CC2E | TIM_CCER_CC2NE | \
                            TIM_CCER_CC3E | TIM_CCER_CC3NE)

typedef enum {
    LS_HW_IDLE = 0,
    LS_HW_RISE,
    LS_HW_DECAY,
    LS_HW_COMPLETE,
    LS_HW_FAULT,
    LS_HW_LOCKED
} LsHardwareState_t;

static volatile LsHardwareState_t ls_state;
static volatile uint8_t ls_dma_overrun;
static volatile uint8_t ls_overcurrent_detected;
static uint8_t ls_adc_restored;
static uint8_t ls_tim1_restored;
static volatile uint32_t ls_freewheel_start_us;
static uint32_t ls_start_tick_ms;
static float ls_vbus_v;
static float ls_rs_ohm;
static volatile uint8_t ls_pin_fault;
static ADC_InitTypeDef ls_saved_adc_init;
static uint32_t ls_saved_adc1_jsqr;
static uint32_t ls_saved_adc1_cfgr2;
static uint32_t ls_saved_adc_cfgr;
static uint32_t ls_saved_adc2_jsqr;
static uint32_t ls_saved_adc2_cfgr2;
static uint32_t ls_saved_adc_sqr1;
static uint32_t ls_saved_adc_smpr1;
static uint32_t ls_saved_adc_smpr2;
static uint32_t ls_saved_adc_tr1;
static uint32_t ls_saved_adc_ier;
static uint16_t ls_adc_raw[LS_CAPTURE_COUNT] __attribute__((aligned(4)));
static LsSample_t ls_samples[LS_CAPTURE_COUNT];

/* bit0=PA8/HIN1，bit1=PB13/LIN1，bit2=PB14/LIN2。 */
static uint8_t LsReadPwmPins(void)
{
    uint32_t pa = GPIOA->IDR;
    uint32_t pb = GPIOB->IDR;
    return (uint8_t)(((pa & GPIO_PIN_8) != 0U ? 1U : 0U) |
                     ((pb & GPIO_PIN_13) != 0U ? 2U : 0U) |
                     ((pb & GPIO_PIN_14) != 0U ? 4U : 0U));
}

static void LsPowerOff(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    __DSB();
    TIM1->CCER &= ~LS_PHASE_CCER_MASK;
    TIM1->DIER &= ~(TIM_DIER_UIE | TIM_DIER_CC3IE);
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM2->CR1 &= ~TIM_CR1_CEN;
}

static void LsRestore(void)
{
    ls_adc_restored = 1U;
    ls_tim1_restored = 0U;
    LsPowerOff();
    (void)HAL_ADC_Stop_DMA(&hadc2);
    __HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_AWD1);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_AWD1);
    hadc2.Init = ls_saved_adc_init;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) ls_adc_restored = 0U;
    ADC2->CFGR = ls_saved_adc_cfgr;
    ADC2->SQR1 = ls_saved_adc_sqr1;
    ADC2->SMPR1 = ls_saved_adc_smpr1;
    ADC2->SMPR2 = ls_saved_adc_smpr2;
    ADC2->TR1 = ls_saved_adc_tr1;
    ADC2->IER = ls_saved_adc_ier;
    /* 停止注入组可能清除 JSQR；HAL_ADC_Init 不会重建该寄存器，故需保存并恢复。 */
    ADC1->CFGR2 = ls_saved_adc1_cfgr2;
    ADC1->JSQR = ls_saved_adc1_jsqr;
    ADC2->CFGR2 = ls_saved_adc2_cfgr2;
    ADC2->JSQR = ls_saved_adc2_jsqr;
    MX_TIM1_Init();
    /* HAL 基础初始化不会清 OPM；Rs/FOC 需要定时器持续产生 PWM 周期。 */
    TIM1->CR1 &= ~(TIM_CR1_OPM | TIM_CR1_CEN);
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER &= ~LS_PHASE_CCER_MASK;
    ls_tim1_restored = ((TIM1->CR1 & TIM_CR1_OPM) == 0U) ? 1U : 0U;
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) ls_adc_restored = 0U;
    if (HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK) ls_adc_restored = 0U;
    if ((ADC1->JSQR != ls_saved_adc1_jsqr) ||
        (ADC2->JSQR != ls_saved_adc2_jsqr)) ls_adc_restored = 0U;
    __HAL_RCC_TIM2_CLK_DISABLE();
    if (ls_adc_restored && ls_tim1_restored) foc_motor_state = FOC_MOTOR_IDLE;
}

uint8_t MotorCalibration_LsIsActive(void)
{
    return (ls_state == LS_HW_RISE || ls_state == LS_HW_DECAY ||
            ls_state == LS_HW_COMPLETE || ls_state == LS_HW_FAULT ||
            ls_state == LS_HW_LOCKED) ? 1U : 0U;
}

HAL_StatusTypeDef MotorCalibration_LsStart(void)
{
    ADC_ChannelConfTypeDef channel = {0};
    ADC_AnalogWDGConfTypeDef watchdog = {0};
    uint32_t pulse_ticks = LS_PULSE_WIDTH_US * (LS_TIMER_HZ / 1000000UL);
    uint32_t prepulse_ticks = LS_PREPULSE_US * (LS_TIMER_HZ / 1000000UL);
    float adc_offset = foc.calibration.ib_offset / 4.0f;
    float adc_gain = foc.current.gain_b * 4.0f;
    float limit_codes = LS_CURRENT_LIMIT_A / adc_gain;

    if (MotorCalibration_LsIsActive() || MotorCalibration_IsActive())
        return HAL_BUSY;
    if (foc_motor_state != FOC_MOTOR_IDLE) {
        DebugConsole_Printf("LS_BLOCK=MOTOR_NOT_IDLE state=%u\r\n",
                            (unsigned int)foc_motor_state);
        return HAL_ERROR;
    }
    if (foc.calibration.calibrated == 0U) {
        DebugConsole_Printf("LS_BLOCK=ADC_OFFSET_NOT_READY\r\n");
        return HAL_ERROR;
    }
    if (!isfinite(foc.state.vbus) ||
        (foc.state.vbus < LS_VBUS_MIN_V) ||
        (foc.state.vbus > LS_VBUS_MAX_V)) {
        DebugConsole_Printf("LS_BLOCK=VBUS_RANGE VBUS=%.4f MIN=%.1f MAX=%.1f\r\n",
                            (double)foc.state.vbus,
                            (double)LS_VBUS_MIN_V, (double)LS_VBUS_MAX_V);
        return HAL_ERROR;
    }
    if ((HAL_RCC_GetPCLK2Freq() != LS_TIMER_HZ) ||
        (HAL_RCC_GetPCLK1Freq() != LS_TIMER_HZ)) {
        DebugConsole_Printf("LS_BLOCK=TIMER_CLOCK PCLK1=%lu PCLK2=%lu EXPECTED=%lu\r\n",
                            (unsigned long)HAL_RCC_GetPCLK1Freq(),
                            (unsigned long)HAL_RCC_GetPCLK2Freq(),
                            (unsigned long)LS_TIMER_HZ);
        return HAL_ERROR;
    }
    if (LS_PULSE_WIDTH_US > LS_PULSE_HARD_LIMIT_US) {
        DebugConsole_Printf("LS_BLOCK=PULSE_CONFIG\r\n");
        return HAL_ERROR;
    }
    if (!isfinite(adc_offset) || !isfinite(adc_gain) ||
        (adc_gain <= 0.0f) || !isfinite(limit_codes) ||
        (adc_offset <= limit_codes) ||
        (adc_offset + limit_codes >= 4095.0f)) {
        DebugConsole_Printf("LS_BLOCK=ADC_RANGE OFFSET=%.2f GAIN=%.8f LIMIT_CODES=%.2f\r\n",
                            (double)adc_offset, (double)adc_gain,
                            (double)limit_codes);
        return HAL_ERROR;
    }

    ls_rs_ohm = MotorCalibration_GetResultOhm();
    if (!isfinite(ls_rs_ohm) || ls_rs_ohm <= 0.0f) {
        DebugConsole_Printf("LS_BLOCK=RS_NOT_MEASURED RUN=rs_identify\r\n");
        return HAL_ERROR;
    }
    ls_vbus_v = foc.state.vbus;
    ls_saved_adc_init = hadc2.Init;
    ls_saved_adc1_jsqr = ADC1->JSQR;
    ls_saved_adc1_cfgr2 = ADC1->CFGR2;
    ls_saved_adc_cfgr = ADC2->CFGR;
    ls_saved_adc2_jsqr = ADC2->JSQR;
    ls_saved_adc2_cfgr2 = ADC2->CFGR2;
    ls_saved_adc_sqr1 = ADC2->SQR1;
    ls_saved_adc_smpr1 = ADC2->SMPR1;
    ls_saved_adc_smpr2 = ADC2->SMPR2;
    ls_saved_adc_tr1 = ADC2->TR1;
    ls_saved_adc_ier = ADC2->IER;
    ls_freewheel_start_us = 0U;
    ls_dma_overrun = 0U;
    ls_overcurrent_detected = 0U;
    ls_pin_fault = 0U;
    memset(ls_adc_raw, 0, sizeof(ls_adc_raw));
    foc_motor_state = FOC_MOTOR_CALIBRATION;

    /* 准备期间先关闭功率输出，停止原 25 kHz ADC 注入触发。 */
    LsPowerOff();
    TIM1->CCER &= ~TIM_CCER_CC4E;
    (void)HAL_ADCEx_InjectedStop_IT(&hadc1);
    (void)HAL_ADCEx_InjectedStop_IT(&hadc2);
    /* Rs 辨识可能留下单脉冲状态；复位 TIM1 后从确定的 GPIO/寄存器状态启动。 */
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM1_FORCE_RESET();
    __HAL_RCC_TIM1_RELEASE_RESET();
    MX_TIM1_Init();
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER &= ~(LS_PHASE_CCER_MASK | TIM_CCER_CC4E);

    /* ADC2_IN3 为 B/V 相运放输出；规则组单次 12 位，不启用 4 倍过采样。 */
    hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc2.Init.DMAContinuousRequests = ENABLE;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) goto fail;
    channel.Channel = ADC_CHANNEL_3;
    channel.Rank = ADC_REGULAR_RANK_1;
    channel.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    channel.SingleDiff = ADC_SINGLE_ENDED;
    channel.OffsetNumber = ADC_OFFSET_NONE;
    if (HAL_ADC_ConfigChannel(&hadc2, &channel) != HAL_OK) goto fail;
    watchdog.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
    watchdog.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
    watchdog.Channel = ADC_CHANNEL_3;
    watchdog.ITMode = ENABLE;
    watchdog.HighThreshold = (uint32_t)(adc_offset + limit_codes);
    watchdog.LowThreshold = (uint32_t)(adc_offset - limit_codes);
    watchdog.FilteringConfig = ADC_AWD_FILTERING_NONE;
    if (HAL_ADC_AnalogWDGConfig(&hadc2, &watchdog) != HAL_OK) goto fail;

    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->CR1 = 0U;
    TIM2->PSC = 0U;
    TIM2->ARR = LS_SAMPLE_TICKS - 1U;
    TIM2->CNT = 0U;
    TIM2->CR2 = TIM_TRGO_UPDATE;
    TIM2->SMCR = TIM_SLAVEMODE_TRIGGER | TIM_TS_ITR0; /* TIM1 TRGO → TIM2 ITR0。 */

    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ls_adc_raw,
                          LS_CAPTURE_COUNT) != HAL_OK) goto fail;

    /* OPM + PWM2：脉冲前 CH1N、CH2N 导通，为 U 高侧自举充电。
     * CCR1..ARR 期间 CH1 导通、CH1N 关闭；CH2/CH2N 均使能以保持 V 低侧导通。 */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CR1 = TIM_CR1_OPM | TIM_CR1_ARPE;
    TIM1->PSC = 0U;
    TIM1->RCR = 0U;
    /* 先关闭比较值预装载并指定模式，再写 CCR，避免沿用 FOC 的影子比较值。 */
    TIM1->CCMR1 = TIM_OCMODE_PWM2 | (TIM_OCMODE_PWM1 << 8);
    TIM1->CR2 = TIM_TRGO_OC1REF; /* CCPC=0，CCER/OCxM 立即生效。 */
    TIM1->ARR = prepulse_ticks + pulse_ticks - 1U;
    TIM1->CCR1 = prepulse_ticks;
    TIM1->CCR2 = 0U;
    /* CH3 输出保持禁用，仅在高侧脉冲中点产生比较中断。 */
    TIM1->CCR3 = prepulse_ticks + pulse_ticks / 2U;
    TIM1->CCR4 = 0U;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                 TIM_CCER_CC2E | TIM_CCER_CC2NE;
    TIM1->CNT = 0U;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR &= ~(TIM_SR_UIF | TIM_SR_CC3IF);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0U, 0U);
    HAL_NVIC_ClearPendingIRQ(TIM1_CC_IRQn);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
    TIM1->DIER = TIM_DIER_UIE | TIM_DIER_CC3IE;
    DebugConsole_Printf("LS_IDENTIFY STARTED LS_DIAG_REV=SINGLE_PULSE_ADC_V10\r\n");
    DebugConsole_Printf("VBUS=%.4f RS_USED=%.6f RS_SOURCE=%s ADC_SAMPLE_RATE=500000 ADC_SAMPLE_PERIOD_US=2\r\n",
                        (double)ls_vbus_v, (double)ls_rs_ohm,
                        "MEASURED");
    DebugConsole_Printf("PULSE_WIDTH_US=%u PULSE_HARD_LIMIT_US=%u CURRENT_LIMIT_A=%.2f CAPTURE_BUFFER_SIZE=%u BOOTSTRAP_CHARGE_MS=%u\r\n",
                        LS_PULSE_WIDTH_US, LS_PULSE_HARD_LIMIT_US,
                        (double)LS_CURRENT_LIMIT_A, LS_CAPTURE_COUNT,
                        LS_BOOTSTRAP_CHARGE_MS);
    ls_state = LS_HW_RISE;
    __DMB();
    TIM1->BDTR |= TIM_BDTR_MOE;
    HAL_Delay(LS_BOOTSTRAP_CHARGE_MS);
    uint8_t precharge_pins = LsReadPwmPins();
    if (precharge_pins != 6U) {
        ls_pin_fault = 1U;
        DebugConsole_Printf("LS_BLOCK=PWM_PRECHARGE_PINS PINS=%u EXPECTED=6\r\n",
                            precharge_pins);
        goto fail;
    }
    ls_start_tick_ms = HAL_GetTick();
    TIM1->CR1 |= TIM_CR1_CEN;

    return HAL_OK;

fail:
    LsRestore();
    ls_state = ls_adc_restored ? LS_HW_IDLE : LS_HW_LOCKED;
    DebugConsole_Printf("LS_STOP=%s ADC_RESTORED=%u TIM1_RESTORED=%u\r\n",
                        ls_pin_fault ? "PWM_PIN_FAULT" : "PERIPHERAL_SETUP",
                        ls_adc_restored, ls_tim1_restored);
    return HAL_ERROR;
}

void MotorCalibration_LsTim1Compare(void)
{
    if (ls_state != LS_HW_RISE) return;
    if (LsReadPwmPins() != 5U) {
        ls_pin_fault = 2U;
        ls_state = LS_HW_FAULT;
        LsPowerOff();
    }
}

void MotorCalibration_LsTim1Update(void)
{
    if (ls_state != LS_HW_RISE) return;
    /* 保留 CH1/CH1N 使能；清除 CC1E 会同时让 U 下管互补输出消失。 */
    TIM1->CCMR1 = (TIM1->CCMR1 & ~TIM_CCMR1_OC1M) |
                   TIM_OCMODE_FORCED_INACTIVE;
    /* 跳过高低侧硬件死区附近的边界样本。 */
    ls_freewheel_start_us = LS_PULSE_WIDTH_US + 1U;
    ls_state = LS_HW_DECAY;
}

void MotorCalibration_LsDmaHalf(void)
{
    if (ls_state != LS_HW_DECAY) return;
    /* 半传输约在 256 us 到达，用引脚读数确认 60 us 脉冲后两相下管续流。 */
    if (LsReadPwmPins() != 6U) {
        ls_pin_fault = 3U;
        ls_state = LS_HW_FAULT;
        LsPowerOff();
    }
}

void MotorCalibration_LsDmaComplete(void)
{
    if (ls_state != LS_HW_DECAY) {
        ls_dma_overrun = 1U;
        ls_state = LS_HW_FAULT;
    } else {
        ls_state = LS_HW_COMPLETE;
    }
    LsPowerOff();
}

void MotorCalibration_LsDmaError(void)
{
    ls_dma_overrun = 1U;
    ls_state = LS_HW_FAULT;
    LsPowerOff();
}

void MotorCalibration_LsOvercurrent(void)
{
    if (ls_state == LS_HW_RISE || ls_state == LS_HW_DECAY) {
        ls_overcurrent_detected = 1U;
        ls_state = LS_HW_FAULT;
        LsPowerOff();
    }
}

void MotorCalibration_LsProcess(void)
{
    LsHardwareState_t completed = ls_state;
    LsFitConfig_t config = {0};
    LsFitResult_t result;
    uint8_t overcurrent = ls_overcurrent_detected;

    if ((completed == LS_HW_RISE || completed == LS_HW_DECAY) &&
        (HAL_GetTick() - ls_start_tick_ms > LS_TIMEOUT_MS)) {
        LsPowerOff();
        ls_state = LS_HW_FAULT;
        completed = LS_HW_FAULT;
    }
    if (completed != LS_HW_COMPLETE && completed != LS_HW_FAULT) return;

    LsPowerOff();
    if (completed == LS_HW_COMPLETE) {
        for (uint32_t i = 0U; i < LS_CAPTURE_COUNT; i++) {
            uint32_t t_us = (i + 1U) * LS_SAMPLE_PERIOD_US;
            float current = ((float)ls_adc_raw[i] - foc.calibration.ib_offset / 4.0f) *
                            (foc.current.gain_b * 4.0f);
            if (fabsf(current) > LS_CURRENT_LIMIT_A) overcurrent = 1U;
            ls_samples[i].adc_raw = ls_adc_raw[i];
            ls_samples[i].timestamp = t_us;
            ls_samples[i].state = (t_us < LS_PULSE_WIDTH_US) ? LS_SAMPLE_RISE :
                (t_us >= ls_freewheel_start_us) ? LS_SAMPLE_DECAY : LS_SAMPLE_INVALID;
        }
    }
    LsRestore();
    ls_state = ls_adc_restored ? LS_HW_IDLE : LS_HW_LOCKED;

    config.timestamp_tick_us = 1.0f;
    config.pulse_start_timestamp = 0U;
    config.pulse_end_timestamp = LS_PULSE_WIDTH_US;
    config.freewheel_start_timestamp = ls_freewheel_start_us;
    config.adc_offset = foc.calibration.ib_offset / 4.0f;
    config.adc_gain_a_per_code = foc.current.gain_b * 4.0f;
    config.applied_voltage_v = ls_vbus_v; /* 近似模型，尚无绕组差分电压测量。 */
    config.loop_resistance_ohm = 2.0f * ls_rs_ohm;
    config.current_limit_a = LS_CURRENT_LIMIT_A;
    config.pulse_hard_limit_us = LS_PULSE_HARD_LIMIT_US;
    config.blanking_us = 2.0f;
    config.adc_overrun = ls_dma_overrun;
    if (completed == LS_HW_COMPLETE && !overcurrent)
        (void)MotorCalibration_LsFit(ls_samples, LS_CAPTURE_COUNT, &config, &result);
    else {
        memset(&result, 0, sizeof(result));
        result.status = overcurrent ? LS_FIT_OVERCURRENT :
                        ls_pin_fault ? LS_FIT_TIMING : LS_FIT_ADC_OVERRUN;
    }
    if (result.status == LS_FIT_OK) {
        DebugConsole_Printf("LS_RESULT RISE_UH=%.3f DECAY_UH=%.3f FIT_VALID=1 SAMPLES=%lu/%lu\r\n",
                            (double)result.ls_rise_uh, (double)result.ls_decay_uh,
                            (unsigned long)result.rise_samples,
                            (unsigned long)result.decay_samples);
    } else {
        DebugConsole_Printf("LS_RESULT RISE_UH=NA DECAY_UH=NA FIT_VALID=0 ABORT_REASON=%u\r\n",
                            (unsigned int)result.status);
    }
    DebugConsole_Printf("ADC_OVERRUN=%u OVERCURRENT_DETECTED=%u ABORT_REASON=%u TIM1_RESTORED=%u ADC_RESTORED=%u POWER_STAGE_OFF=1\r\n",
                        ls_dma_overrun, overcurrent, (unsigned int)result.status,
                        ls_tim1_restored, ls_adc_restored);
}
