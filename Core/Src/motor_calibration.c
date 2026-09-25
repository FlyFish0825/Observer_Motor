#include "motor_calibration.h"

#include "debug_console.h"
#include "tim.h"
#include <math.h>
#include <stdint.h>

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
/* 定电流对照模式下平均电流相对目标值的允许误差比例。 */
#define MOTOR_CALIBRATION_TARGET_TOLERANCE        0.05f
/* 定电流对照模式首尾窗口允许的电流幅值均值差；单位 A。 */
#define MOTOR_CALIBRATION_STABLE_DELTA_A          0.002f
/* 默认定电压模式首尾窗口允许的电流幅值均值差；单位 A。 */
#define MOTOR_CALIBRATION_VOLTAGE_STABLE_DELTA_A  0.010f
/* 标定期间允许的最大 PWM 占空比，范围 0..1。 */
#define MOTOR_CALIBRATION_MAX_DUTY                0.20f
/* 保留的定电流对照模式离散积分系数，单位 duty/A。 */
#define MOTOR_CALIBRATION_CURRENT_KP              0.003f
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

/* 定电流对照模式的四档目标电流，单位 A；与定电压默认模式互斥。 */
/**
 * @brief 可选定电流对照模式的四档目标；按阶段索引访问，单位 A。
 */
static const float motor_calibration_stage_target_current[
    MOTOR_CALIBRATION_STAGE_COUNT] = {
    0.50f, 0.80f, 1.15f, 1.50f
};

/* 默认定电压模式的四档命令平均电压，单位 V，不代表实测绕组电压。 */
/**
 * @brief 默认定电压模式的四档命令目标；按阶段索引访问，单位 V。
 * @note 这是控制器目标值，不是绕组端实测 AB 差分电压。
 */
static const float rs_voltage_targets[MOTOR_CALIBRATION_STAGE_COUNT] = {
    0.65f, 0.95f, 1.25f, 1.55f
};

/** @brief 单一注入档的采样累计值、稳定性判定和故障标志。 */
typedef struct
{
    float sum_ib;                         /**< 正式采样有符号 Ib 累加值，单位 A。 */
    float sum_abs_ib;                     /**< 正式采样 |Ib| 累加值，单位 A。 */
    uint64_t sum_adc_b;                   /**< 正式采样 ADC_B 原始累加码值总和。 */
    uint64_t sum_ccr1;                    /**< 与正式 ADC 样本配对的 CCR1 总和。 */
    float sum_vbus;                       /**< 正式采样母线电压总和，单位 V。 */
    float sum_u_cmd;                      /**< Σ(已施加 Duty * 对应 Vbus)，单位 V。 */
    float sum_start_abs_ib;               /**< 首 250 个正式样本的 |Ib| 累加，单位 A。 */
    float sum_end_abs_ib;                 /**< 末 250 个正式样本的 |Ib| 累加，单位 A。 */
    float sum_start_duty;                 /**< 首窗口配对 Duty 总和，比例值。 */
    float sum_end_duty;                   /**< 末窗口配对 Duty 总和，比例值。 */
    float min_duty;                       /**< 正式采样期间最小已施加 Duty。 */
    float max_duty;                       /**< 正式采样期间最大已施加 Duty。 */
    float min_ib;                         /**< 正式采样期间最小有符号 Ib，单位 A。 */
    float max_ib;                         /**< 正式采样期间最大有符号 Ib，单位 A。 */
    float max_abs_ib_seen;                /**< 全阶段观测到的最大 |Ib|，单位 A。 */
    uint32_t sample_count;                /**< 正式采样有效样本数。 */
    uint32_t start_window_count;          /**< 首窗口已累计样本数。 */
    uint32_t end_window_count;            /**< 末窗口已累计样本数。 */
    uint32_t settle_start_tick_ms;        /**< 稳定等待计时起点，HAL tick 毫秒。 */
    uint32_t settle_elapsed_ms;           /**< 稳定等待实测耗时，单位 ms。 */
    uint32_t duty_target_consecutive_cycles; /**< 等待期间连续达到目标 Duty 的回调数。 */
    uint8_t settle_timing_valid;          /**< settle_elapsed_ms 是否来自完整计时。 */
    uint8_t target_reached;               /**< 定电流模式目标电流误差是否合格。 */
    uint8_t voltage_target_reached;       /**< 定电压模式平均命令电压是否合格。 */
    uint8_t duty_target_settled;          /**< 正式采样前 Duty 是否连续稳定到位。 */
    uint8_t current_stable;               /**< 首尾窗口电流差是否满足当前模式阈值。 */
    uint8_t duty_saturated;               /**< 阶段是否因 Duty 上限而饱和。 */
    uint8_t overcurrent_detected;         /**< 此阶段是否触发软件过流关断。 */
} MotorCalibrationStageData_t;

/** @brief 跨 ISR 与主循环共享的辨识运行上下文及最终拟合结果。 */
typedef struct
{
    volatile MotorCalibrationState_t state; /**< ISR 与主循环共享的当前状态。 */
    volatile uint8_t finish_pending;         /**< ISR 请求主循环完成关断/报告的标志。 */
    volatile MotorCalibrationState_t terminal_state; /**< 本次运行期望的终态。 */
    uint32_t cycle;                          /**< 当前阶段等待/采样总回调计数。 */
    uint8_t stage;                           /**< 当前阶段索引，范围 0..3。 */
    RsInjectionMode_t mode;                  /**< 当前注入模式，决定 Duty 控制路径。 */
    float duty;                              /**< 下一次写入 CCR1 的控制 Duty 比例。 */
    volatile uint32_t last_adc_step_tick_ms; /**< 最近一次 ADC 处理时间，HAL tick 毫秒。 */
    volatile uint32_t stage_start_tick_ms;   /**< 当前阶段起点，HAL tick 毫秒。 */

    /* ADC 回调在 PWM 顶部附近由 CH4 触发。回调写入的 CCR 先进入预装载寄存器，
     * 在随后中心对齐计数器 CNT=0 的更新事件才生效，并早于下一次 CH4 采样。
     * 因此此处保存的是下一样本所配对的命令，不能用本次回调刚算出的新命令
     * 回填当前样本的电压。 */
    float next_sample_duty;                  /**< 下一 ADC 样本应配对的量化 Duty。 */
    uint16_t next_sample_ccr1;               /**< 同一 PWM 命令的量化 CCR1 值。 */

    MotorCalibrationStageData_t stage_data[MOTOR_CALIBRATION_STAGE_COUNT]; /**< 四档独立累计数据。 */
    float r_ab_fit;                          /**< 四档拟合线间斜率，单位 Ω。 */
    float voltage_fit_intercept;             /**< 四档 U-I 拟合截距，单位 V。 */
    float rs_all;                            /**< 四档单相 Rs = r_ab_fit / 2，单位 Ω。 */
    float rs_2_4;                            /**< 第 2～4 档对照拟合的单相 Rs，单位 Ω。 */
    float ideal_r_ab_2_4;                    /**< 第 2～4 档拟合线间斜率，单位 Ω。 */
    float ideal_intercept_2_4;               /**< 第 2～4 档拟合截距，单位 V。 */
    float fit_all_current_span;              /**< 四档平均电流跨度，单位 A。 */
    float fit_2_4_current_span;              /**< 第 2～4 档平均电流跨度，单位 A。 */
    float fit_all_denominator;               /**< 四档最小二乘分母，用于退化诊断。 */
    float fit_2_4_denominator;               /**< 第 2～4 档拟合分母，用于退化诊断。 */
    uint8_t fit_all_valid;                   /**< 四档拟合是否数值有效。 */
    uint8_t fit_2_4_valid;                   /**< 第 2～4 档拟合是否数值有效。 */
    uint8_t all_stages_valid;                /**< 四档样本和阶段判据是否全部通过。 */
    uint8_t error_code;                      /**< 最近一次失败原因码；0 表示无错误。 */
} MotorCalibrationContext_t;

/* 模块唯一运行上下文；ISR 写入采样数据，主循环读取并完成报告。 */
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
 * @return 无；结果写入该阶段的 target_reached、voltage_target_reached 和
 *         current_stable 字段。
 */
static void MotorCalibration_CheckStage(uint8_t index)
{
    /* 下列量均由当前阶段样本计算；电流使用幅值，电压使用实际配对命令。 */
    MotorCalibrationStageData_t *data = &motor_calibration.stage_data[index]; /* 当前阶段累计记录。 */
    float target_i = motor_calibration_stage_target_current[index]; /* 电流对照目标，A。 */
    float target_u = rs_voltage_targets[index]; /* 默认模式命令电压目标，V。 */
    float average_i = MotorCalibration_Average(data->sum_abs_ib,
                                                data->sample_count); /* |Ib| 全窗均值，A。 */
    float average_u = MotorCalibration_Average(data->sum_u_cmd,
                                                data->sample_count); /* 命令电压全窗均值，V。 */
    float start_i = MotorCalibration_Average(data->sum_start_abs_ib,
                                              data->start_window_count); /* 首窗口 |Ib| 均值，A。 */
    float end_i = MotorCalibration_Average(data->sum_end_abs_ib,
                                            data->end_window_count); /* 末窗口 |Ib| 均值，A。 */
    float stable_delta_a = motor_calibration.mode == RS_INJECTION_VOLTAGE ?
        MOTOR_CALIBRATION_VOLTAGE_STABLE_DELTA_A :
        MOTOR_CALIBRATION_STABLE_DELTA_A; /* 当前模式采用的首尾差阈值，A。 */

    data->target_reached =
        (data->sample_count == MOTOR_CALIBRATION_SAMPLE_CYCLES) &&
        isfinite(average_i) &&
        (fabsf(average_i - target_i) <=
         target_i * MOTOR_CALIBRATION_TARGET_TOLERANCE);
    data->voltage_target_reached =
        (data->sample_count == MOTOR_CALIBRATION_SAMPLE_CYCLES) &&
        isfinite(average_u) &&
        (fabsf(average_u - target_u) <=
         target_u * MOTOR_CALIBRATION_TARGET_TOLERANCE);
    data->current_stable =
        (data->start_window_count == MOTOR_CALIBRATION_WINDOW_CYCLES) &&
        (data->end_window_count == MOTOR_CALIBRATION_WINDOW_CYCLES) &&
        isfinite(start_i) && isfinite(end_i) &&
        (fabsf(end_i - start_i) < stable_delta_a);
}

/**
 * @brief 对连续阶段的 (平均 |Ib|, 平均 U_CMD) 点做带截距最小二乘拟合。
 * @param first_stage 首档数组索引。
 * @param stage_count 纳入拟合的连续档数。
 * @param slope 输出线间电阻斜率，单位 Ω。
 * @param intercept 输出电压截距，单位 V。
 * @param current_span 输出参与拟合的最大与最小平均电流之差，单位 A。
 * @param fit_denominator 输出最小二乘分母，用于日志诊断。
 * @return 1 表示拟合数值有效；0 表示样本不足、电流跨度不足或拟合无效。
 * @note 输出指针必须有效；失败时所有输出先置为 NAN，斜率不是单相 Rs。
 */
static uint8_t MotorCalibration_Fit(uint8_t first_stage,
                                    uint8_t stage_count,
                                    float *slope,
                                    float *intercept,
                                    float *current_span,
                                    float *fit_denominator)
{
    /* 对每档平均点拟合 U=R_AB*I+b；截距 b 吸收固定电压偏置，
     * 最终单相参数由外层计算 Rs=R_AB/2。 */
    float sum_i = 0.0f;       /* ΣI，用于最小二乘正规方程。 */
    float sum_u = 0.0f;       /* ΣU，用于计算斜率和截距。 */
    float sum_ii = 0.0f;      /* Σ(I²)，用于拟合分母。 */
    float sum_iu = 0.0f;      /* Σ(IU)，用于拟合分子。 */
    float min_i = INFINITY;   /* 所选阶段平均电流最小值，A。 */
    float max_i = -INFINITY;  /* 所选阶段平均电流最大值，A。 */
    float denominator;        /* nΣ(I²)-(ΣI)²，判断样本是否可辨识。 */
    uint8_t offset;           /* 所选阶段区间内的零基循环索引。 */

    *slope = NAN;
    *intercept = NAN;
    *current_span = NAN;
    *fit_denominator = NAN;

    for (offset = 0U; offset < stage_count; ++offset)
    {
        uint8_t index = (uint8_t)(first_stage + offset); /* 当前档索引。 */
        const MotorCalibrationStageData_t *data =
            &motor_calibration.stage_data[index]; /* 当前档只读累计数据。 */
        float average_i; /* 当前档平均电流幅值，A。 */
        float average_u; /* 当前档平均理想命令电压，V。 */

        if (data->sample_count == 0U)
        {
            return 0U;
        }

        average_i = MotorCalibration_Average(
            data->sum_abs_ib, data->sample_count);
        average_u = MotorCalibration_Average(
            data->sum_u_cmd, data->sample_count);
        if ((!isfinite(average_i)) || (!isfinite(average_u)))
        {
            return 0U;
        }

        if (average_i < min_i)
        {
            min_i = average_i;
        }
        if (average_i > max_i)
        {
            max_i = average_i;
        }

        sum_i += average_i;
        sum_u += average_u;
        sum_ii += average_i * average_i;
        sum_iu += average_i * average_u;
    }

    *current_span = max_i - min_i;
    denominator = (float)stage_count * sum_ii - sum_i * sum_i;
    *fit_denominator = denominator;
    if ((!isfinite(*current_span)) || (!isfinite(denominator)) ||
        (*current_span <= MOTOR_CALIBRATION_MIN_CURRENT_SPREAD_A) ||
        (denominator <= MOTOR_CALIBRATION_FIT_DENOMINATOR_MIN))
    {
        return 0U;
    }
    if ((sum_i / (float)stage_count) < MOTOR_CALIBRATION_MIN_CURRENT_A)
    {
        return 0U;
    }

    *slope = ((float)stage_count * sum_iu - sum_i * sum_u) /
             denominator;
    *intercept = (sum_u - (*slope * sum_i)) / (float)stage_count;
    if ((!isfinite(*slope)) || (!isfinite(*intercept)) || (*slope <= 0.0f))
    {
        *slope = NAN;
        *intercept = NAN;
        return 0U;
    }

    return 1U;
}

/**
 * @brief 计算四档正式 Rs 和第 2～4 档对照拟合结果。
 * @return 无；拟合状态、线间斜率、截距、跨度和单相 Rs 写入全局上下文。
 * @note 所有斜率先按 AB 线间电阻计算，单相 Rs 再乘 0.5；拟合包含截距。
 */
static void MotorCalibration_ComputeFits(void)
{
    float slope_all;       /* 四档 AB 线间拟合斜率，Ω。 */
    float intercept_all;   /* 四档电压拟合截距，V。 */
    float span_all;        /* 四档平均电流跨度，A。 */
    float slope_2_4;       /* 第 2～4 档 AB 线间拟合斜率，Ω。 */
    float intercept_2_4;   /* 第 2～4 档拟合截距，V。 */
    float span_2_4;        /* 第 2～4 档平均电流跨度，A。 */

    /* 第一组拟合使用全部四档；第二组跳过最低电流档，仅作诊断对照。 */
    motor_calibration.fit_all_valid = MotorCalibration_Fit(
        0U, MOTOR_CALIBRATION_STAGE_COUNT,
        &slope_all, &intercept_all, &span_all,
        &motor_calibration.fit_all_denominator);
    motor_calibration.fit_2_4_valid = MotorCalibration_Fit(
        1U, MOTOR_CALIBRATION_STAGE_COUNT - 1U,
        &slope_2_4, &intercept_2_4, &span_2_4,
        &motor_calibration.fit_2_4_denominator);

    motor_calibration.r_ab_fit = slope_all;
    motor_calibration.voltage_fit_intercept = intercept_all;
    motor_calibration.fit_all_current_span = span_all;
    motor_calibration.fit_2_4_current_span = span_2_4;
    motor_calibration.rs_all = motor_calibration.fit_all_valid != 0U ?
                               (slope_all * 0.5f) : NAN;
    motor_calibration.rs_2_4 = motor_calibration.fit_2_4_valid != 0U ?
                               (slope_2_4 * 0.5f) : NAN;
    motor_calibration.ideal_r_ab_2_4 = slope_2_4;
    motor_calibration.ideal_intercept_2_4 = intercept_2_4;
}

/**
 * @brief 清空一次辨识的阶段累计、状态标志和拟合结果。
 * @return 无；全部累加器归零，未形成的数据和结果以 NAN 表示。
 */
static void MotorCalibration_ResetData(void)
{
    uint8_t index; /* 遍历四个辨识阶段。 */

    motor_calibration.cycle = 0U;
    motor_calibration.stage = 0U;
    motor_calibration.duty = 0.0f;
    motor_calibration.last_adc_step_tick_ms = 0U;
    motor_calibration.stage_start_tick_ms = 0U;
    motor_calibration.finish_pending = 0U;
    motor_calibration.next_sample_duty = 0.0f;
    motor_calibration.next_sample_ccr1 = 0U;
    motor_calibration.r_ab_fit = NAN;
    motor_calibration.voltage_fit_intercept = NAN;
    motor_calibration.rs_all = NAN;
    motor_calibration.rs_2_4 = NAN;
    motor_calibration.ideal_r_ab_2_4 = NAN;
    motor_calibration.ideal_intercept_2_4 = NAN;
    motor_calibration.fit_all_valid = 0U;
    motor_calibration.fit_2_4_valid = 0U;
    motor_calibration.all_stages_valid = 0U;
    motor_calibration.fit_all_current_span = NAN;
    motor_calibration.fit_2_4_current_span = NAN;
    motor_calibration.fit_all_denominator = NAN;
    motor_calibration.fit_2_4_denominator = NAN;
    motor_calibration.error_code = 0U;
    motor_calibration.terminal_state = MOTOR_CALIBRATION_ERROR;

    for (index = 0U; index < MOTOR_CALIBRATION_STAGE_COUNT; ++index)
    {
        MotorCalibrationStageData_t *data = &motor_calibration.stage_data[index]; /* 当前档待清零记录。 */
        data->sum_ib = 0.0f;
        data->sum_abs_ib = 0.0f;
        data->sum_adc_b = 0U;
        data->sum_ccr1 = 0U;
        data->sum_vbus = 0.0f;
        data->sum_u_cmd = 0.0f;
        data->sum_start_abs_ib = 0.0f;
        data->sum_end_abs_ib = 0.0f;
        data->sum_start_duty = 0.0f;
        data->sum_end_duty = 0.0f;
        data->min_duty = INFINITY;
        data->max_duty = -INFINITY;
        data->min_ib = INFINITY;
        data->max_ib = -INFINITY;
        data->max_abs_ib_seen = 0.0f;
        data->sample_count = 0U;
        data->start_window_count = 0U;
        data->end_window_count = 0U;
        data->settle_start_tick_ms = 0U;
        data->settle_elapsed_ms = 0U;
        data->duty_target_consecutive_cycles = 0U;
        data->settle_timing_valid = 0U;
        data->target_reached = 0U;
        data->voltage_target_reached = 0U;
        data->duty_target_settled = 0U;
        data->current_stable = 0U;
        data->duty_saturated = 0U;
        data->overcurrent_detected = 0U;
    }
}

/**
 * @brief 在辨识启动后由非 ISR 上下文打印参数、量程和硬件快照。
 * @return 无；只输出诊断文本，不修改电流控制、Duty 或拟合状态。
 */
static void MotorCalibration_PrintHeader(void)
{
    float pwm_frequency_hz; /* 按中心对齐周期计算的 PWM 频率，Hz。 */

    pwm_frequency_hz = (foc.timer.pwm_arr == 0U) ? NAN :
        ((float)foc.timer.clock_freq /
         (2.0f * (float)foc.timer.pwm_arr));

    DebugConsole_Printf("RS_IDENTIFY STARTED\r\n");
    DebugConsole_Printf("RS_DIAG_REV=VOLTAGE_INJECTION_V7\r\n");
    DebugConsole_Printf("INJECTION_MODE=%s DUTY_STEP=%.7f DUTY_SETTLE_CYCLES=%u\r\n",
        motor_calibration.mode == RS_INJECTION_VOLTAGE ? "VOLTAGE" : "CURRENT",
        (double)MOTOR_CALIBRATION_VOLTAGE_DUTY_STEP,
        (unsigned int)MOTOR_CALIBRATION_DUTY_SETTLE_CYCLES);
    DebugConsole_Printf("CURRENT_PI_ACTIVE=%s\r\n",
        motor_calibration.mode == RS_INJECTION_CURRENT ? "YES" : "NO");
    DebugConsole_Printf("CURRENT_STABLE_DELTA_LIMIT_A=%.6f\r\n",
        (double)(motor_calibration.mode == RS_INJECTION_VOLTAGE ?
            MOTOR_CALIBRATION_VOLTAGE_STABLE_DELTA_A :
            MOTOR_CALIBRATION_STABLE_DELTA_A));
    DebugConsole_Printf("RS_IDENTIFY_MAX_CURRENT=%.2f ADC_STALL_TIMEOUT_MS=%u STAGE_TIMEOUT_MS=%u\r\n",
        (double)RS_IDENTIFY_MAX_CURRENT_A,
        (unsigned int)RS_IDENTIFY_ADC_STALL_TIMEOUT_MS,
        (unsigned int)RS_IDENTIFY_STAGE_TIMEOUT_MS);
    DebugConsole_Printf("VOLTAGE_MODE_VBUS_RANGE=[1.0,%.1f] V\r\n",
        (double)RS_IDENTIFY_VOLTAGE_VBUS_MAX_V);
    DebugConsole_Printf("VOLTAGE_MODEL=IDEAL_COMMAND AB_DIFFERENTIAL_VERIFIED=NO\r\n");
    DebugConsole_Printf(
        "CURRENT_KP=%.6f SETTLE_CYCLES=%u SAMPLE_CYCLES=%u WINDOW_CYCLES=%u\r\n",
        (double)MOTOR_CALIBRATION_CURRENT_KP,
        (unsigned int)MOTOR_CALIBRATION_SETTLE_CYCLES,
        (unsigned int)MOTOR_CALIBRATION_SAMPLE_CYCLES,
        (unsigned int)MOTOR_CALIBRATION_WINDOW_CYCLES);
    DebugConsole_Printf(
        "PWM_FREQ=%.3f PWM_ARR=%lu DEADTIME_DTG=%u VBUS=%.6f TARGET_TOL=+/-%.1f%%\r\n",
        (double)pwm_frequency_hz,
        (unsigned long)foc.timer.pwm_arr,
        (unsigned int)foc.timer.dead_time,
        (double)foc.state.vbus,
        (double)(MOTOR_CALIBRATION_TARGET_TOLERANCE * 100.0f));
    DebugConsole_Printf("PWM_CLOCK_HZ=%lu TIM1_ARR_REG=%lu\r\n",
        (unsigned long)foc.timer.clock_freq,
        (unsigned long)TIM1->ARR);
    DebugConsole_Printf(
        "ADC_OFFSET_A=%.3f ADC_OFFSET_B=%.3f ADC_OFFSET_C=%.3f\r\n",
        (double)foc.calibration.ia_offset,
        (double)foc.calibration.ib_offset,
        (double)foc.calibration.ic_offset);
    DebugConsole_Printf(
        "ADC_GAIN_A=%.12g ADC_GAIN_B=%.12g ADC_GAIN_C=%.12g\r\n",
        (double)foc.current.gain_a,
        (double)foc.current.gain_b,
        (double)foc.current.gain_c);
}

/**
 * @brief 检查阶段是否收齐规定数量的正式样本。
 * @param data 待检查阶段的只读累计记录。
 * @return 1 表示样本数完整；0 表示不完整。
 */
static uint8_t MotorCalibration_StageSamplesValid(
    const MotorCalibrationStageData_t *data)
{
    return data->sample_count == MOTOR_CALIBRATION_SAMPLE_CYCLES;
}

/**
 * @brief 检查阶段样本、控制目标和电流稳定条件是否均通过。
 * @param data 待检查阶段的只读累计记录。
 * @return 1 表示阶段可用于结果；0 表示任一必要条件未通过。
 * @note 定电压模式检查电压目标和 Duty 稳定；定电流模式检查电流目标；
 *       两种模式都要求电流稳定。
 */
static uint8_t MotorCalibration_StageCurrentValid(
    const MotorCalibrationStageData_t *data)
{
    return (MotorCalibration_StageSamplesValid(data) != 0U) &&
           (((motor_calibration.mode == RS_INJECTION_VOLTAGE) &&
             (data->duty_target_settled != 0U) &&
             (data->voltage_target_reached != 0U)) ||
            ((motor_calibration.mode == RS_INJECTION_CURRENT) &&
             (data->target_reached != 0U))) &&
           (data->current_stable != 0U);
}

/**
 * @brief 判断 ADC 增益和零偏是否为有限值且数字码范围覆盖过流阈值。
 * @return 1 表示按当前软件换算仍有 ADC 数字余量；0 表示参数非法或余量不足。
 * @note 此项只检查换算后的 ADC 数字范围，不验证模拟前端线性度，也不替代硬件保护。
 */
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
static void MotorCalibration_PrintReport(void)
{
    uint8_t index; /* 当前输出阶段索引。 */

    for (index = 0U; index < MOTOR_CALIBRATION_STAGE_COUNT; ++index)
    {
        const MotorCalibrationStageData_t *data =
            &motor_calibration.stage_data[index]; /* 当前档只读数据源。 */
        /* 以下派生量只供日志展示，不回写辨识累计数据。 */
        float average_ib = MotorCalibration_Average(
            data->sum_ib, data->sample_count); /* 有符号 Ib 平均值，A。 */
        float average_abs_ib = MotorCalibration_Average(
            data->sum_abs_ib, data->sample_count); /* |Ib| 平均值，A。 */
        float average_adc_b = data->sample_count == 0U ? NAN :
            (float)data->sum_adc_b / (float)data->sample_count; /* ADC_B 平均原始码值。 */
        float average_ccr1 = data->sample_count == 0U ? NAN :
            (float)data->sum_ccr1 / (float)data->sample_count; /* 配对 CCR1 平均值。 */
        float average_duty = foc.timer.pwm_arr == 0U ? NAN :
            average_ccr1 / (float)(foc.timer.pwm_arr + 1U); /* CCR1 换算占空比。 */
        float average_vbus = MotorCalibration_Average(
            data->sum_vbus, data->sample_count); /* 母线平均值，V。 */
        float average_u = MotorCalibration_Average(
            data->sum_u_cmd, data->sample_count); /* 理想命令电压平均值，V。 */
        float average_start_i = MotorCalibration_Average(
            data->sum_start_abs_ib, data->start_window_count); /* 首窗口 |Ib| 均值，A。 */
        float average_end_i = MotorCalibration_Average(
            data->sum_end_abs_ib, data->end_window_count); /* 末窗口 |Ib| 均值，A。 */
        float average_start_duty = MotorCalibration_Average(
            data->sum_start_duty, data->start_window_count); /* 首窗口占空比均值。 */
        float average_end_duty = MotorCalibration_Average(
            data->sum_end_duty, data->end_window_count); /* 末窗口占空比均值。 */
        float measured_adc_rate_hz =
            (data->settle_timing_valid != 0U) &&
            (data->settle_elapsed_ms != 0U) ?
            (float)MOTOR_CALIBRATION_SETTLE_CYCLES * 1000.0f /
            (float)data->settle_elapsed_ms : NAN; /* 按稳定等待实测时长估算回调频率，Hz。 */
        float min_ib = data->sample_count == 0U ? NAN : data->min_ib; /* 最小有符号 Ib，A。 */
        float max_ib = data->sample_count == 0U ? NAN : data->max_ib; /* 最大有符号 Ib，A。 */
        float min_duty = data->sample_count == 0U ? NAN : data->min_duty; /* 最小已施加 Duty。 */
        float max_duty = data->sample_count == 0U ? NAN : data->max_duty; /* 最大已施加 Duty。 */

        DebugConsole_Printf("STAGE %u\r\n", (unsigned int)(index + 1U));
        if (motor_calibration.mode == RS_INJECTION_VOLTAGE)
        {
            DebugConsole_Printf("TARGET_U=%.6f\r\n",
                (double)rs_voltage_targets[index]);
        }
        else
        {
            DebugConsole_Printf("TARGET_I=%.6f\r\n",
                (double)motor_calibration_stage_target_current[index]);
        }
        DebugConsole_Printf("IB_AVG=%.6f FABS_IB_AVG=%.6f\r\n",
            (double)average_ib, (double)average_abs_ib);
        DebugConsole_Printf(
            "ADC_B_AVG=%.3f DUTY_AVG=%.8f CCR1_AVG=%.3f VBUS_AVG=%.6f U_CMD_AVG=%.6f U_IDEAL_AVG=%.6f\r\n",
            (double)average_adc_b, (double)average_duty,
            (double)average_ccr1, (double)average_vbus,
            (double)average_u, (double)average_u);
        DebugConsole_Printf("DUTY_MIN=%.8f DUTY_MAX=%.8f IB_MIN=%.6f IB_MAX=%.6f\r\n",
            (double)min_duty, (double)max_duty,
            (double)min_ib, (double)max_ib);
        DebugConsole_Printf(
            "I_START_AVG=%.6f I_END_AVG=%.6f DUTY_START_AVG=%.8f DUTY_END_AVG=%.8f\r\n",
            (double)average_start_i, (double)average_end_i,
            (double)average_start_duty, (double)average_end_duty);
        DebugConsole_Printf(
            "TARGET_REACHED=%s VOLTAGE_TARGET_REACHED=%s DUTY_TARGET_SETTLED=%s CURRENT_STABLE=%s STAGE_VALID=%s SAMPLES=%lu\r\n",
            motor_calibration.mode == RS_INJECTION_CURRENT ?
                (data->target_reached != 0U ? "YES" : "NO") : "NA",
            motor_calibration.mode == RS_INJECTION_VOLTAGE ?
                (data->voltage_target_reached != 0U ? "YES" : "NO") : "NA",
            motor_calibration.mode == RS_INJECTION_VOLTAGE ?
                (data->duty_target_settled != 0U ? "YES" : "NO") : "NA",
            data->current_stable != 0U ? "YES" : "NO",
            MotorCalibration_StageCurrentValid(data) != 0U ? "YES" : "NO",
            (unsigned long)data->sample_count);
        DebugConsole_Printf(
            "IB_ABS_PEAK_ADC=%.6f DUTY_SATURATED=%s OVERCURRENT_DETECTED=%s\r\n",
            (double)data->max_abs_ib_seen,
            data->duty_saturated != 0U ? "YES" : "NO",
            data->overcurrent_detected != 0U ? "YES" : "NO");
        DebugConsole_Printf(
            "SETTLE_MEASURED=%s SETTLE_MS=%lu ADC_STEP_HZ_EST=%.1f\r\n",
            data->settle_timing_valid != 0U ? "YES" : "NO",
            (unsigned long)data->settle_elapsed_ms,
            (double)measured_adc_rate_hz);
    }

    DebugConsole_Printf("DATA_VALID=%s RESULT_VALID=%s\r\n",
        motor_calibration.all_stages_valid != 0U ? "YES" : "NO",
        (motor_calibration.all_stages_valid != 0U &&
         motor_calibration.fit_all_valid != 0U) ? "YES" : "NO");
    DebugConsole_Printf(
        "FIT_ALL_VALID=%s FIT_2_4_VALID=%s FIT_SPAN_ALL=%.6f FIT_SPAN_2_4=%.6f\r\n",
        motor_calibration.fit_all_valid != 0U ? "YES" : "NO",
        motor_calibration.fit_2_4_valid != 0U ? "YES" : "NO",
        (double)motor_calibration.fit_all_current_span,
        (double)motor_calibration.fit_2_4_current_span);
    DebugConsole_Printf(
        "FIT_DENOM_ALL=%.9g FIT_DENOM_2_4=%.9g DENOM_MIN=%.9g\r\n",
        (double)motor_calibration.fit_all_denominator,
        (double)motor_calibration.fit_2_4_denominator,
        (double)MOTOR_CALIBRATION_FIT_DENOMINATOR_MIN);
    DebugConsole_Printf(
        "R_AB_FIT=%.6f VOLTAGE_FIT_INTERCEPT=%.6f RS_ALL=%.6f RS_2_4=%.6f\r\n",
        (double)motor_calibration.r_ab_fit,
        (double)motor_calibration.voltage_fit_intercept,
        (double)motor_calibration.rs_all,
        (double)motor_calibration.rs_2_4);
    DebugConsole_Printf("RS_IDENTIFY RESULT\r\n");
    DebugConsole_Printf(
        "RS_IDENTIFY_MAX_CURRENT=%.2f RS_IDENTIFY_ABORT_REASON=%s\r\n",
        (double)RS_IDENTIFY_MAX_CURRENT_A,
        MotorCalibration_AbortReason(motor_calibration.error_code));
    DebugConsole_Printf("RS_IDEAL_ALL=%.6f RS_IDEAL_2_4=%.6f\r\n",
        (double)motor_calibration.rs_all,
        (double)motor_calibration.rs_2_4);
    DebugConsole_Printf(
        "R_AB_IDEAL=%.6f R_AB_IDEAL_2_4=%.6f IDEAL_INTERCEPT=%.6f IDEAL_2_4_INTERCEPT=%.6f\r\n",
        (double)motor_calibration.r_ab_fit,
        (double)motor_calibration.ideal_r_ab_2_4,
        (double)motor_calibration.voltage_fit_intercept,
        (double)motor_calibration.ideal_intercept_2_4);
    DebugConsole_Printf(
        "VOLTAGE_MODEL=IDEAL_COMMAND AB_DIFFERENTIAL_VERIFIED=NO\r\n");
}


/**
 * @brief 校验运行条件并按 MOE 关闭、定时器同步、桥臂配置、最后开 MOE 的次序启动。
 * @param mode 注入模式枚举；定电压为默认，定电流只用于显式对照。
 * @return HAL_OK 表示已进入运行；HAL_BUSY 表示已有请求占用；HAL_ERROR 表示
 *         输入模式、FOC 状态、零偏、母线或 ADC 数字余量不符合要求。
 * @note 成功返回不代表辨识已完成；结果稍后由 MotorCalibration_Process 输出。
 */
HAL_StatusTypeDef MotorCalibration_StartMode(RsInjectionMode_t mode)
{
    HAL_StatusTypeDef status; /* HAL 定时器启动状态，传回调用方。 */
    uint32_t adc_trigger_mask = TIM_CCER_CC4E; /* CH4 ADC 触发输出位。 */
    uint32_t primask; /* 保存中断屏蔽状态，确保只恢复本函数改变的状态。 */

    if ((mode != RS_INJECTION_VOLTAGE) && (mode != RS_INJECTION_CURRENT))
    {
        return HAL_ERROR;
    }

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
        ((mode == RS_INJECTION_VOLTAGE) &&
         (foc.state.vbus > RS_IDENTIFY_VOLTAGE_VBUS_MAX_V)))
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
    motor_calibration.mode = mode;
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
 * @brief 使用默认定电压模式启动辨识。
 * @return 与 MotorCalibration_StartMode 相同的 HAL 状态码。
 */
HAL_StatusTypeDef MotorCalibration_Start(void)
{
    return MotorCalibration_StartMode(RS_INJECTION_VOLTAGE);
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
void MotorCalibration_AdcStep(uint16_t adc_b_raw)
{
    float signed_ia;       /* 当前采样换算的有符号 A 相电流，A。 */
    float signed_ib;       /* 当前采样换算的有符号 B 相电流，A。 */
    float signed_ic;       /* 当前采样换算的有符号 C 相电流，A。 */
    float current_a;       /* 电流幅值 |Ib|，供保护及控制/辨识使用，A。 */
    float target_a;        /* 定电流对照模式的当前档目标电流，A。 */
    float target_duty;     /* 定电压目标除以当前母线后的目标 Duty。 */
    float error_a;         /* 定电流对照模式目标与测量的差值，A。 */
    float sample_duty;     /* 与当前 ADC 样本对应的已施加 Duty。 */
    float sample_vbus;     /* 当前样本对应的母线电压，V。 */
    float sample_u_ideal;  /* 已施加 Duty * Vbus 的理想命令电压，V。 */
    MotorCalibrationStageData_t *data; /* 当前阶段的累计数据记录。 */
    uint16_t sample_ccr1;  /* 与样本配对的已施加 CCR1。 */
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
    if (current_a > data->max_abs_ib_seen)
    {
        data->max_abs_ib_seen = current_a;
    }
    if ((fabsf(signed_ia) > RS_IDENTIFY_MAX_CURRENT_A) ||
        (current_a > RS_IDENTIFY_MAX_CURRENT_A) ||
        (fabsf(signed_ic) > RS_IDENTIFY_MAX_CURRENT_A))
    {
        data->overcurrent_detected = 1U;
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
    sample_ccr1 = motor_calibration.next_sample_ccr1;
    sample_vbus = foc.state.vbus;
    if ((sample_vbus < 1.0f) ||
        ((motor_calibration.mode == RS_INJECTION_VOLTAGE) &&
         (sample_vbus > RS_IDENTIFY_VOLTAGE_VBUS_MAX_V)))
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

    if (motor_calibration.cycle == 0U)
    {
        data->settle_start_tick_ms = HAL_GetTick();
    }

    /* 定电压模式逐回调向目标 Duty 靠近；定电流模式才执行误差积分。 */
    if (motor_calibration.mode == RS_INJECTION_VOLTAGE)
    {
        target_duty = rs_voltage_targets[motor_calibration.stage] / sample_vbus;
        if ((!isfinite(target_duty)) || (target_duty < 0.0f))
        {
            MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 11U);
            return;
        }
        if (target_duty > MOTOR_CALIBRATION_MAX_DUTY)
        {
            data->duty_saturated = 1U;
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
    }
    else
    {
        target_a = motor_calibration_stage_target_current[motor_calibration.stage];
        error_a = target_a - current_a;
        /* 保留定电流对照积分器；只有显式选择该模式时才执行此分支。 */
        motor_calibration.duty += MOTOR_CALIBRATION_CURRENT_KP * error_a;
        if (motor_calibration.duty < 0.0f)
        {
            motor_calibration.duty = 0.0f;
        }
        if ((motor_calibration.duty >= MOTOR_CALIBRATION_MAX_DUTY) &&
            (error_a > 0.0f))
        {
            motor_calibration.duty = MOTOR_CALIBRATION_MAX_DUTY;
            data->duty_saturated = 1U;
            MotorCalibration_Finish(MOTOR_CALIBRATION_ERROR, 6U);
            return;
        }
    }

    next_ccr1 = (uint16_t)FOC_DutyToCCR(
        motor_calibration.duty, foc.timer.pwm_arr);
    TIM1->CCR1 = next_ccr1;
    /* 记录量化后的下一 PWM 命令，供下一次 ADC 样本正确配对。 */
    motor_calibration.next_sample_duty =
        (float)next_ccr1 / (float)(foc.timer.pwm_arr + 1U);
    motor_calibration.next_sample_ccr1 = next_ccr1;

    /* 等待阶段仅计数并确认 Duty 到位，不把爬升样本混入正式平均。 */
    if (motor_calibration.cycle < MOTOR_CALIBRATION_SETTLE_CYCLES)
    {
        if (motor_calibration.mode == RS_INJECTION_VOLTAGE)
        {
            /* 只统计本次采样之前已生效的 Duty 是否到达目标。 */
            if (fabsf(sample_duty - target_duty) <=
                MOTOR_CALIBRATION_DUTY_SETTLE_TOLERANCE)
            {
                data->duty_target_consecutive_cycles++;
            }
            else
            {
                data->duty_target_consecutive_cycles = 0U;
            }
        }
        motor_calibration.cycle++;
        return;
    }

    if ((motor_calibration.mode == RS_INJECTION_VOLTAGE) &&
        (data->sample_count == 0U))
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

    if (data->sample_count == 0U)
    {
        data->settle_elapsed_ms = HAL_GetTick() -
                                  data->settle_start_tick_ms;
        data->settle_timing_valid = 1U;
    }

    {
        /* 首尾 250 点为不重叠窗口，用于判断正式采样期间电流是否仍漂移。 */
        if (data->sample_count < MOTOR_CALIBRATION_WINDOW_CYCLES)
        {
            data->sum_start_abs_ib += current_a;
            data->sum_start_duty += sample_duty;
            data->start_window_count++;
        }
        if (data->sample_count >=
            MOTOR_CALIBRATION_SAMPLE_CYCLES -
            MOTOR_CALIBRATION_WINDOW_CYCLES)
        {
            data->sum_end_abs_ib += current_a;
            data->sum_end_duty += sample_duty;
            data->end_window_count++;
        }
        data->sum_ib += signed_ib;
        data->sum_abs_ib += current_a;
        data->sum_adc_b += (uint64_t)adc_b_raw;
        if (sample_duty < data->min_duty)
        {
            data->min_duty = sample_duty;
        }
        if (sample_duty > data->max_duty)
        {
            data->max_duty = sample_duty;
        }
        data->sum_ccr1 += (uint64_t)sample_ccr1;
        data->sum_vbus += sample_vbus;
        data->sum_u_cmd += sample_u_ideal;
        if (data->sample_count == 0U)
        {
            data->min_ib = signed_ib;
            data->max_ib = signed_ib;
        }
        else
        {
            if (signed_ib < data->min_ib)
            {
                data->min_ib = signed_ib;
            }
            if (signed_ib > data->max_ib)
            {
                data->max_ib = signed_ib;
            }
        }
        data->sample_count++;
    }

    motor_calibration.cycle++;
    if (motor_calibration.stage_data[motor_calibration.stage].sample_count <
        MOTOR_CALIBRATION_SAMPLE_CYCLES)
    {
        return;
    }

    if (motor_calibration.mode == RS_INJECTION_VOLTAGE)
    {
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
    uint8_t index; /* 完成时遍历全部阶段的索引。 */

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
        motor_calibration.all_stages_valid = 1U;
        for (index = 0U; index < MOTOR_CALIBRATION_STAGE_COUNT; ++index)
        {
            MotorCalibration_CheckStage(index);
            if (MotorCalibration_StageCurrentValid(
                    &motor_calibration.stage_data[index]) == 0U)
            {
                motor_calibration.all_stages_valid = 0U;
            }
        }

        /* 所有输出均已关闭后再验证每档数据，并计算完整与后三档拟合。 */
        MotorCalibration_ComputeFits();
        if (motor_calibration.fit_all_valid == 0U)
        {
            terminal_state = MOTOR_CALIBRATION_ERROR;
            motor_calibration.error_code = 3U;
        }
        else if (motor_calibration.all_stages_valid == 0U)
        {
            terminal_state = MOTOR_CALIBRATION_ERROR;
            motor_calibration.error_code = 5U;
        }
        motor_calibration.terminal_state = terminal_state;
    }

    /* 不恢复保存的 CCER；辨识结束后所有相功率输出都保持关闭。 */
    foc_motor_state = FOC_MOTOR_IDLE;
    motor_calibration.state = terminal_state;
    __DMB();

    MotorCalibration_PrintReport();
    if (terminal_state == MOTOR_CALIBRATION_DONE)
    {
        DebugConsole_Printf("RS_IDENTIFY DONE Rs=%.6f ohm\r\n",
                            (double)motor_calibration.rs_all);
    }
    else
    {
        DebugConsole_Printf("RS_IDENTIFY ERROR code=%u\r\n",
                            (unsigned int)motor_calibration.error_code);
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
