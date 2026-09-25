#include "motor_calibration.h"

#include "adc.h"
#include "debug_console.h"
#include "tim.h"

#include <math.h>
#include <string.h>

/* 第一版只允许当前 168 MHz TIM1/TIM2 配置，周期均由定时器硬件产生。 */
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
static uint8_t ls_capture_valid;
static volatile uint8_t ls_precharge_pins;
static volatile uint8_t ls_midpulse_pins;
static volatile uint8_t ls_midpulse_seen;
static volatile uint8_t ls_update_seen;
static volatile uint8_t ls_decay_pins;
static volatile uint8_t ls_decay_seen;
static volatile uint8_t ls_pin_fault;
static volatile uint32_t ls_midpulse_cnt;
static volatile uint32_t ls_midpulse_ccer;
static volatile uint32_t ls_midpulse_bdtr;
static volatile uint32_t ls_midpulse_ccr1;
static volatile uint32_t ls_midpulse_ccmr1;
static volatile uint32_t ls_midpulse_cr2;
static volatile uint32_t ls_decay_ccer;
static volatile uint32_t ls_decay_bdtr;
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
    ls_capture_valid = 0U;
    ls_precharge_pins = 0U;
    ls_midpulse_pins = 0U;
    ls_midpulse_seen = 0U;
    ls_update_seen = 0U;
    ls_decay_pins = 0U;
    ls_decay_seen = 0U;
    ls_pin_fault = 0U;
    ls_midpulse_cnt = 0U;
    ls_midpulse_ccer = 0U;
    ls_midpulse_bdtr = 0U;
    ls_midpulse_ccr1 = 0U;
    ls_midpulse_ccmr1 = 0U;
    ls_midpulse_cr2 = 0U;
    ls_decay_ccer = 0U;
    ls_decay_bdtr = 0U;
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
    ls_precharge_pins = LsReadPwmPins();
    if (ls_precharge_pins != 6U) {
        ls_pin_fault = 1U;
        DebugConsole_Printf("LS_BLOCK=PWM_PRECHARGE_PINS PINS=%u EXPECTED=6\r\n",
                            ls_precharge_pins);
        DebugConsole_Printf("LS_PRECHARGE_REG BDTR=0x%08lX CCER=0x%08lX CCMR1=0x%08lX CR1=0x%08lX CR2=0x%08lX SR=0x%08lX GPIOA_MODER=0x%08lX GPIOB_MODER=0x%08lX\r\n",
                            (unsigned long)TIM1->BDTR,
                            (unsigned long)TIM1->CCER,
                            (unsigned long)TIM1->CCMR1,
                            (unsigned long)TIM1->CR1,
                            (unsigned long)TIM1->CR2,
                            (unsigned long)TIM1->SR,
                            (unsigned long)GPIOA->MODER,
                            (unsigned long)GPIOB->MODER);
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
    ls_midpulse_pins = LsReadPwmPins();
    ls_midpulse_cnt = TIM1->CNT;
    ls_midpulse_ccer = TIM1->CCER;
    ls_midpulse_bdtr = TIM1->BDTR;
    ls_midpulse_ccr1 = TIM1->CCR1;
    ls_midpulse_ccmr1 = TIM1->CCMR1;
    ls_midpulse_cr2 = TIM1->CR2;
    ls_midpulse_seen = 1U;
    if (ls_midpulse_pins != 5U) {
        ls_pin_fault = 2U;
        ls_state = LS_HW_FAULT;
        LsPowerOff();
    }
}

void MotorCalibration_LsTim1Update(void)
{
    if (ls_state != LS_HW_RISE) return;
    ls_update_seen = 1U;
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
    ls_decay_pins = LsReadPwmPins();
    ls_decay_ccer = TIM1->CCER;
    ls_decay_bdtr = TIM1->BDTR;
    ls_decay_seen = 1U;
    if (ls_decay_pins != 6U) {
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
        ls_capture_valid = 1U;
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
    uint16_t adc_min = 4095U, adc_max = 0U;

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
            if (ls_adc_raw[i] < adc_min) adc_min = ls_adc_raw[i];
            if (ls_adc_raw[i] > adc_max) adc_max = ls_adc_raw[i];
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
    DebugConsole_Printf("LS_IDENTIFY RESULT RISE_SAMPLES=%lu DECAY_SAMPLES=%lu\r\n",
                        (unsigned long)result.rise_samples,
                        (unsigned long)result.decay_samples);
    if (completed == LS_HW_COMPLETE) {
        DebugConsole_Printf("ADC_DIAG OFFSET=%.2f GAIN_A_PER_CODE=%.7f RAW_MIN=%u RAW_MAX=%u RAW0=%u RAW2=%u RAW6=%u RAW12=%u RAW20=%u FREEWHEEL_US=%lu\r\n",
                            (double)config.adc_offset,
                            (double)config.adc_gain_a_per_code,
                            adc_min, adc_max, ls_adc_raw[0], ls_adc_raw[2],
                            ls_adc_raw[6], ls_adc_raw[12], ls_adc_raw[20],
                            (unsigned long)ls_freewheel_start_us);
    }
    DebugConsole_Printf("TIM1_DIAG PRE_PINS=%u MID_PINS=%u MID_SEEN=%u UPDATE_SEEN=%u MID_CNT=%lu MID_CCER=0x%08lX MID_BDTR=0x%08lX PIN_BITS=PA8,PB13,PB14 EXPECTED_PRE=6 EXPECTED_MID=5\r\n",
                        ls_precharge_pins, ls_midpulse_pins, ls_midpulse_seen,
                        ls_update_seen, (unsigned long)ls_midpulse_cnt,
                        (unsigned long)ls_midpulse_ccer,
                        (unsigned long)ls_midpulse_bdtr);
    DebugConsole_Printf("TIM1_REG MID_CCR1=%lu MID_CCMR1=0x%08lX MID_CR2=0x%08lX PWM_PIN_FAULT=%u\r\n",
                        (unsigned long)ls_midpulse_ccr1,
                        (unsigned long)ls_midpulse_ccmr1,
                        (unsigned long)ls_midpulse_cr2, ls_pin_fault);
    DebugConsole_Printf("FREEWHEEL_DIAG PINS=%u SEEN=%u CCER=0x%08lX BDTR=0x%08lX EXPECTED_PINS=6\r\n",
                        ls_decay_pins, ls_decay_seen,
                        (unsigned long)ls_decay_ccer,
                        (unsigned long)ls_decay_bdtr);
    if (result.status == LS_FIT_INCONSISTENT) {
        DebugConsole_Printf("LS_FIT_DIAG RISE_CANDIDATE_UH=%.3f DECAY_CANDIDATE_UH=%.3f REASON=INCONSISTENT\r\n",
                            (double)result.ls_rise_uh, (double)result.ls_decay_uh);
    }
    if (result.status == LS_FIT_OK) {
        DebugConsole_Printf("TAU_RISE_US=%.3f TAU_DECAY_US=%.3f L_AB_RISE_UH=%.3f L_AB_DECAY_UH=%.3f\r\n",
                            (double)result.tau_rise_us, (double)result.tau_decay_us,
                            (double)result.l_ab_rise_uh, (double)result.l_ab_decay_uh);
        DebugConsole_Printf("LS_RISE_UH=%.3f LS_DECAY_UH=%.3f FIT_RMSE_RISE=%.5f FIT_RMSE_DECAY=%.5f FIT_VALID=1\r\n",
                            (double)result.ls_rise_uh, (double)result.ls_decay_uh,
                            (double)result.rise_rmse_a, (double)result.decay_rmse_a);
    } else {
        DebugConsole_Printf("TAU_RISE_US=NA TAU_DECAY_US=NA L_AB_RISE_UH=NA L_AB_DECAY_UH=NA\r\n");
        DebugConsole_Printf("LS_RISE_UH=NA LS_DECAY_UH=NA FIT_RMSE_RISE=NA FIT_RMSE_DECAY=NA FIT_VALID=0\r\n");
    }
    DebugConsole_Printf("ADC_OVERRUN=%u OVERCURRENT_DETECTED=%u ABORT_REASON=%u TIM1_RESTORED=%u ADC_RESTORED=%u POWER_STAGE_OFF=1\r\n",
                        ls_dma_overrun, overcurrent, (unsigned int)result.status,
                        ls_tim1_restored, ls_adc_restored);
}

void MotorCalibration_LsDump(void)
{
    if (MotorCalibration_LsIsActive()) {
        DebugConsole_Printf("ERR ls_identify dump: capture active\r\n");
        return;
    }
    if (!ls_capture_valid) {
        DebugConsole_Printf("ERR ls_identify dump: no completed capture\r\n");
        return;
    }
    DebugConsole_Printf("LS_ADC_DUMP COUNT=%u PERIOD_US=%u OFFSET=%.2f GAIN_A_PER_CODE=%.7f\r\n",
                        LS_CAPTURE_COUNT, LS_SAMPLE_PERIOD_US,
                        (double)(foc.calibration.ib_offset / 4.0f),
                        (double)(foc.current.gain_b * 4.0f));
    for (uint32_t i = 0U; i < LS_CAPTURE_COUNT; i += 8U) {
        DebugConsole_Printf("ADC[%03lu..%03lu]=%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                            (unsigned long)i, (unsigned long)(i + 7U),
                            ls_adc_raw[i], ls_adc_raw[i + 1U],
                            ls_adc_raw[i + 2U], ls_adc_raw[i + 3U],
                            ls_adc_raw[i + 4U], ls_adc_raw[i + 5U],
                            ls_adc_raw[i + 6U], ls_adc_raw[i + 7U]);
    }
}
