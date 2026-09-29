

#include "motor_calibration.h"
#include "debug_console.h"
#include "foc_math.h"
#include "adc.h"
#include "tim.h"

#include "arm_math.h" /* 与工程的CMSIS-DSP数学接口保持统一。 */
#include <string.h>  /* Rs启动时清零当前辨识实例。 */

/* ======================== 共同使用：变量与引脚定义 ======================== */

volatile MotorCalibration_t motor_cal;

typedef enum {
    MOTOR_AH = 1, /* A相上管：PA8 / TIM1_CH1。 */
    MOTOR_AL,     /* A相下管：PB13 / TIM1_CH1N。 */
    MOTOR_BH,     /* B相上管：PA9 / TIM1_CH2。 */
    MOTOR_BL,     /* B相下管：PB14 / TIM1_CH2N。 */
    MOTOR_CH,     /* C相上管：PA10 / TIM1_CH3。 */
    MOTOR_CL      /* C相下管：PB15 / TIM1_CH3N。 */
} Motor_Pin_t;

typedef enum {
    MOTOR_GPIO_LOW = 0, /* GPIO推挽输出低电平，关闭对应MOS。 */
    MOTOR_GPIO_HIGH,    /* GPIO推挽输出高电平，静态导通对应MOS。 */
    MOTOR_PWM           /* TIM1复用输出，保留原通道极性与死区。 */
} Motor_Mode_t;

/* ======================== 共同使用：桥臂引脚控制 ======================== */

/* 按指定桥臂和模式配置单个GPIO；切GPIO前先撤销定时器输出许可。 */
static void Motor_PinMode(Motor_Pin_t pin, Motor_Mode_t mode)
{
    GPIO_InitTypeDef gpio = {0}; /* HAL GPIO初始化参数，局部栈变量。 */
    GPIO_TypeDef *port;         /* 当前半桥对应的GPIO端口。 */
    uint16_t pin_num;           /* GPIO引脚位掩码。 */
    uint32_t af;                /* TIM1对应的复用功能编号。 */
    uint32_t ccer;              /* TIM1通道/互补通道输出使能位。 */

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
        /* 先配置TIM1复用功能，再允许该通道向引脚输出。 */
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Alternate = af;
        HAL_GPIO_Init(port, &gpio);
        TIM1->CCER |= ccer;
    } else {
        /* 先断定时器输出，再预置GPIO电平，最后切为GPIO，避免切换毛刺。 */
        TIM1->CCER &= ~ccer;
        HAL_GPIO_WritePin(port, pin_num,
            (mode == MOTOR_GPIO_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        gpio.Mode = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(port, &gpio);
    }
}

/* ========================= Rs：电阻辨识专用 ========================= */

/* ISR仅写MOE/CCR；GPIO与HAL外设恢复由主循环完成。 */
static inline void Cal_RsQuickOff(void)
{
    __HAL_TIM_MOE_DISABLE(&htim1);
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0U;
}

/* Rs换相：先全关六路，再配置当前PWM与回流低侧。 */
static void Cal_ConfigPhase(void)
{
    __HAL_TIM_MOE_DISABLE(&htim1);
    Motor_PinMode(MOTOR_AH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_AL, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_BH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_BL, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_CH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_CL, MOTOR_GPIO_LOW);

    switch (motor_cal.phase) {
    case CAL_PHASE_AB:
        Motor_PinMode(MOTOR_AH, MOTOR_PWM);
        Motor_PinMode(MOTOR_BL, MOTOR_GPIO_HIGH);
        break;
    case CAL_PHASE_BC:
        Motor_PinMode(MOTOR_BH, MOTOR_PWM);
        Motor_PinMode(MOTOR_BL, MOTOR_PWM);
        Motor_PinMode(MOTOR_CL, MOTOR_GPIO_HIGH);
        break;
    case CAL_PHASE_CA:
        Motor_PinMode(MOTOR_CH, MOTOR_PWM);
        Motor_PinMode(MOTOR_CL, MOTOR_PWM);
        Motor_PinMode(MOTOR_AL, MOTOR_GPIO_HIGH);
        break;
    default:
        break;
    }
}

/* 将0~1的占空比换算为TIM1比较值；只更新当前Rs回路对应通道。 */
static void Cal_SetDuty(float duty)
{
    uint32_t ccr = (uint32_t)((float)TIM1->ARR * duty); /* 当前PWM比较计数。 */

    if (ccr < 1U)
        ccr = 1U;
    if (ccr >= TIM1->ARR)
        ccr = TIM1->ARR - 1U;

    switch (motor_cal.phase) {
    case CAL_PHASE_AB: TIM1->CCR1 = ccr; break;
    case CAL_PHASE_BC: TIM1->CCR2 = ccr; break;
    case CAL_PHASE_CA: TIM1->CCR3 = ccr; break;
    default: return;
    }
    __HAL_TIM_MOE_ENABLE(&htim1);
}

/* Rs测量时取当前两相电流绝对值的平均，避免采样极性影响拟合。 */
static float Cal_LineCurrent(float ia, float ib, float ic)
{
    switch (motor_cal.phase) {
    case CAL_PHASE_AB: return 0.5f * (__builtin_fabsf(ia) + __builtin_fabsf(ib));
    case CAL_PHASE_BC: return 0.5f * (__builtin_fabsf(ib) + __builtin_fabsf(ic));
    case CAL_PHASE_CA: return 0.5f * (__builtin_fabsf(ic) + __builtin_fabsf(ia));
    default: return 0.0f;
    }
}

/* 取最后 CAL_FIT_POINTS 点，用 V=R*I+b 的最小二乘斜率求线间电阻。
 * R=(n*ΣIV-ΣI*ΣV)/(n*ΣI²-(ΣI)²)；数据不足或电流变化过小时返回0。 */
static float Cal_FitResistance(void)
{
    uint16_t end = motor_cal.point_count; /* 当前回路已保存点数。 */
    uint16_t start = (end > CAL_FIT_POINTS) ? (end - CAL_FIT_POINTS) : 0U; /* 拟合起始点。 */
    uint16_t n = end - start; /* 实际用于最小二乘的数据点数量。 */

    float si = 0.0f;  /* 电流和ΣI。 */
    float sv = 0.0f;  /* 电压和ΣV。 */
    float sii = 0.0f; /* 电流平方和ΣI²。 */
    float siv = 0.0f; /* 电流电压乘积和Σ(I×V)。 */

    if (n < 2U)
        return 0.0f;

    for (uint16_t k = start; k < end; k++) {
        float i = motor_cal.point[k % CAL_FIT_POINTS].current; /* 环形缓存中对应档电流(A)。 */
        float v = motor_cal.point[k % CAL_FIT_POINTS].voltage; /* 对应档电压(V)。 */

        si += i;       // ΣI
        sv += v;       // ΣV
        sii += i * i;  // Σ(I²)
        siv += i * v;  // Σ(I*V)
    }

    float den = (float)n * sii - si * si; /* 最小二乘斜率分母。 */

    if (__builtin_fabsf(den) < 1e-6f)
        return 0.0f;

    return ((float)n * siv - si * sv) / den;
}

/* ========================= Ls：电感辨识专用 ========================= */

/* TIM3比较中断切GPIO，TIM2触发规则组ADC DMA。
 * 下列变量仅属于Ls辨识；中断负责安全时序，主循环负责拟合。 */
typedef enum {
    LS_IDLE = 0,  /* 空闲：未占用TIM2/TIM3和ADC规则组。 */
    LS_SAMPLING,  /* DMA正在采集电流，TIM3负责脉冲边沿。 */
    LS_READY,     /* DMA采满64点，等待主循环拟合。 */
    LS_FAILED     /* 已关断功率级，等待主循环恢复外设并打印原因。 */
} LsState_t;

static uint16_t ls_adc[LS_SAMPLE_COUNT]; /* 一次64点12位ADC原始数据，RAM中复用。 */
static CalPhase_t ls_phase;              /* 当前Ls回路：AB、BC或CA。 */
static ADC_HandleTypeDef *ls_hadc;       /* 选定规则组ADC：AB/BC为ADC2，CA为ADC1。 */
static DMA_Channel_TypeDef *ls_dma;      /* 对应DMA通道，用于读取剩余采样数量。 */
static uint32_t ls_high_pin;             /* 注入高侧的GPIO位掩码（PA8或PA10）。 */
static uint32_t ls_comp_pin;             /* 该高侧对应的低侧位掩码（PB13或PB15）。 */
static uint32_t ls_low_pin;              /* 回流低侧位掩码（PB13或PB14）。 */
static float ls_gain;                    /* ADC码值换算电流的增益，单位A/LSB。 */
static float ls_offset;                  /* 被采样相的零电流ADC原始码值。 */
static const char *ls_name;              /* 串口打印的回路名称：AB/BC/CA。 */

static volatile LsState_t ls_state;      /* 主循环和中断共同访问的Ls状态。 */
static volatile uint8_t ls_error;        /* 故障码：1过流、2 ADC/DMA、3超时、5时序。 */
static volatile uint8_t ls_update_seen;  /* TIM3的60us关断事件是否已处理。 */
static volatile uint8_t ls_compare_seen; /* TIM3的40us开启事件是否已处理。 */
static volatile uint8_t ls_armed;        /* 仅在正式启动后允许TIM3中断切桥臂。 */
static uint32_t ls_begin_cycles;         /* 正式启动时的DWT计数值，两个边沿共用。 */
static uint32_t ls_tim_ticks_us;        /* TIM2/3每微秒计数，用实际APB1定时器时钟计算。 */
static uint32_t ls_cpu_ticks_us;        /* DWT每微秒周期数，来自实际SystemCoreClock。 */
static uint32_t ls_tick;                 /* 启动时HAL毫秒tick，用于主循环超时保护。 */
static float ls_vbus;                    /* 本次脉冲前锁存母线电压，单位V。 */
static float ls_rline;                   /* 线间补偿电阻，统一取2×平均相电阻Rs。 */

/* ADC回调调用：CA辨识时ADC1规则组DMA归Ls使用，而非母线采样。 */
bool MotorCalibration_LsUsesADC1(void)
{
    return (ls_state != LS_IDLE) && (ls_phase == CAL_PHASE_CA);
}

/* 主循环调用：检验安全条件并启动指定回路的一次Ls脉冲采样。
 * 任何启动失败都通过Stop恢复外设，不在中断里拟合或打印。 */
bool MotorCalibration_LsStart(CalPhase_t phase)
{
    if ((phase > CAL_PHASE_CA) || ls_state || (foc_motor_state != FOC_MOTOR_IDLE) ||
        (foc.calibration.calibrated == 0U) ||
        (motor_cal.rs <= 0.0f) ||
        (foc.state.vbus < 5.0f) || (foc.state.vbus > 50.0f) ||
        (__HAL_RCC_TIM2_IS_CLK_ENABLED() || __HAL_RCC_TIM3_IS_CLK_ENABLED()))
        return false;

    /* APB1分频不为1时，TIM2/3时钟为2×PCLK1；DWT始终按CPU时钟计数。 */
    uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
        tim_clk *= 2U;
    if ((tim_clk % LS_SAMPLE_HZ) || (tim_clk % 1000000U) ||
        (SystemCoreClock % 1000000U) ||
        ((LS_DELAY_US + LS_PULSE_US) * (tim_clk / 1000000U) >= 65535U))
        return false;
    ls_tim_ticks_us = tim_clk / 1000000U;
    ls_cpu_ticks_us = SystemCoreClock / 1000000U;

    MotorCalibration_Stop(); /* Rs结果保留，正常FOC保持停机。 */
    ls_phase = phase;
    switch (phase) {
    case CAL_PHASE_AB: /* AH -> BL，测 IB。 */
        ls_name = "AB";
        ls_high_pin = GPIO_PIN_8;
        ls_comp_pin = GPIO_PIN_13;
        ls_low_pin = GPIO_PIN_14;
        break;
    case CAL_PHASE_BC: /* CH -> BL，测 IB。 */
        ls_name = "BC";
        ls_high_pin = GPIO_PIN_10;
        ls_comp_pin = GPIO_PIN_15;
        ls_low_pin = GPIO_PIN_14;
        break;
    case CAL_PHASE_CA: /* CH -> AL，测 IA。 */
        ls_name = "CA";
        ls_high_pin = GPIO_PIN_10;
        ls_comp_pin = GPIO_PIN_15;
        ls_low_pin = GPIO_PIN_13;
        break;
    default:
        return false;
    }
    ls_hadc = (phase == CAL_PHASE_CA) ? &hadc1 : &hadc2;
    ls_dma = (phase == CAL_PHASE_CA) ? DMA1_Channel3 : DMA1_Channel4;
    ls_offset = (phase == CAL_PHASE_CA) ? foc.calibration.ia_offset : foc.calibration.ib_offset;
    ls_gain = (phase == CAL_PHASE_CA) ? foc.current.gain_a : foc.current.gain_b;
    ls_rline = 2.0f * motor_cal.rs;
    ls_vbus = foc.state.vbus;
    /* CA暂停原母线/温度DMA，恢复时使用MX_ADC1_Init()标准配置。 */
    if (phase == CAL_PHASE_CA)
        ADC_Regular_PauseForLs();
    ls_error = 0U;
    ls_update_seen = ls_compare_seen = ls_armed = 0U;

    /* 在切ADC之前进入辨识状态，屏蔽原FOC注入回调对PWM寄存器的干扰。 */
    foc_motor_state = FOC_MOTOR_CALIBRATION;
    ls_state = LS_SAMPLING;

    if (HAL_ADCEx_InjectedStop_IT(ls_hadc) != HAL_OK) {
        ls_error = 2U;
        MotorCalibration_Stop();
        return false;
    }
    ADC_TypeDef *adc = ls_hadc->Instance; /* 本次独占ADC的寄存器基址。 */
    /* 规则组临时改为唯一通道IN3；正常配置由MX_ADCx_Init恢复。 */
    MODIFY_REG(adc->SQR1, ADC_SQR1_L | ADC_SQR1_SQ1,
               3U << ADC_SQR1_SQ1_Pos);
    MODIFY_REG(adc->SMPR1, ADC_SMPR1_SMP3, ADC_SMPR1_SMP3_1);
    float span = LS_HARD_CURRENT / ls_gain; /* 10A换算为ADC码值跨度。 */
    float lo = ls_offset - span;          /* 看门狗下限，防止负向过流。 */
    float hi = ls_offset + span;          /* 看门狗上限，防止正向过流。 */
    adc->TR1 = (((uint32_t)((hi > 4095.0f) ? 4095.0f : hi)) << 16) |
               (uint32_t)((lo < 0.0f) ? 0.0f : lo);
    /* TIM2 TRGO触发规则组单次转换，DMA接收，AWD1监测IN3。 */
    MODIFY_REG(adc->CFGR, ADC_CFGR_EXTSEL | ADC_CFGR_EXTEN | ADC_CFGR_DMACFG |
               ADC_CFGR_AWD1CH | ADC_CFGR_AWD1SGL | ADC_CFGR_AWD1EN,
               ADC_EXTERNALTRIG_T2_TRGO | ADC_CFGR_EXTEN_0 |
               ADC_CFGR_AWD1SGL | ADC_CFGR_AWD1EN |
               (3U << ADC_CFGR_AWD1CH_Pos));
    adc->ISR = ADC_ISR_AWD1 | ADC_ISR_OVR;
    adc->IER |= ADC_IER_AWD1IE; /* 软件保护与外部硬件过流保护并存。 */

    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->CR1 = 0U; TIM2->CR2 = 0U; TIM2->SMCR = 0U;
    TIM2->PSC = 0U;
    TIM2->ARR = ls_tim_ticks_us * 1000000U / LS_SAMPLE_HZ - 1U;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    TIM2->CR2 = TIM_CR2_MMS_1; /* 更新事件触发所选 ADC 规则组。 */

    /* TIM3 独立计时 GPIO 边沿；TIM1 保持 FOC 原始配置且 MOE 关闭。 */
    __HAL_TIM_MOE_DISABLE(&htim1);
    __HAL_RCC_TIM3_CLK_ENABLE();
    TIM3->CR1 = 0U;
    TIM3->DIER = 0U;
    TIM3->PSC = 0U;
    TIM3->ARR = 65535U;
    TIM3->CCMR1 = 0U;
    TIM3->CCER = 0U;
    TIM3->CCR1 = LS_DELAY_US * ls_tim_ticks_us; /* 40us开启事件。 */
    TIM3->CCR2 = (LS_DELAY_US + LS_PULSE_US) * ls_tim_ticks_us; /* 60us关断事件。 */
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0U;

    /* 先全关六路输出；只给本回路高侧对应的自举电容预充2ms。 */
    Motor_PinMode(MOTOR_AH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_AL, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_BH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_BL, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_CH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_CL, MOTOR_GPIO_LOW);
    GPIOB->BSRR = ls_comp_pin; /* 对应低侧导通，给自举电容充电。 */
    HAL_Delay(2U);
    GPIOB->BSRR = ls_comp_pin << 16; /* 预充完成，先关闭该低侧。 */
    GPIOB->BSRR = ls_low_pin; /* 回流低侧常导通：AB/BC为BL，CA为AL。 */

    /* 先武装64点DMA，避免启动GPIO脉冲后丢失最初采样。 */
    if (HAL_ADC_Start_DMA(ls_hadc, (uint32_t *)ls_adc, LS_SAMPLE_COUNT) != HAL_OK) {
        ls_error = 2U;
        MotorCalibration_Stop();
        return false;
    }
    ls_tick = HAL_GetTick();
    if (TIM3->CCR1 != LS_DELAY_US * ls_tim_ticks_us) {
        MotorCalibration_Stop();
        return false;
    }
    TIM3->SR = 0U;
    NVIC_ClearPendingIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 0U);
    NVIC_EnableIRQ(TIM3_IRQn);
    ls_begin_cycles = DWT->CYCCNT; /* DWT时基用于检测真实中断时刻。 */
    ls_armed = 1U; /* 开始允许TIM3中断切换功率管。 */
    TIM3->DIER = TIM_DIER_CC1IE | TIM_DIER_CC2IE;
    TIM2->CR1 |= TIM_CR1_CEN; /* 先启动ADC采样定时器。 */
    TIM3->CR1 |= TIM_CR1_CEN;
    return true;
}

/* TIM3中断：40us CC1先关对应低侧，再经1us死区开高侧；
 * 60us CC2先关高侧，确认采样索引/时序后再经死区开对应低侧续流。
 * ISR内只做寄存器/GPIO操作；任何异常立即调用故障关断。 */
void MotorCalibration_LsTimerIRQ(void)
{
    uint32_t flags = TIM3->SR & TIM3->DIER; /* 本次已使能的TIM3比较事件标志。 */
    if ((ls_state != LS_SAMPLING) || !ls_armed) {
        TIM3->DIER = 0U;
        TIM3->SR = 0U;
        return;
    }
    if (flags & TIM_SR_CC1IF) {
        TIM3->SR = ~TIM_SR_CC1IF;
        TIM3->DIER &= ~TIM_DIER_CC1IE;
        uint32_t elapsed = DWT->CYCCNT - ls_begin_cycles; /* CC1实际延时周期。 */
        if (!(TIM3->CR1 & TIM_CR1_CEN) || (TIM3->SR & TIM_SR_CC2IF) ||
            (elapsed < (LS_DELAY_US - 2U) * ls_cpu_ticks_us) ||
            (elapsed > (LS_DELAY_US + 2U) * ls_cpu_ticks_us)) {
            MotorCalibration_LsFault(5U);
            return;
        }
        GPIOB->BSRR = ls_comp_pin << 16; /* 必须先关对应低侧，防止桥臂直通。 */
        uint32_t dead = DWT->CYCCNT; /* 以CPU周期实现约1us换向死区。 */
        while ((uint32_t)(DWT->CYCCNT - dead) < ls_cpu_ticks_us) { }
        if (TIM3->SR & TIM_SR_CC2IF) {
            MotorCalibration_LsFault(5U);
            return;
        }
        GPIOA->BSRR = ls_high_pin; /* 时序正常才允许打开注入高侧。 */
        ls_compare_seen = 1U;
    }
    if (TIM3->SR & TIM_SR_CC2IF) {
        GPIOA->BSRR = ls_high_pin << 16; /* 先关闭待测回路的高侧。 */
        TIM3->CR1 &= ~TIM_CR1_CEN;
        ls_update_seen = 1U;
        uint32_t elapsed = DWT->CYCCNT - ls_begin_cycles; /* CC2实际延时周期。 */
        uint16_t index = (uint16_t)(LS_SAMPLE_COUNT - ls_dma->CNDTR); /* CC2时DMA采样数。 */
        TIM3->DIER = 0U;
        TIM3->SR = 0U;
        ls_armed = 0U;
        /* 两个事件必须顺序正确，且DWT和DMA时标均落入容差窗口。 */
        if (!ls_compare_seen ||
            (elapsed < (LS_DELAY_US + LS_PULSE_US - 3U) * ls_cpu_ticks_us) ||
            (elapsed > (LS_DELAY_US + LS_PULSE_US + 3U) * ls_cpu_ticks_us) ||
            (index < (LS_DELAY_US + LS_PULSE_US) *
                     LS_SAMPLE_HZ / 1000000U - 4U) ||
            (index > (LS_DELAY_US + LS_PULSE_US) *
                     LS_SAMPLE_HZ / 1000000U + 4U)) {
            MotorCalibration_LsFault(5U);
            return;
        }
        uint32_t dead = DWT->CYCCNT; /* 高侧关断后再次等待约1us死区。 */
        while ((uint32_t)(DWT->CYCCNT - dead) < ls_cpu_ticks_us) { }
        GPIOB->BSRR = ls_comp_pin; /* 高侧对应的低侧与回流低侧同步续流。 */
    }
}

/* ADC DMA采满一半约80us时调用；若未看到60us关断事件则立即判定时序故障。 */
void MotorCalibration_LsDmaHalf(void)
{
    if ((ls_state == LS_SAMPLING) && !ls_update_seen)
        MotorCalibration_LsFault(5U);
}

/* ADC DMA采满64点的完成回调：立即全关桥臂、停止两个定时器；
 * 仅将状态置为READY，耗时的浮点拟合留在主循环。 */
void MotorCalibration_LsDmaComplete(void)
{
    if (ls_state == LS_SAMPLING) {
        __HAL_TIM_MOE_DISABLE(&htim1);
        GPIOA->BSRR = ((uint32_t)GPIO_PIN_8 | GPIO_PIN_10) << 16;
        GPIOB->BSRR = ((uint32_t)GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15) << 16;
        TIM3->CR1 &= ~TIM_CR1_CEN;
        TIM3->DIER = 0U;
        ls_armed = 0U;
        TIM2->CR1 &= ~TIM_CR1_CEN;
        if (!ls_update_seen)
            ls_error = 5U; /* 60us 更新事件未执行，不能信任本次 Ls。 */
        ls_state = LS_READY;
    }
}

/* 任一中断或主循环可调用的Ls故障入口：先快速关断再记录故障信息。
 * reason约定为1过流、2 ADC/DMA异常、3软件超时、5边沿时序异常。 */
void MotorCalibration_LsFault(uint8_t reason)
{
    if (ls_state != LS_SAMPLING)
        return;
    __HAL_TIM_MOE_DISABLE(&htim1); /* 先断功率级，再等待主循环清理外设。 */
    GPIOA->BSRR = ((uint32_t)GPIO_PIN_8 | GPIO_PIN_10) << 16;
    GPIOB->BSRR = ((uint32_t)GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15) << 16;
    TIM3->DIER = 0U;
    ls_armed = 0U;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    ls_error = reason;
    ls_state = LS_FAILED;
}

/* 主循环处理一次采样：检查10ms超时、恢复外设、分别计算两沿Ls并输出。
 * 两沿有效时取线间电感均值再除2，保存到对应回路的RAM结果。 */
void MotorCalibration_LsProcess(void)
{
    switch (ls_state) {
    case LS_SAMPLING:
        if ((HAL_GetTick() - ls_tick) > 10U)
            MotorCalibration_LsFault(3U);
        return;
    case LS_READY:
    case LS_FAILED:
        break; /* 采样已结束，允许在主循环计算或输出故障。 */
    default:
        return;
    }

    uint8_t error = ls_error; /* 锁存本次故障码，Stop之后仍按原始原因报告。 */
    float rise_l = 0.0f;  /* 上升沿得到的线间电感(H)，0表示无效。 */
    float fall_l = 0.0f;  /* 下降沿得到的线间电感(H)，0表示无效。 */
    float rise_i0 = 0.0f, rise_i1 = 0.0f; /* 上升拟合窗口起止电流(A)。 */
    float fall_i0 = 0.0f, fall_i1 = 0.0f; /* 下降拟合窗口起止电流(A)。 */
    float peak = 0.0f; /* 本次采样的最大电流(A)。 */
    if ((ls_state == LS_READY) && !error) {
        const uint32_t on = LS_DELAY_US * LS_SAMPLE_HZ / 1000000U; /* 40us高侧开启对应采样序号。 */
        const uint32_t off = (LS_DELAY_US + LS_PULSE_US) * LS_SAMPLE_HZ / 1000000U; /* 60us关断序号。 */
        const uint32_t up0 = on + 1U, up1 = off - 2U; /* 上升沿去除头尾开关毛刺。 */
        const uint32_t down0 = off + 2U, down1 = LS_SAMPLE_COUNT - 2U; /* 下降沿去除换向和末端噪声。 */
        float offset = 0.0f;   /* 脉冲前ADC码值均值，修正本次电流零偏。 */
        float integral = 0.0f; /* 对电压积分得到电感分子(V·s)。 */

        /* 只用注入前的静态样本求零偏，避免电流上升污染偏置值。 */
        for (uint32_t k = 0U; k < on - 1U; k++)
            offset += (float)ls_adc[k];
        offset /= (float)(on - 1U);
        /* 沿用上升段的电流方向，避免下降末端噪声被 fabs 误判成回升。 */
        float direction = ((float)ls_adc[up1] >= offset) ? 1.0f : -1.0f; /* 实际电流符号。 */
        float scale = direction * ls_gain; /* ADC码差到正向回路电流的换算比例(A/LSB)。 */
        rise_i0 = ((float)ls_adc[up0] - offset) * scale;
        rise_i1 = ((float)ls_adc[up1] - offset) * scale;
        fall_i0 = ((float)ls_adc[down0] - offset) * scale;
        fall_i1 = ((float)ls_adc[down1] - offset) * scale;

        /* 扫描本次脉冲和续流段的峰值，剔除过流数据。 */
        for (uint32_t k = on; k < LS_SAMPLE_COUNT; k++) {
            float i = ((float)ls_adc[k] - offset) * scale; /* 当前点的回路电流(A)。 */
            if (i > peak) peak = i;
        }

        /* 上升段：按L·di/dt=Vbus-2Rs·i，使用梯形积分补偿电阻压降。 */
        for (uint32_t k = up0; k < up1; k++) {
            float i0 = ((float)ls_adc[k] - offset) * scale; /* 本采样点电流(A)。 */
            float i1 = ((float)ls_adc[k + 1U] - offset) * scale; /* 下一采样点电流(A)。 */
            integral += (ls_vbus - ls_rline * 0.5f * (i0 + i1)) /
                        (float)LS_SAMPLE_HZ;
        }
        if ((rise_i1 - rise_i0 > 0.03f) && (peak < LS_HARD_CURRENT))
            rise_l = integral / (rise_i1 - rise_i0);

        /* 高侧关断后两低侧续流，端电压近似零：L*di/dt=-2Rs*i。 */
        integral = 0.0f;
        for (uint32_t k = down0; k < down1; k++) {
            float i0 = ((float)ls_adc[k] - offset) * scale; /* 本采样点续流电流(A)。 */
            float i1 = ((float)ls_adc[k + 1U] - offset) * scale; /* 下一采样点续流电流(A)。 */
            integral += ls_rline * 0.5f * (i0 + i1) / (float)LS_SAMPLE_HZ;
        }
        if ((fall_i0 - fall_i1 > 0.03f) && (fall_i1 > 0.0f) &&
            (peak < LS_HARD_CURRENT))
            fall_l = integral / (fall_i0 - fall_i1);

        if (peak >= LS_HARD_CURRENT)
            error = 1U;
    }

    MotorCalibration_Stop(); /* 先安全关断，再输出结果。 */
    if (error) {
        DebugConsole_Printf("Ls %s ERROR %u (1=current 2=ADC/DMA 3=timeout 5=edge)\r\n",
                            ls_name, (unsigned)error);
        return;
    }

    /* 只保留上升、下降及综合值；任一拟合无效则不更新RAM结果。 */
    if (!(rise_l > 0.000001f && rise_l < 0.1f &&
          fall_l > 0.000001f && fall_l < 0.1f)) {
        DebugConsole_Printf("Ls %s invalid: rise=%.3f fall=%.3fuH\r\n",
                            ls_name, rise_l * 5e5f, fall_l * 5e5f);
        return;
    }
    float ls = (rise_l + fall_l) * 0.25f; /* 两个线电感的均值，再除2得到相电感。 */
    switch (ls_phase) {
    case CAL_PHASE_AB: motor_cal.ls_ab = ls; break;
    case CAL_PHASE_BC: motor_cal.ls_bc = ls; break;
    case CAL_PHASE_CA: motor_cal.ls_ca = ls; break;
    default: return;
    }
    DebugConsole_Printf("Ls %s: RISE=%.3f FALL=%.3f mean(rise,fall)=%.3fuH Ipeak=%.2fA\r\n",
                        ls_name, rise_l * 5e5f, fall_l * 5e5f, ls * 1e6f, peak);
}

/* ========================= Rs：启动与状态机 ========================= */

/* 启动完整Rs辨识：清空上一轮Rs/Ls RAM结果，从AB回路、初始占空比开始。 */
void MotorCalibration_Start(void)
{
    MotorCalibration_Stop(); /* 先关闭功率级并恢复外设，确保重新测量安全。 */
    memset((void *)&motor_cal, 0, sizeof(motor_cal)); /* 清零本轮状态及测量缓存。 */

    motor_cal.phase = CAL_PHASE_AB;
    motor_cal.duty = CAL_DUTY_START;
    Cal_ConfigPhase();
    motor_cal.state = CAL_SET_DUTY;
}

/* ========================= 共同使用：安全停止 ========================= */

/* 统一安全退出Rs/Ls：先关MOE和GPIO，再停Ls定时器/DMA并恢复ADC、TIM1。
 * 不清空motor_cal中的辨识结果；任何退出后功率级保持关闭。 */
void MotorCalibration_Stop(void)
{
    /* 先快速关闭功率输出，再停止全部PWM。此函数只由主循环调用。 */
    motor_cal.state = CAL_IDLE;
    __HAL_TIM_MOE_DISABLE(&htim1);
    ls_armed = 0U;
    FOC_PWM_Stop();

    /* 切回标准TIM1配置前，六路GPIO必须先全部拉低。 */
    Motor_PinMode(MOTOR_AH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_AL, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_BH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_BL, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_CH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_CL, MOTOR_GPIO_LOW);

    if (ls_state != LS_IDLE) {
        /* 先停TIM3脉冲和TIM2采样，再停止ADC/DMA并调用原CubeMX初始化。 */
        NVIC_DisableIRQ(TIM3_IRQn);
        TIM3->CR1 = TIM3->DIER = 0U;
        TIM3->SR = 0U;
        NVIC_ClearPendingIRQ(TIM3_IRQn);
        __HAL_RCC_TIM3_CLK_DISABLE();
        TIM2->CR1 = TIM2->SMCR = TIM2->CR2 = 0U;
        __HAL_RCC_TIM2_CLK_DISABLE();

        if (HAL_ADC_Stop_DMA(ls_hadc) != HAL_OK)
            Error_Handler(); /* 恢复失败时不允许进入无电流反馈的FOC。 */

        /* 仅恢复本次占用的ADC，不影响另一路正在使用的ADC和CAN/串口。 */
        if (ls_phase == CAL_PHASE_CA)
            MX_ADC1_Init();
        else
            MX_ADC2_Init();
        CLEAR_BIT(ls_hadc->Instance->CFGR, ADC_CFGR_AWD1EN); /* 解除Ls过流看门狗。 */
        CLEAR_BIT(ls_hadc->Instance->IER, ADC_IER_AWD1IE);
        ls_hadc->Instance->ISR = ADC_ISR_AWD1 | ADC_ISR_OVR;
    }

    /* 复用标准TIM1初始化，恢复中心对齐、PWM、死区、采样触发和引脚AF。 */
    MX_TIM1_Init();
    __HAL_TIM_MOE_DISABLE(&htim1);
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC4E);
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0U;

    if (ls_state != LS_IDLE) {
        if (HAL_ADCEx_InjectedStart_IT(ls_hadc) != HAL_OK)
            Error_Handler();
        ls_state = LS_IDLE; /* CA的常规DMA交由ADC_Regular_Service下轮重启。 */
    }
    foc_motor_state = FOC_MOTOR_IDLE;
}

/* ========================= Rs：采样、拟合与输出 ========================= */

/* ADC注入转换回调逐拍执行Rs状态机：过流立即停机；正常时逐档稳定、采样、拟合和换相.
 * ia/ib/ic为相电流(A)，vbus为当前母线电压(V)，不在中断里格式化串口文本。 */
void MotorCalibration_Run(float ia, float ib, float ic, float vbus)
{
    /* 中断仅处理活动状态，终态及换相由主循环完成。 */
    if ((motor_cal.state == CAL_IDLE) || (motor_cal.state >= CAL_PHASE_SWITCH))
        return;
    float current = Cal_LineCurrent(ia, ib, ic); /* 当前回路平均线电流(A)。 */

    if ((__builtin_fabsf(ia) >= CAL_HARD_CURRENT) ||
        (__builtin_fabsf(ib) >= CAL_HARD_CURRENT) ||
        (__builtin_fabsf(ic) >= CAL_HARD_CURRENT)) {
        Cal_RsQuickOff(); /* 先硬件关断，不在25kHz中断调用HAL。 */
        motor_cal.state = CAL_ERROR;
        return;
    }

    switch (motor_cal.state) {
    case CAL_IDLE:
    case CAL_PHASE_SWITCH:
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
        /* 等待电流过渡结束，此阶段不把瞬态采样计入电阻拟合。 */
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

        float avg_i = motor_cal.current_sum / (float)CAL_SAMPLE_COUNT; /* 本档平均线电流(A)。 */
        float avg_vbus = motor_cal.vbus_sum / (float)CAL_SAMPLE_COUNT; /* 本档平均母线电压(V)。 */

        if (motor_cal.point_count < CAL_MAX_POINTS) {
            volatile MotorCalPoint_t *p =
                &motor_cal.point[motor_cal.point_count % CAL_FIT_POINTS];
            motor_cal.point_count++; /* 只保留末尾5档，累计档数仍按原上限50。 */

            p->voltage = motor_cal.duty * avg_vbus;
            p->current = avg_i;

            if ((avg_i < CAL_TARGET_CURRENT) &&
                (motor_cal.duty + CAL_DUTY_STEP <= CAL_DUTY_MAX)) {
                motor_cal.duty += CAL_DUTY_STEP;
                motor_cal.state = CAL_SET_DUTY;
                break;
            }
        }

        Cal_RsQuickOff();

        CalPhase_t finished_phase = motor_cal.phase; /* 刚完成的线间回路编号。 */
        float line_r = Cal_FitResistance(); /* 当前回路最小二乘拟合线间电阻(Ω)。 */
        switch (finished_phase) {
        case CAL_PHASE_AB:
            motor_cal.r_ab = line_r;
            motor_cal.phase = CAL_PHASE_BC;
            break;
        case CAL_PHASE_BC:
            motor_cal.r_bc = line_r;
            motor_cal.phase = CAL_PHASE_CA;
            break;
        case CAL_PHASE_CA:
            motor_cal.r_ca = line_r;
            motor_cal.r_a = (motor_cal.r_ab + motor_cal.r_ca - motor_cal.r_bc) * 0.5f;
            motor_cal.r_b = (motor_cal.r_ab + motor_cal.r_bc - motor_cal.r_ca) * 0.5f;
            motor_cal.r_c = (motor_cal.r_bc + motor_cal.r_ca - motor_cal.r_ab) * 0.5f;
            motor_cal.rs = (motor_cal.r_a + motor_cal.r_b + motor_cal.r_c) / 3.0f;
            motor_cal.state = CAL_DONE; /* 主循环恢复外设，不在ADC ISR调用HAL。 */
            break;
        default:
            motor_cal.state = CAL_ERROR;
            return;
        }

        /* 每相结束提交线间电阻，主循环立即输出这一相的结果。 */
        DebugConsole_LogFromISR(DEBUG_LOG_RS_RESISTANCE,
                                (uint16_t)finished_phase, line_r, 0.0f, 0.0f);

        if (finished_phase == CAL_PHASE_CA)
            return;

        motor_cal.state = CAL_PHASE_SWITCH; /* 在主循环中进行HAL_GPIO换相。 */
        break;
    }

    default:
        Cal_RsQuickOff();
        motor_cal.state = CAL_ERROR;
        break;
    }
}

/* 主循环完成Rs换相与HAL恢复；ISR只负责采样及快速关断。 */
void MotorCalibration_DebugProcess(void)
{
    static MotorCalState_t handled = CAL_IDLE; /* 已执行恢复的终态。 */
    static MotorCalState_t printed = CAL_IDLE; /* 已打印的终态。 */
    MotorCalState_t state = motor_cal.state;

    if (state == CAL_PHASE_SWITCH) {
        Cal_ConfigPhase(); /* MOE已关闭，允许在主循环重新配置GPIO。 */
        motor_cal.duty = CAL_DUTY_START;
        motor_cal.point_count = 0U;
        motor_cal.state = CAL_SET_DUTY;
        return;
    }
    if ((state != CAL_DONE) && (state != CAL_ERROR)) {
        handled = printed = CAL_IDLE;
        return;
    }

    if (handled != state) {
        MotorCalibration_Stop(); /* HAL/GPIO等耗时操作不占用ADC中断。 */
        motor_cal.state = state; /* Stop会设为IDLE，恢复结果状态供主循环读取。 */
        handled = state;
    }
    if ((printed == state) || DebugConsole_LogPending())
        return;

    printed = state;
    switch (state) {
    case CAL_DONE:
        DebugConsole_Printf("\r\n===== Rs Calibration Result =====\r\n"
                            "R_A  = %.6f ohm\r\nR_B  = %.6f ohm\r\n"
                            "R_C  = %.6f ohm\r\nR_S  = %.6f ohm\r\n",
                            motor_cal.r_a, motor_cal.r_b,
                            motor_cal.r_c, motor_cal.rs);
        break;
    case CAL_ERROR:
        DebugConsole_Printf("\r\nRs calibration ERROR: duty=%.2f%%\r\n",
                            motor_cal.duty * 100.0f);
        break;
    default:
        break;
    }
}