




/**
 * @file motor_calibration.c
 * @brief 电机线间电阻 Rs 与线间脉冲电感 Ls 的辨识实现。
 *
 * 【模块分工】
 *  1. 共用：六路桥臂GPIO模式切换、全关与外设安全恢复。
 *  2. Rs：TIM1 PWM 注入，ADC 注入回调以固定节奏等待和累计；
 *     主循环处理每档均值、末五点最小二乘以及 AB→BC→CA 换相。
 *  3. Ls：TIM2 触发规则组 ADC DMA（400 kHz、64点），TIM3 控制
 *     40/60 us GPIO 边沿；主循环对上升/续流下降两段分别积分拟合。
 *
 * 【跨上下文约定】
 *  中断只做采样、边沿操作、标志发布和紧急关断；HAL初始化、
 *  浮点拟合、串口格式化和下一回路启动均放在主循环。
 *  Ls 的 LS_READY/LS_FAILED 与 Rs 的 CAL_READY 是“交给主循环处理”的信号。
 *
 * 【安全约定】
 *  使用外部硬件过流保护的同时，固件仍执行采样电流和时序检查；
 *  停止时先撤销功率输出，再恢复 ADC/TIM1；恢复不代表自动开启 PWM。
 *  此文件只保存辨识结果到 RAM，不在这里写入 Flash。
 */

#include "motor_calibration.h"
#include "debug_console.h"
#include "foc_math.h"
#include "adc.h"
#include "tim.h"

#include "arm_math.h" /* 与工程的CMSIS-DSP数学接口保持统一。 */
#include <string.h>  /* Rs启动时清零当前辨识实例。 */

/* ========================================================================== */
/* 一、共用数据和功率桥臂编号                                                */
/* ========================================================================== */

/* 唯一对外辨识实例：Rs/Ls共用phase和结果存储；中断与主循环通过state交接。 */
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

/* ========================================================================== */
/* 二、共用 GPIO 配置（只在主循环/初始化路径使用）                             */
/* ========================================================================== */

/**
 * @brief 将某一路 MOS 引脚配置成低电平、高电平或 TIM1 PWM 复用。
 * 步骤1：按六路桥臂编号查找 GPIO 端口/引脚、AF 编号与 TIM1 CCER 许可位。
 * 步骤2：PWM 模式先配置 AF，再打开对应 CCER 输出许可。
 * 步骤3：GPIO 模式先撤销 CCER，再预置输出电平，最后改为推挽输出，降低切换毛刺。
 * 注意：此函数会调用 HAL_GPIO_Init，不能用于 TIM3 微秒级换向中断。
 * @note 调用环境：Rs换相、Ls启动与Stop的主循环路径
 * @param pin 待切换的 AH/AL/BH/BL/CH/CL 桥臂。
 * @param mode 目标为 GPIO 低、GPIO 高或 TIM1 复用 PWM。
 */
static void Motor_PinMode(Motor_Pin_t pin, Motor_Mode_t mode)
{
    GPIO_InitTypeDef gpio = {0}; /* HAL GPIO初始化参数，局部栈变量。 */
    GPIO_TypeDef *port;         /* 当前半桥对应的GPIO端口。 */
    uint16_t pin_num;           /* GPIO引脚位掩码。 */
    uint32_t af;                /* TIM1对应的复用功能编号。 */
    uint32_t ccer;              /* TIM1通道/互补通道输出使能位。 */

    /* 2.1 将逻辑半桥编号映射到实际引脚及定时器输出使能位。 */
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

    /* 2.2 两种模式共用引脚号、无上下拉和高速输出配置。 */
    gpio.Pin = pin_num;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    /* 2.3 按模式选择切换顺序：复用先配AF，GPIO先断TIM1许可。 */
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

/**
 * @brief 将六路桥臂依次切为 GPIO 低电平。
 * 供 Rs 换相、Ls 自举预充前和统一 Stop 复用，消除三处相同的引脚初始化序列。
 * 仅用于允许 HAL 的普通执行路径；ISR 紧急关断仍直接写 GPIO BSRR。
 * @note 调用环境：主循环
 */
static void Cal_AllPinsLow(void)
{
    Motor_PinMode(MOTOR_AH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_AL, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_BH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_BL, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_CH, MOTOR_GPIO_LOW);
    Motor_PinMode(MOTOR_CL, MOTOR_GPIO_LOW);
}

/* ========================================================================== */
/* 三、Rs：桥臂与拟合的内部辅助函数                                           */
/* ========================================================================== */

/**
 * @brief Rs 快速停止 TIM1 PWM 注入。
 * 先撤销 MOE，再清零三相比较值；ADC 中断中不调用阻塞式 HAL。
 * 此处不执行 GPIO 模式恢复：静态回流低侧和外设清理由主循环后续处理。
 * @note 调用环境：ADC注入中断；也可从普通流程调用
 */
static inline void Cal_RsQuickOff(void)
{
    __HAL_TIM_MOE_DISABLE(&htim1);
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0U;
}

/**
 * @brief 配置当前 Rs 线间回路的桥臂组合。
 * 先禁止 MOE 并将六路引脚全部拉低，再按 motor_cal.phase 选择注入/回流管。
 * AB：AH PWM、BL 静态高；BC：BH/BL PWM、CL 静态高；
 * CA：CH/CL PWM、AL 静态高。占空比及真正开启 MOE 在 Cal_SetDuty 中完成。
 * @note 调用环境：主循环；不得在25 kHz ADC注入中断中调用
 */
static void Cal_ConfigPhase(void)
{
    __HAL_TIM_MOE_DISABLE(&htim1);
    Cal_AllPinsLow();

    /* 3.1 按线间回路设置功率通路；非参与桥臂保持低电平。 */
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

/**
 * @brief 将本档 duty 转换为 TIM1 比较值，并开启对应回路 PWM。
 * 先将 duty×ARR 限定在 [1, ARR-1]，避免比较值为0或越过周期。
 * 仅写本回路对应 CCR；最后打开 MOE。下一档的稳定等待从主循环置 CAL_SETTLE 开始。
 * @note 调用环境：Rs主循环的短临界区；仅直接寄存器操作
 * @param duty 当前档位占空比，正常由 Rs 状态机控制。
 */
static void Cal_SetDuty(float duty)
{
    uint32_t ccr = (uint32_t)((float)TIM1->ARR * duty); /* 当前PWM比较计数。 */

    /* 3.2 防止零比较值和满计数比较值形成边界行为。 */
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

/**
 * @brief 从三相瞬时电流中提取当前 Rs 回路的线电流幅值。
 * 使用参与回路的两相电流绝对值平均，避免电流采样极性影响正向 V-I 拟合。
 * 此函数不采样 ADC，只处理 ADC 注入回调传来的 Ia/Ib/Ic。
 * @note 调用环境：25 kHz ADC注入中断
 * @return 当前回路的线电流幅值，单位 A。
 */
static float Cal_LineCurrent(float ia, float ib, float ic)
{
    switch (motor_cal.phase) {
    case CAL_PHASE_AB: return 0.5f * (__builtin_fabsf(ia) + __builtin_fabsf(ib));
    case CAL_PHASE_BC: return 0.5f * (__builtin_fabsf(ib) + __builtin_fabsf(ic));
    case CAL_PHASE_CA: return 0.5f * (__builtin_fabsf(ic) + __builtin_fabsf(ia));
    default: return 0.0f;
    }
}

/**
 * @brief 通过当前回路最近最多五个 V-I 均值点计算线间电阻。
 * 模型：V = R×I + b；用最小二乘的斜率 R 表示线间电阻。
 * 步骤1：仅选取环形缓存最后 CAL_FIT_POINTS 个点。
 * 步骤2：累计 ΣI、ΣV、ΣI²、Σ(I×V)。
 * 步骤3：按 R=(nΣIV-ΣIΣV)/(nΣI²-(ΣI)²) 计算。
 * 若少于两点或分母过小，返回0；不在该函数内进行相间换算。
 * @note 调用环境：Rs主循环 CAL_READY 后
 * @return 当前线间电阻，单位 Ω；无有效拟合时返回0。
 */
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

    /* 3.3 按真实档号取模访问五点环形缓存，较老的档位已被覆盖。 */
    for (uint16_t k = start; k < end; k++) {
        float i = motor_cal.point[k % CAL_FIT_POINTS].current; /* 环形缓存中对应档电流(A)。 */
        float v = motor_cal.point[k % CAL_FIT_POINTS].voltage; /* 对应档电压(V)。 */

        si += i;       // ΣI
        sv += v;       // ΣV
        sii += i * i;  // Σ(I²)
        siv += i * v;  // Σ(I*V)
    }

    /* 3.4 分母对应电流点的离散程度；接近0时斜率会不稳定。 */
    float den = (float)n * sii - si * si; /* 最小二乘斜率分母。 */

    if (__builtin_fabsf(den) < 1e-6f)
        return 0.0f;

    return ((float)n * siv - si * sv) / den;
}

/* ========================================================================== */
/* 四、Ls：采样资源、时序状态和测量快照                                      */
/* ========================================================================== */

/* TIM2 的 TRGO 触发 ADC 规则组，DMA存放64点原始码；TIM3 CC1/CC2只管理
 * GPIO脉冲边沿。LS_READY/FAILED 交给主循环处理，LS_IDLE 表示资源已恢复。 */
typedef enum {
    LS_IDLE = 0,  /* 空闲：未占用TIM2/TIM3和ADC规则组。 */
    LS_SAMPLING,  /* DMA正在采集电流，TIM3负责脉冲边沿。 */
    LS_READY,     /* DMA采满64点，等待主循环拟合。 */
    LS_FAILED     /* 已关断功率级，等待主循环恢复外设并打印原因。 */
} LsState_t;

/* 4.1 一次脉冲的数据及预先选好的GPIO掩码；高优先级中断不重复查引脚映射。 */
static uint16_t ls_adc[LS_SAMPLE_COUNT]; /* 一次64点12位ADC原始数据，RAM中复用。 */
static uint32_t ls_high_pin;             /* 注入高侧的GPIO位掩码（PA8或PA10）。 */
static uint32_t ls_comp_pin;             /* 该高侧对应的低侧位掩码（PB13或PB15）。 */
static float ls_gain;                    /* ADC码值换算电流的增益，单位A/LSB。 */

/* 4.2 ISR与主循环共享的控制标志；volatile用于观察异步更新，不替代关中断临界区。 */
static volatile LsState_t ls_state;      /* 主循环和中断共同访问的Ls状态。 */
static volatile uint8_t ls_auto;       /* 主循环/按键中断共用；IDLE+自动标志表示待启动下一组。 */
static volatile uint8_t ls_error;        /* 故障码：1过流、2 ADC/DMA、3超时、5时序。 */
static volatile uint8_t ls_update_seen;  /* TIM3的60us关断事件是否已处理。 */
static volatile uint8_t ls_compare_seen; /* TIM3的40us开启事件是否已处理。 */
static volatile uint8_t ls_armed;        /* 仅在正式启动后允许TIM3中断切桥臂。 */
/* 4.3 两套不同时间基准：TIM2/3计数与CPU DWT周期不能混为一谈。 */
static uint32_t ls_begin_cycles;         /* 正式启动时的DWT计数值，两个边沿共用。 */
static uint32_t ls_tim_ticks_us;        /* TIM2/3每微秒计数，用实际APB1定时器时钟计算。 */
static uint32_t ls_cpu_ticks_us;        /* DWT每微秒周期数，来自实际SystemCoreClock。 */
static uint32_t ls_tick;                 /* 启动时HAL毫秒tick，用于主循环超时保护。 */
/* 4.4 采样前锁存电气参数；CA占用ADC1期间不依赖母线DMA持续更新。 */
static float ls_vbus;                    /* 本次脉冲前锁存母线电压，单位V。 */
static float ls_rline;                   /* 线间补偿电阻，统一取2×平均相电阻Rs。 */

/**
 * @brief 判断 ADC1 当前的规则组 DMA 数据是否归 Ls 使用。
 * 只有 Ls 尚未恢复且本回路为 CA 时才返回 true；其他时候 ADC1 数据按正常母线/温度规则组处理。
 * 采用 Rs/Ls 共用的 motor_cal.phase，不再维护第二个 ls_phase。
 * @note 调用环境：ADC回调，要求判定路径简短
 * @return true 表示 ADC1 规则组当前由 CA 电感辨识占用。
 */
bool MotorCalibration_LsUsesADC1(void)
{
    return (ls_state != LS_IDLE) && (motor_cal.phase == CAL_PHASE_CA);
}

/* CAN等主循环调用者必须同时考虑Ls自动换相间隙：此时ls_state为IDLE但ls_auto仍为1。 */
bool MotorCalibration_LsBusy(void)
{
    return (ls_state != LS_IDLE) || (ls_auto != 0U);
}

/* LsFault记录中断故障；主循环双沿拟合无效记6，0表示正常或主动停止。 */
uint8_t MotorCalibration_LsLastError(void)
{
    return ls_error;
}

/**
 * @brief 配置并启动指定 AB/BC/CA 回路的一次 Ls 测量。
 * 步骤1：检查待机、ADC校零、Rs、母线范围及 TIM2/TIM3占用情况。
 * 步骤2：计算实际时钟刻度；Stop恢复原外设并选定回路与电流通道。
 * 步骤3：暂停本次 ADC 注入组，临时配置规则组 IN3、模拟看门狗和 TIM2 TRGO。
 * 步骤4：配置 TIM3 比较点，六路拉低并预充对应自举电容。
 * 步骤5：先启动64点DMA，再在短临界区内核对状态并启动TIM2/TIM3。
 * 预检失败直接返回 false；开始配置后若失败则执行统一Stop恢复。
 * 本函数不拟合 Ls，DMA完成后交给主循环。
 * @note 调用环境：主循环；内部允许HAL调用和2 ms预充延时
 * @param phase 要测试的线间回路。
 * @return true 表示采样已武装，false 表示拒绝或启动失败。
 */
bool MotorCalibration_LsStart(CalPhase_t phase)
{
    /* 4.5【步骤1：启动资格】禁止运动状态、未经校零、无有效Rs或母线异常下通电。
     * 还要排除TIM2/TIM3已经被其他功能使用的情况。 */
    if ((phase > CAL_PHASE_CA) || ls_state || (foc_motor_state != FOC_MOTOR_IDLE) ||
        (foc.calibration.calibrated == 0U) ||
        (motor_cal.rs <= 0.0f) ||
        (foc.state.vbus < 5.0f) || (foc.state.vbus > 50.0f) ||
        (__HAL_RCC_TIM2_IS_CLK_ENABLED() || __HAL_RCC_TIM3_IS_CLK_ENABLED()))
        return false;

    /* APB1分频不为1时，TIM2/3时钟为2×PCLK1；DWT始终按CPU时钟计数。 */
    /* 4.6【步骤2：时钟核算】APB1预分频影响定时器倍频；要求采样率和us刻度可整除。
     * DWT按SystemCoreClock计数；两种刻度分别记录，不能互相代替。 */
    uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
        tim_clk *= 2U;
    if ((tim_clk % LS_SAMPLE_HZ) || (tim_clk % 1000000U) ||
        (SystemCoreClock % 1000000U) ||
        ((LS_DELAY_US + LS_PULSE_US) * (tim_clk / 1000000U) >= 65535U))
        return false;
    ls_tim_ticks_us = tim_clk / 1000000U;
    ls_cpu_ticks_us = SystemCoreClock / 1000000U;

    /* 4.7【步骤3：清场与回路映射】先恢复标准外设并保持关断，再设置当前Ls回路。
     * BC/CA共用CH高侧及CL互补低侧；CA回流到AL，AB/BC回流到BL。 */
    MotorCalibration_Stop(); /* Rs结果保留，正常FOC保持停机。 */
    motor_cal.phase = phase; /* 与Rs共用AB/BC/CA，不再维护第二份回路状态。 */
    switch (phase) {
    case CAL_PHASE_AB: /* AH -> BL，测 IB。 */
        ls_high_pin = GPIO_PIN_8;
        ls_comp_pin = GPIO_PIN_13;
        break;
    case CAL_PHASE_BC: /* CH -> BL，测 IB。 */
    case CAL_PHASE_CA: /* CH -> AL，测 IA。两组高侧与互补低侧相同。 */
        ls_high_pin = GPIO_PIN_10;
        ls_comp_pin = GPIO_PIN_15;
        break;
    default:
        return false;
    }
    /* 4.8【步骤4：采样来源与参数锁存】AB/BC测IB(ADC2)，CA测IA(ADC1)。
     * 过流看门狗的初始偏置用已有校零值，拟合会再由脉冲前样本修正零偏。 */
    ADC_HandleTypeDef *hadc = (phase == CAL_PHASE_CA) ? &hadc1 : &hadc2;
    float offset = (phase == CAL_PHASE_CA) ? foc.calibration.ia_offset : foc.calibration.ib_offset;
    ls_gain = (phase == CAL_PHASE_CA) ? foc.current.gain_a : foc.current.gain_b;
    ls_rline = 2.0f * motor_cal.rs;
    ls_vbus = foc.state.vbus;
    /* CA只暂停ADC1母线/温度轮询；不能连带停止B相的ADC2注入组。
     * 暂停失败时仍未配置脉冲或导通GPIO，不允许继续辨识。 */
    if ((phase == CAL_PHASE_CA) && (ADC_Regular_PauseForLs() != HAL_OK)) {
        ls_error = 2U;
        MotorCalibration_Stop();
        return false;
    }
    ls_error = 0U;
    ls_update_seen = ls_compare_seen = ls_armed = 0U;

    /* 在切ADC之前进入辨识状态，屏蔽原FOC注入回调对PWM寄存器的干扰。 */
    foc_motor_state = FOC_MOTOR_CALIBRATION;
    ls_state = LS_SAMPLING;

    /* 4.9【步骤5：临时ADC模式】停所选ADC的注入中断，规则组改为IN3单通道。
     * AWD1直接监视ADC原始码，阈值由校零值±10A对应码值设置。 */
    if (HAL_ADCEx_InjectedStop_IT(hadc) != HAL_OK) {
        ls_error = 2U;
        MotorCalibration_Stop();
        return false;
    }
    ADC_TypeDef *adc = hadc->Instance; /* 本次独占ADC的寄存器基址。 */
    /* 规则组临时改为唯一通道IN3；正常配置由MX_ADCx_Init恢复。 */
    MODIFY_REG(adc->SQR1, ADC_SQR1_L | ADC_SQR1_SQ1,
               3U << ADC_SQR1_SQ1_Pos);
    MODIFY_REG(adc->SMPR1, ADC_SMPR1_SMP3, ADC_SMPR1_SMP3_1);
    float span = LS_HARD_CURRENT / ls_gain; /* 10A换算为ADC码值跨度。 */
    float lo = offset - span;          /* 看门狗下限，防止负向过流。 */
    float hi = offset + span;          /* 看门狗上限，防止正向过流。 */
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

    /* 4.10【步骤6：TIM2采样节拍】ARR换算为400 kHz；TRGO更新事件触发ADC。
     * TIM2只负责采样，不负责GPIO高低侧的通断。 */
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->CR1 = 0U; TIM2->CR2 = 0U; TIM2->SMCR = 0U;
    TIM2->PSC = 0U;
    TIM2->ARR = ls_tim_ticks_us * 1000000U / LS_SAMPLE_HZ - 1U;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    TIM2->CR2 = TIM_CR2_MMS_1; /* 更新事件触发所选 ADC 规则组。 */

    /* TIM3 独立计时 GPIO 边沿；TIM1 保持 FOC 原始配置且 MOE 关闭。 */
    /* 4.11【步骤7：TIM3脉冲计划】CC1=40us、CC2=60us，事件由TIM3 IRQ实现。
     * 在脉冲期间TIM1保持关闭，禁止与正常FOC PWM混用。 */
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
    /* 4.12【步骤8：预充与回流路径】先六路全关，导通本高侧的对应低侧2ms预充。
     * 预充后先关该低侧并检查取消请求，然后打开当前回路的回流低侧。 */
    Cal_AllPinsLow();
    GPIOB->BSRR = ls_comp_pin; /* 对应低侧导通，给自举电容充电。 */
    HAL_Delay(2U);
    GPIOB->BSRR = ls_comp_pin << 16; /* 预充完成，先关闭该低侧。 */
    if (ls_state != LS_SAMPLING) { /* 预充期间若按键取消，不再导通回流低侧。 */
        MotorCalibration_Stop();
        return false;
    }
    GPIOB->BSRR = (phase == CAL_PHASE_CA) ? GPIO_PIN_13 : GPIO_PIN_14; /* 回流低侧：CA为AL，其余为BL。 */

    /* 先武装64点DMA，避免启动GPIO脉冲后丢失最初采样。 */
    /* 4.13【步骤9：采样先于激励】DMA先收脉冲前静态点，避免丢失基线和上升起点。 */
    if (HAL_ADC_Start_DMA(hadc, (uint32_t *)ls_adc, LS_SAMPLE_COUNT) != HAL_OK) {
        ls_error = 2U;
        MotorCalibration_Stop();
        return false;
    }
    ls_tick = HAL_GetTick();
    if (TIM3->CCR1 != LS_DELAY_US * ls_tim_ticks_us) {
        MotorCalibration_Stop();
        return false;
    }
    /* 最后一次检查并原子武装：防止预充/DMA配置期间按键关断后又重开桥臂。 */
    /* 4.14【步骤10：原子武装】在禁止中断的短窗口内再次确认未被取消，
     * 清TIM3标志、记录DWT起点，再允许比较中断，先启动TIM2后启动TIM3。
     * 这样避免预充或DMA配置期间按键停止后又重新开启MOS。 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (ls_state != LS_SAMPLING) {
        __set_PRIMASK(primask);
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
    __set_PRIMASK(primask);
    return true;
}

/**
 * @brief 开启一次 AB→BC→CA 的自动 Ls 辨识序列。
 * 复用单组 LsStart 的启动、安全校验和采样配置，不另外复制一套脉冲实现。
 * AB启动成功后清空上一轮三组Ls结果，设置ls_auto；每组有效后由主循环排队下一组。
 * 若AB启动失败，则不置自动标志、不继续其他回路。
 * @note 调用环境：串口命令对应的主循环入口
 * @return true 表示第一组AB已启动。
 */
bool MotorCalibration_LsStartAll(void)
{
    if (!MotorCalibration_LsStart(CAL_PHASE_AB))
        return false;
    motor_cal.ls_ab = motor_cal.ls_bc = motor_cal.ls_ca = 0.0f; /* 不混入上一轮结果。 */
    ls_auto = 1U;
    return true;
}

/**
 * @brief 处理TIM3的40/60us比较事件并驱动Ls功率脉冲。
 * CC1：检查DWT时间与CC2状态，先关互补低侧，等待约1us死区，才开高侧。
 * CC2：先关高侧，校验DWT时间及DMA采样下标，再等待死区使两低侧续流。
 * ls_armed阻止尚未配置完毕时误开桥臂；时序异常立即快速关断。
 * 中断内只操作寄存器，不调用HAL GPIO初始化、串口格式化或浮点拟合。
 * @note 调用环境：TIM3中断；故障进入LsFault(5)
 */
void MotorCalibration_LsTimerIRQ(void)
{
    /* 5.1 读取使能的比较标志。未武装时清请求并退出，不能因残留标志开MOS。 */
    uint32_t flags = TIM3->SR & TIM3->DIER; /* 本次已使能的TIM3比较事件标志。 */
    if ((ls_state != LS_SAMPLING) || !ls_armed) {
        TIM3->DIER = 0U;
        TIM3->SR = 0U;
        return;
    }
    /* 5.2 CC1：先确认40us比较事件没有迟到，CC2也未提前到来。 */
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
        /* 切换顺序不可颠倒：先关互补低侧、等待死区、确认CC2未到，再开启高侧。 */
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
    /* 5.3 CC2：先物理关闭高侧，再进行时序和采样位置判断，避免延长激励。 */
    if (TIM3->SR & TIM_SR_CC2IF) {
        GPIOA->BSRR = ls_high_pin << 16; /* 先关闭待测回路的高侧。 */
        TIM3->CR1 &= ~TIM_CR1_CEN;
        ls_update_seen = 1U;
        uint32_t elapsed = DWT->CYCCNT - ls_begin_cycles; /* CC2实际延时周期。 */
        /* CNDTR是剩余DMA长度，64-CNDTR即已采样数；CA取ADC1，其余取ADC2关联DMA。 */
        uint16_t index = (uint16_t)(LS_SAMPLE_COUNT -
            ((motor_cal.phase == CAL_PHASE_CA) ? hadc1.DMA_Handle : hadc2.DMA_Handle)->Instance->CNDTR); /* 由ADC关联DMA读取CC2采样数。 */
        TIM3->DIER = 0U;
        TIM3->SR = 0U;
        ls_armed = 0U;
        /* 两个事件必须顺序正确，且DWT和DMA时标均落入容差窗口。 */
        /* DWT时间和DMA点数必须同时落入窗口，且CC1已发生，才允许进入续流。 */
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
        /* 验证成功后仍需高侧关断到互补低侧开启之间的死区。 */
        uint32_t dead = DWT->CYCCNT; /* 高侧关断后再次等待约1us死区。 */
        while ((uint32_t)(DWT->CYCCNT - dead) < ls_cpu_ticks_us) { }
        GPIOB->BSRR = ls_comp_pin; /* 高侧对应的低侧与回流低侧同步续流。 */
    }
}

/**
 * @brief DMA采到32点时检查60us关断是否发生。
 * 400 kHz下半缓冲为约80us；CC2应已完成。若未发生，按边沿时序异常关断。
 * @note 调用环境：ADC DMA半传输回调
 */
void MotorCalibration_LsDmaHalf(void)
{
    if ((ls_state == LS_SAMPLING) && !ls_update_seen)
        MotorCalibration_LsFault(5U);
}

/**
 * @brief 64点（约160us）DMA完成时安全断电并通知主循环。
 * 先关MOE、六路GPIO、TIM3中断和TIM2采样触发。
 * 若60us关断事件缺失，锁存时序错误；统一将状态交给主循环，不在ISR拟合。
 * @note 调用环境：ADC DMA完成中断
 */
void MotorCalibration_LsDmaComplete(void)
{
    if (ls_state == LS_SAMPLING) {
        /* 5.4 DMA完成后不允许桥臂继续输出；立即拉低所有参与脉冲的GPIO。 */
        __HAL_TIM_MOE_DISABLE(&htim1);
        GPIOA->BSRR = ((uint32_t)GPIO_PIN_8 | GPIO_PIN_10) << 16;
        GPIOB->BSRR = ((uint32_t)GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15) << 16;
        TIM3->CR1 &= ~TIM_CR1_CEN;
        TIM3->DIER = 0U;
        ls_armed = 0U;
        TIM2->CR1 &= ~TIM_CR1_CEN;
        /* 若CC2没有被处理，现有采样数据不可作为有效电感曲线。 */
        if (!ls_update_seen)
            ls_error = 5U; /* 60us 更新事件未执行，不能信任本次 Ls。 */
        ls_state = LS_READY;
    }
}

/**
 * @brief 统一Ls快速故障：先禁止输出，再向主循环发布错误。
 * 1=过流，2=ADC/DMA异常，3=主循环超时或停止，5=边沿/采样时序错误。
 * LS_IDLE且收到停止码3，仅取消排队的下一组；采样阶段则直接关所有输出。
 * 已READY/FAILED时不覆盖既有状态，完整HAL恢复留给主循环。
 * @note 调用环境：TIM3/ADC中断或主循环
 * @param reason 故障码。
 */
void MotorCalibration_LsFault(uint8_t reason)
{
    /* 5.5 已处于IDLE时可能还有自动下一组排队，停止只需清ls_auto。 */
    if (ls_state == LS_IDLE) {
        if (reason == 3U)
            ls_auto = 0U; /* 按键取消已排队、但尚未启动的下一组。 */
        return;
    }
    /* 不覆盖DMA已提交的READY或先前锁存的FAILED。 */
    if (ls_state != LS_SAMPLING)
        return;
    __HAL_TIM_MOE_DISABLE(&htim1); /* 先断功率级，再等待主循环清理外设。 */
    GPIOA->BSRR = ((uint32_t)GPIO_PIN_8 | GPIO_PIN_10) << 16;
    GPIOB->BSRR = ((uint32_t)GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15) << 16;
    TIM3->DIER = 0U;
    ls_armed = 0U;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    /* 输出已经关断，再发布LS_FAILED，让主循环执行耗时恢复。 */
    ls_error = reason;
    ls_state = LS_FAILED;
}

/**
 * @brief 处理一次Ls采样的超时、双沿积分、结果与自动三相推进。
 * 步骤1：采样期间检查10ms超时；READY/FAILED才进入处理。
 * 步骤2：锁存回路、自动标志和故障，再从预脉冲ADC样本修正本次零偏。
 * 步骤3：统一电流方向，求峰值，分别积分上升沿与低侧续流下降沿。
 * 步骤4：先Stop关桥并恢复ADC/TIM1；出错或任一沿无效不写Ls。
 * 步骤5：两沿线电感取平均后除2为相电感，保存当前AB/BC/CA结果。
 * 步骤6：自动模式将下一回路排队到下一轮主循环，CA结束输出三组汇总。
 * @note 调用环境：统一主循环，不在DMA中断执行
 */
static void MotorCalibration_LsProcess(void)
{
    /* 6.1 DMA进行时不读未完成波形，仅检查启动至今是否超过10ms。 */
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

    /* 6.2 Stop会清ls_auto与状态，所以先保存本次故障、自动模式及回路。 */
    uint8_t error = ls_error; /* 锁存本次故障码，Stop之后仍按原始原因报告。 */
    uint8_t auto_run = ls_auto; /* Stop会取消自动序列，先保留本次连续辨识意图。 */
    CalPhase_t finished = motor_cal.phase;
    const char *name = (finished == CAL_PHASE_AB) ? "AB" :
                       (finished == CAL_PHASE_BC) ? "BC" : "CA"; /* 仅输出时使用。 */
    float rise_l = 0.0f;  /* 上升沿得到的线间电感(H)，0表示无效。 */
    float fall_l = 0.0f;  /* 下降沿得到的线间电感(H)，0表示无效。 */
    float rise_i0 = 0.0f, rise_i1 = 0.0f; /* 上升拟合窗口起止电流(A)。 */
    float fall_i0 = 0.0f, fall_i1 = 0.0f; /* 下降拟合窗口起止电流(A)。 */
    float peak = 0.0f; /* 本次采样的最大电流(A)。 */
    /* 6.3 FAILED或已知故障不拟合；只有DMA完成且无错误才读取64点。 */
    if ((ls_state == LS_READY) && !error) {
        /* 6.4 将40/60us换算成样本序号；窗口去掉边沿毛刺和DMA尾端噪声。 */
        const uint32_t on = LS_DELAY_US * LS_SAMPLE_HZ / 1000000U; /* 40us高侧开启对应采样序号。 */
        const uint32_t off = (LS_DELAY_US + LS_PULSE_US) * LS_SAMPLE_HZ / 1000000U; /* 60us关断序号。 */
        const uint32_t up0 = on + 1U, up1 = off - 2U; /* 上升沿去除头尾开关毛刺。 */
        const uint32_t down0 = off + 2U, down1 = LS_SAMPLE_COUNT - 2U; /* 下降沿去除换向和末端噪声。 */
        float offset = 0.0f;   /* 脉冲前ADC码值均值，修正本次电流零偏。 */
        float integral = 0.0f; /* 对电压积分得到电感分子(V·s)。 */

        /* 只用注入前的静态样本求零偏，避免电流上升污染偏置值。 */
        /* 6.5 注入前的ADC样本平均形成本次零点，不用脉冲电流污染偏置。 */
        for (uint32_t k = 0U; k < on - 1U; k++)
            offset += (float)ls_adc[k];
        offset /= (float)(on - 1U);
        /* 沿用上升段的电流方向，避免下降末端噪声被 fabs 误判成回升。 */
        /* 6.6 用上升段决定电流正方向，两段均沿用相同scale，避免符号跳变。 */
        float direction = ((float)ls_adc[up1] >= offset) ? 1.0f : -1.0f; /* 实际电流符号。 */
        float scale = direction * ls_gain; /* ADC码差到正向回路电流的换算比例(A/LSB)。 */
        rise_i0 = ((float)ls_adc[up0] - offset) * scale;
        rise_i1 = ((float)ls_adc[up1] - offset) * scale;
        fall_i0 = ((float)ls_adc[down0] - offset) * scale;
        fall_i1 = ((float)ls_adc[down1] - offset) * scale;

        /* 扫描本次脉冲和续流段的峰值，剔除过流数据。 */
        /* 6.7 扫描本次峰值，达到保护阈值后不信任该次计算结果。 */
        for (uint32_t k = on; k < LS_SAMPLE_COUNT; k++) {
            float i = ((float)ls_adc[k] - offset) * scale; /* 当前点的回路电流(A)。 */
            if (i > peak) peak = i;
        }

        /* 上升段：按L·di/dt=Vbus-2Rs·i，使用梯形积分补偿电阻压降。 */
        /* 6.8 上升沿：积分(Vbus-2Rs×I)dt，再除以该窗口电流增量得到线电感。 */
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
        /* 6.9 下降沿：两低侧续流，端电压近似0；积分2Rs×I并除以电流下降量。 */
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

    /* 6.10 无论拟合成功与否，先恢复外设且保持功率级关闭，再处理结果与日志。 */
    MotorCalibration_Stop(); /* 先安全关断，再输出结果。 */
    /* 6.11 中断故障优先，自动序列到此终止，不进入下一回路。 */
    if (error) {
        DebugConsole_Printf("Ls %s ERROR %u (1=current 2=ADC/DMA 3=timeout 5=edge)\r\n",
                            name, (unsigned)error);
        if (auto_run)
            DebugConsole_Printf("Ls sequence stopped at %s\r\n", name);
        return;
    }

    /* 只保留上升、下降及综合值；任一拟合无效则不更新RAM结果。 */
    /* 6.12 任一沿积分无效或越界，丢弃本组，不更新RAM。 */
    if (!(rise_l > 0.000001f && rise_l < 0.1f &&
          fall_l > 0.000001f && fall_l < 0.1f)) {
        DebugConsole_Printf("Ls %s invalid: rise=%.3f fall=%.3fuH\r\n",
                            name, rise_l * 5e5f, fall_l * 5e5f);
        ls_error = 6U; /* 双沿拟合无效，供CAN异步结果监控区分故障与主动停止。 */
        if (auto_run)
            DebugConsole_Printf("Ls sequence stopped at %s\r\n", name);
        return;
    }
    /* 6.13 两个线间电感先平均，再除2换算相电感，总系数为0.25。 */
    float ls = (rise_l + fall_l) * 0.25f; /* 两个线电感的均值，再除2得到相电感。 */
    switch (motor_cal.phase) {
    case CAL_PHASE_AB: motor_cal.ls_ab = ls; break;
    case CAL_PHASE_BC: motor_cal.ls_bc = ls; break;
    case CAL_PHASE_CA: motor_cal.ls_ca = ls; break;
    default: return;
    }
    DebugConsole_Printf("Ls %s: RISE=%.3f FALL=%.3f mean(rise,fall)=%.3fuH Ipeak=%.2fA\r\n",
                        name, rise_l * 5e5f, fall_l * 5e5f, ls * 1e6f, peak);

    /* 6.14 下一组在下一次Process才启动，先给按键和串口停止留处理机会。 */
    if (auto_run) {
        if (finished != CAL_PHASE_CA) {
            /* Stop已恢复并关断；下一轮主循环先处理停止命令，再决定是否启动下组。 */
            motor_cal.phase = (CalPhase_t)(finished + 1U);
            ls_auto = 1U; /* LS_IDLE + ls_auto：下一组待启动，绝不在ISR中换相。 */
        } else {
            DebugConsole_Printf("Ls complete: AB=%.3f BC=%.3f CA=%.3fuH\r\n",
                                motor_cal.ls_ab * 1e6f, motor_cal.ls_bc * 1e6f,
                                motor_cal.ls_ca * 1e6f);
        }
    }
}

/* ========================================================================== */
/* 七、Rs：新一轮辨识启动                                                     */
/* ========================================================================== */

/**
 * @brief 启动完整AB→BC→CA线间电阻辨识。
 * 先Stop恢复标准外设并清空本轮Rs/Ls RAM，再设置AB、1%初始占空比。
 * 配置桥臂后只置CAL_SET_DUTY；下一轮主循环在短临界区启动PWM。
 * @note 调用环境：串口rs或PC13按键经主循环调用
 */
void MotorCalibration_Start(void)
{
    MotorCalibration_Stop(); /* 先关闭功率级并恢复外设，确保重新测量安全。 */
    /* 7.1 新Rs会使旧Ls使用的电阻补偿失效，因此同时清空全部旧测量结果。 */
    memset((void *)&motor_cal, 0, sizeof(motor_cal)); /* 清零本轮状态及测量缓存。 */

    /* 7.2 只排队第一个档位，实际PWM开启不在本函数中执行。 */
    motor_cal.phase = CAL_PHASE_AB;
    motor_cal.duty = CAL_DUTY_START;
    Cal_ConfigPhase();
    motor_cal.state = CAL_SET_DUTY;
}

/* ========================================================================== */
/* 八、统一安全退出与外设恢复                                                 */
/* ========================================================================== */

/**
 * @brief 统一关闭功率级并恢复正式固件的ADC/TIM1配置。
 * 步骤1：关MOE、取消自动下一组并停止PWM，六路GPIO全拉低。
 * 步骤2：若Ls使用过TIM2/3与ADC DMA，则停止触发、DMA和当前ADC规则组。
 * 步骤3：恢复本次占用ADC的通道配置，清除Ls模拟看门狗设置。
 * 步骤4：MX_TIM1_Init恢复正常PWM与ADC触发配置，但继续强制关闭输出。
 * 步骤5：与上电共用恢复入口，重新武装两路注入组：ADC1仅JEOS、ADC2无中断。
 * 两路恢复成功后才置LS_IDLE/FOC待机；不擦除辨识数值。
 * CA原母线/温度轮询由MotorApp_Process后续恢复，不在此启动规则DMA。
 * @note 调用环境：主循环；不得在高频中断中执行HAL初始化
 */
void MotorCalibration_Stop(void)
{
    /* 先快速关闭功率输出，再停止全部PWM。此函数只由主循环调用。 */
    /* 8.1 先禁止功率输出和自动续测，再进入耗时的外设重建。 */
    motor_cal.state = CAL_IDLE;
    __HAL_TIM_MOE_DISABLE(&htim1);
    ls_armed = 0U;
    ls_auto = 0U; /* 任何Stop（包括ls stop或按键）都取消剩余自动回路。 */
    FOC_PWM_Stop();
    TIM1->CR1 &= ~TIM_CR1_CEN; /* 明确停止触发源，恢复ADC期间不产生新采样。 */

    /* 切回标准TIM1配置前，六路GPIO必须先全部拉低。 */
    Cal_AllPinsLow();
    /* 8.2 本次是否确实占用Ls外设只记录一次；纯Rs不额外停止ADC规则DMA。 */
    const bool restore_ls = (ls_state != LS_IDLE);
    ADC_HandleTypeDef *hadc = restore_ls ?
        ((motor_cal.phase == CAL_PHASE_CA) ? &hadc1 : &hadc2) : NULL;

    /* 8.3 先停止硬件触发，再停DMA和重配ADC，防止恢复过程中再次转换。 */
    if (restore_ls) {
        /* 先停TIM3脉冲和TIM2采样，再停止ADC/DMA并调用原CubeMX初始化。 */
        NVIC_DisableIRQ(TIM3_IRQn);
        TIM3->CR1 = TIM3->DIER = 0U;
        TIM3->SR = 0U;
        NVIC_ClearPendingIRQ(TIM3_IRQn);
        __HAL_RCC_TIM3_CLK_DISABLE();
        TIM2->CR1 = TIM2->SMCR = TIM2->CR2 = 0U;
        __HAL_RCC_TIM2_CLK_DISABLE();

        if (HAL_ADC_Stop_DMA(hadc) != HAL_OK)
            Error_Handler(); /* 恢复失败时不允许进入无电流反馈的FOC。 */

        /* 仅恢复本次占用的ADC，不影响另一路正在使用的ADC和CAN/串口。 */
        /* 8.4 只恢复被当前Ls临时占用的ADC，不重配无关外设。 */
        if (motor_cal.phase == CAL_PHASE_CA)
            MX_ADC1_Init();
        else
            MX_ADC2_Init();
        CLEAR_BIT(hadc->Instance->CFGR, ADC_CFGR_AWD1EN); /* 解除Ls过流看门狗。 */
        CLEAR_BIT(hadc->Instance->IER, ADC_IER_AWD1IE);
        hadc->Instance->ISR = ADC_ISR_AWD1 | ADC_ISR_OVR;
    }

    /* 复用标准TIM1初始化，恢复中心对齐、PWM、死区、采样触发和引脚AF。 */
    /* 8.5 标准TIM1配置恢复后仍撤销MOE/CEN/CCER，不能自动转动。 */
    MX_TIM1_Init();
    __HAL_TIM_MOE_DISABLE(&htim1);
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC4E);
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0U;

    /* 8.6 Rs/Ls退出均恢复两路注入采样，不能只重启最后使用的ADC。
     * 与上电共用JEOS策略；恢复失败时保持MOE/CEN关闭，不发布可运行状态。 */
    if (ADC_Injected_RestoreForFoc() != HAL_OK) {
        Error_Handler();
        return;
    }
    ls_state = LS_IDLE;
    /* 8.7 整个外设恢复完成后回到待机；结果继续保存在RAM。 */
    foc_motor_state = FOC_MOTOR_IDLE;
}

/* ========================================================================== */
/* 九、Rs实时采样与Rs/Ls主循环统一调度                                        */
/* ========================================================================== */

/**
 * @brief 推进一次Rs的PWM同步ADC注入采样。
 * 只响应CAL_SETTLE/CAL_SAMPLE；任一相达到5.5A立即快速关PWM并置ERROR。
 * 先舍弃250拍暂态，再累计250拍线电流与母线电压；完成时关PWM并置READY。
 * 平均、拟合、升档、换相和完整恢复都交由主循环。
 * @note 调用环境：25 kHz ADC注入中断
 * @param ia A相电流(A)。
 * @param ib B相电流(A)。
 * @param ic C相电流(A)。
 * @param vbus 本拍母线电压(V)。
 */
void MotorCalibration_Run(float ia, float ib, float ic, float vbus)
{
    /* 9.1 READY以后本档累加值不再由中断修改，供主循环读取。 */
    MotorCalState_t state = motor_cal.state;
    if ((state != CAL_SETTLE) && (state != CAL_SAMPLE))
        return;

    /* 电流保护始终优先于正常采样完成，异常直接硬件关断。 */
    if ((__builtin_fabsf(ia) >= CAL_HARD_CURRENT) ||
        (__builtin_fabsf(ib) >= CAL_HARD_CURRENT) ||
        (__builtin_fabsf(ic) >= CAL_HARD_CURRENT)) {
        Cal_RsQuickOff();
        motor_cal.state = CAL_ERROR;
        return;
    }

    /* 9.2 稳定等待约10ms，不将刚改变PWM后的暂态纳入电阻拟合。 */
    if (state == CAL_SETTLE) {
        if (++motor_cal.count >= CAL_SETTLE_COUNT) {
            motor_cal.count = 0U;
            motor_cal.current_sum = 0.0f;
            motor_cal.vbus_sum = 0.0f;
            motor_cal.state = CAL_SAMPLE;
        }
        return;
    }

    /* 9.3 累计250拍；主循环随后分别除以250，得到本档平均I和Vbus。 */
    motor_cal.current_sum += Cal_LineCurrent(ia, ib, ic);
    motor_cal.vbus_sum += vbus;
    /* 9.4 采满先快速关PWM，再通知主循环，防止主循环延迟导致继续注入。 */
    if (++motor_cal.count >= CAL_SAMPLE_COUNT) {
        Cal_RsQuickOff(); /* 本档到点先关PWM，主循环处理完才允许下一档。 */
        motor_cal.state = CAL_READY;
    }
}

/**
 * @brief 统一完成Ls续测、Rs档位推进、拟合以及终态输出。
 * 先处理Ls自动队列和Ls当前采样，再处理Rs的设置PWM、READY与终态。
 * Rs CAL_READY内求平均、保存五点环形缓存，并按目标电流/上限决定升档或拟合。
 * AB/BC拟合后换相，CA结束后反算RA/RB/RC及平均Rs。
 * 最终Stop只执行一次；等待已有日志排空后只打印一次汇总。
 * @note 调用环境：MotorApp_Process每轮调用
 */
void MotorCalibration_Process(void)
{
    /* 9.5 handled避免终态重复Stop，printed避免终态重复打印。 */
    static MotorCalState_t handled = CAL_IDLE; /* 已完成外设恢复的Rs终态。 */
    static MotorCalState_t printed = CAL_IDLE; /* 已输出的Rs终态。 */

    /* LS_IDLE且自动序列待续：必须先经过主循环按键/串口停止处理。 */
    /* 9.6 Ls排队的下一组由主循环启动，此前先处理串口/按键停止请求。 */
    if ((ls_state == LS_IDLE) && ls_auto) {
        CalPhase_t next = motor_cal.phase;
        if (!ls_auto)
            return; /* EXTI恰好取消了下一组。 */
        if (MotorCalibration_LsStart(next)) {
            ls_auto = 1U; /* Start内部Stop会清除标志，成功后恢复连续辨识。 */
            DebugConsole_Printf("Ls %s started\r\n",
                                (next == CAL_PHASE_AB) ? "AB" :
                                (next == CAL_PHASE_BC) ? "BC" : "CA");
        } else {
            ls_auto = 0U;
            DebugConsole_Printf("Ls sequence ERROR: next phase rejected\r\n");
        }
        return;
    }

    /* Ls仍沿用原DMA、TIM3时序和双沿计算。 */
    /* 9.7 Ls占用ADC DMA和TIM2/3期间，不并行推进Rs档位。 */
    if (ls_state != LS_IDLE) {
        MotorCalibration_LsProcess();
        return;
    }

    MotorCalState_t state = motor_cal.state;
    /* 9.8 只在短临界区设置CCR/MOE和计数，避免按键ISR刚关断又被主循环开启。 */
    if (state == CAL_SET_DUTY) {
        /* 防止按键ISR刚关MOE，主循环又在同一时刻重新打开功率级。 */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (motor_cal.state == CAL_SET_DUTY) {
            motor_cal.count = 0U;
            motor_cal.current_sum = 0.0f;
            motor_cal.vbus_sum = 0.0f;
            Cal_SetDuty(motor_cal.duty); /* 纯寄存器操作，临界区内不调用HAL。 */
            motor_cal.state = CAL_SETTLE;
        }
        __set_PRIMASK(primask);
        handled = printed = CAL_IDLE;
        return;
    }

    /* 9.9 PWM已关闭，本档累加值不再变化，可在主循环进行浮点计算。 */
    if (state == CAL_READY) {
        /* 中断已关MOE，两个和及本档Duty不会再被ISR修改。 */
        float avg_i = motor_cal.current_sum / (float)CAL_SAMPLE_COUNT;
        float avg_vbus = motor_cal.vbus_sum / (float)CAL_SAMPLE_COUNT;

        /* 9.10 总档数最多50，内存只循环保存最近5档的V/I均值。 */
        if (motor_cal.point_count < CAL_MAX_POINTS) {
            volatile MotorCalPoint_t *p =
                &motor_cal.point[motor_cal.point_count % CAL_FIT_POINTS];
            motor_cal.point_count++;
            p->voltage = motor_cal.duty * avg_vbus;
            p->current = avg_i;

            /* 原终止条件和五点拟合窗口不变；下一档重新等待250拍。 */
            if ((avg_i < CAL_TARGET_CURRENT) &&
                (motor_cal.duty + CAL_DUTY_STEP <= CAL_DUTY_MAX)) {
                motor_cal.duty += CAL_DUTY_STEP;
                motor_cal.state = CAL_SET_DUTY;
                return;
            }
        }

        /* 9.11 本回路停止升档，拟合五点斜率；最终CA再将三个线阻换算为相阻。 */
        CalPhase_t finished = motor_cal.phase;
        float line_r = Cal_FitResistance();
        switch (finished) {
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
            break;
        default:
            motor_cal.state = CAL_ERROR;
            break;
        }

        if (finished <= CAL_PHASE_CA)
            DebugConsole_Printf("R_%s = %.6f ohm\r\n",
                (finished == CAL_PHASE_AB) ? "AB" :
                (finished == CAL_PHASE_BC) ? "BC" : "CA", line_r);

        /* 9.12 不是CA时配置下一回路并从1%重新开始；CA置DONE。 */
        if (motor_cal.state != CAL_ERROR) {
            if (finished != CAL_PHASE_CA) {
                Cal_ConfigPhase(); /* 主循环换相，所有PWM在此之前均关闭。 */
                motor_cal.duty = CAL_DUTY_START;
                motor_cal.point_count = 0U;
                motor_cal.state = CAL_SET_DUTY;
                return;
            }
            motor_cal.state = CAL_DONE;
        }
        state = motor_cal.state;
    }

    /* 9.13 仅最终DONE/ERROR触发完整Stop；非终态等待下一轮调度。 */
    if ((state != CAL_DONE) && (state != CAL_ERROR)) {
        handled = printed = CAL_IDLE;
        return;
    }

    /* 9.14 Stop会重置state为IDLE，随后恢复DONE/ERROR供日志输出。 */
    if (handled != state) {
        MotorCalibration_Stop(); /* 与Ls一样，只在主循环做完整外设恢复。 */
        motor_cal.state = state; /* Stop设为IDLE，保留终态供打印。 */
        handled = state;
    }
    /* 9.15 等队列中旧日志排空并避免重复打印最终结果。 */
    if (printed == state)
        return;

    printed = state;
    if (state == CAL_DONE) {
        DebugConsole_Printf("\r\n===== Rs Calibration Result =====\r\n"
                            "R_A  = %.6f ohm\r\nR_B  = %.6f ohm\r\n"
                            "R_C  = %.6f ohm\r\nR_S  = %.6f ohm\r\n",
                            motor_cal.r_a, motor_cal.r_b,
                            motor_cal.r_c, motor_cal.rs);
    } else {
        DebugConsole_Printf("\r\nRs calibration ERROR: duty=%.2f%%\r\n",
                            motor_cal.duty * 100.0f);
    }
}

/* 规则组DMA回调：Ls期间把半传输/完成/错误和模拟看门狗直接交给
 * 辨识状态机；普通运行路径仍由BoardAdc轮询，不依赖这些回调。 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc != NULL) &&
        ((hadc->Instance == ADC2) ||
         ((hadc->Instance == ADC1) && MotorCalibration_LsUsesADC1()))) {
        MotorCalibration_LsDmaHalf();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc != NULL) &&
        ((hadc->Instance == ADC2) ||
         ((hadc->Instance == ADC1) && MotorCalibration_LsUsesADC1()))) {
        MotorCalibration_LsDmaComplete();
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == NULL) return;
    if ((hadc->Instance == ADC2) ||
        ((hadc->Instance == ADC1) && MotorCalibration_LsUsesADC1())) {
        MotorCalibration_LsFault(2U);
    }
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc != NULL) &&
        ((hadc->Instance == ADC2) ||
         ((hadc->Instance == ADC1) && MotorCalibration_LsUsesADC1()))) {
        MotorCalibration_LsFault(1U);
    }
}
