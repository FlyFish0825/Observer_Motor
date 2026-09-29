#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* ======================= 共同使用：回路定义 ======================= */

/* 依次辨识的三组线间回路。 */
typedef enum {
    CAL_PHASE_AB = 0, /* A-B回路：Rs按AB，Ls为AH→BL。 */
    CAL_PHASE_BC,     /* B-C回路：Rs按BC，Ls为CH→BL。 */
    CAL_PHASE_CA      /* C-A回路：Rs按CA，Ls为CH→AL。 */
} CalPhase_t;

/* ======================== Rs：测量参数 ======================== */
#define CAL_DUTY_START       0.010f   /* Rs起始占空比1%，每组线间电阻均从该值开始。 */
#define CAL_DUTY_STEP        0.005f   /* Rs每档占空比增加0.5%，避免电流突变。 */
#define CAL_DUTY_MAX         0.500f   /* Rs最高允许占空比50%，到达后结束该回路。 */

#define CAL_TARGET_CURRENT   5.0f     /* 单档平均线电流达到5A时停止本回路升压。 */
#define CAL_HARD_CURRENT     5.5f     /* Rs任一相瞬时电流达到5.5A立即软件关断。 */

#define CAL_SETTLE_COUNT     250U     /* 每档等待250次注入ADC回调（25kHz时约10ms）。 */
#define CAL_SAMPLE_COUNT     250U     /* 每档累加250次ADC采样求平均，滤除纹波。 */

#define CAL_MAX_POINTS       50U     /* 当前回路最多保存50组电压/电流平均点，换相复用。 */
#define CAL_FIT_POINTS       5U      /* 最小二乘只取当前回路末尾5个测量点。 */

/* ======================== Rs：测量点与状态 ======================== */

/* 单档占空比对应的一组平均电压、电流测量点。 */
typedef struct {
    float voltage;    /* 本档占空比×平均母线电压(V)。 */
    float current;    /* 本档平均线电流(A)。 */
} MotorCalPoint_t;

/* Rs由主循环推进流程；ADC中断只等待、采样、快速关断并发布READY。 */
typedef enum {
    CAL_IDLE = 0,     /* 空闲，未进行辨识。 */
    CAL_SET_DUTY,     /* 设置本档占空比并开启 PWM。 */
    CAL_SETTLE,       /* 等待电流稳定，不累计测量值。 */
    CAL_SAMPLE,       /* 累计电流与母线电压。 */
    CAL_READY,        /* 本档250点就绪，PWM已关闭，主循环计算并升档/换相。 */
    CAL_DONE,         /* 已快速关断，主循环恢复外设并输出结果。 */
    CAL_ERROR         /* 已快速关断，主循环恢复外设并输出错误。 */
} MotorCalState_t;

/* ======================== Ls：采样参数 ======================== */
/* TIM3控制GPIO，TIM2触发规则组DMA；
 * 40us启动、60us关闭高侧，400kHz采样64点；外部快速过流保护独立有效。 */
#define LS_SAMPLE_HZ        400000U /* TIM2触发规则组ADC采样频率400kHz，即每2.5us一点。 */
#define LS_SAMPLE_COUNT     64U     /* 单次DMA采集64点，覆盖160us电流上升和衰减。 */
#define LS_DELAY_US         40U     /* TIM3启动后40us产生高侧导通比较事件。 */
#define LS_PULSE_US         20U     /* 注入脉冲目标宽度20us，60us比较事件关断。 */
#define LS_HARD_CURRENT     10.0f  /* Ls ADC看门狗电流限值10A；外部快保护独立。 */

/* ==================== 共同使用：RAM状态与结果 ==================== */

/* Rs/Ls共享的唯一全局结果实例；不增加新的全局数据结构。 */
typedef struct {
    /* ------------------- Rs：状态、采样与拟合结果 ------------------- */
    volatile MotorCalState_t state; /* Rs中断采样并通知就绪，主循环计算和推进。 */
    CalPhase_t phase;               /* Rs/Ls共用当前测量回路：AB、BC或CA。 */

    float duty;         /* 当前 PWM 占空比，范围 0~1。 */
    uint32_t count;     /* 当前稳定等待或采样阶段的 ADC 回调计数。 */
    float current_sum;  /* 当前采样窗口的线电流累加值。 */
    float vbus_sum;     /* 当前采样窗口的母线电压累加值。 */

    uint16_t point_count; /* 当前回路累计测量档数，上限仍为50。 */
    MotorCalPoint_t point[CAL_FIT_POINTS]; /* 只循环保存最后5档，拟合窗口不变。 */

    float r_ab; /* A-B 线间电阻，单位 Ω。 */
    float r_bc; /* B-C 线间电阻，单位 Ω。 */
    float r_ca; /* C-A 线间电阻，单位 Ω。 */
    float r_a;  /* 换算后的 A 相电阻，单位 Ω。 */
    float r_b;  /* 换算后的 B 相电阻，单位 Ω。 */
    float r_c;  /* 换算后的 C 相电阻，单位 Ω。 */
    float rs;   /* 三相电阻平均值，作为 FOC 定子相电阻 Rs。 */
    /* ---------------------- Ls：辨识结果 ---------------------- */
    float ls_ab; /* AB回路上、下降沿线间电感均值再除2，单位H。 */
    float ls_bc; /* BC回路上、下降沿线间电感均值再除2，单位H。 */
    float ls_ca; /* CA回路上、下降沿线间电感均值再除2，单位H。 */
} MotorCalibration_t;

/* 全局辨识实例：ADC 回调更新，主循环读取状态和结果。 */
extern volatile MotorCalibration_t motor_cal;

/* ======================== Rs：电阻辨识接口 ======================== */

/* 启动完整AB→BC→CA电阻辨识；调用前确保ADC注入采样正常。 */
void MotorCalibration_Start(void);

/* ADC注入中断调用：ia/ib/ic 为三相电流(A)，vbus为母线电压(V)。 */
void MotorCalibration_Run(float ia, float ib, float ic, float vbus);

/* 主循环统一入口：Rs处理档位/拟合，Ls处理DMA结果；两者共用Stop。 */
void MotorCalibration_Process(void);

/* ======================== 共同使用：安全停止 ======================== */

/* 统一安全停止：关桥臂、结束Ls采样、恢复ADC/TIM1，保持电机停机。 */
void MotorCalibration_Stop(void);

/* ======================== Ls：电感辨识接口 ======================== */

/* 启动指定回路的单次Ls辨识：AB/BC采ADC2-B相，CA采ADC1-A相；
 * 用2×平均Rs补偿线间电阻；若电机忙、未校零、母线异常或定时器被占用则返回false。 */
bool MotorCalibration_LsStart(CalPhase_t phase); /* 单回路调试入口。 */
bool MotorCalibration_LsStartAll(void);         /* ls命令：自动AB→BC→CA。 */
bool MotorCalibration_LsUsesADC1(void);   /* 告知ADC回调：CA期间ADC1 DMA数据属于Ls。 */
void MotorCalibration_LsDmaComplete(void); /* DMA完成：立即关输出。 */
void MotorCalibration_LsDmaHalf(void);     /* 半传输：检查60us关断事件。 */
void MotorCalibration_LsTimerIRQ(void);    /* TIM3比较中断：切换GPIO并校验时序。 */
void MotorCalibration_LsFault(uint8_t reason); /* 故障关桥：1过流、2 ADC/DMA、3超时、5时序。 */

#endif
