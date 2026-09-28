#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

#include "main.h"
#include <stdint.h>

/* ===== 可调参数 ===== */
#define CAL_DUTY_START       0.010f   /* 1% */
#define CAL_DUTY_STEP        0.005f   /* 每档 +0.5% */
#define CAL_DUTY_MAX         0.500f   /* 最大 50% */

#define CAL_TARGET_CURRENT   5.0f     /* 平均线电流到 5A 正常结束 */
#define CAL_HARD_CURRENT     5.5f     /* 瞬时硬保护 */

#define CAL_SETTLE_COUNT     250U     /* 25kHz 下 10ms */
#define CAL_SAMPLE_COUNT     250U     /* 25kHz 下平均 10ms */

#define CAL_MAX_POINTS       50U
#define CAL_FIT_POINTS       5U       /* 最后 5 点拟合 */

/* 单档占空比对应的一组平均电压、电流测量点。 */
typedef struct {
    float duty;       /* 当前 PWM 占空比，范围 0~1。 */
    float voltage;    /* 注入电压近似值：占空比 × 平均母线电压，单位 V。 */
    float current;    /* 当前线间回路的平均电流，单位 A。 */
} MotorCalPoint_t;

/* 依次辨识的三组线间回路。 */
typedef enum {
    CAL_PHASE_AB = 0, /* A-B 线间电阻。 */
    CAL_PHASE_BC,     /* B-C 线间电阻。 */
    CAL_PHASE_CA      /* C-A 线间电阻。 */
} CalPhase_t;

/* 电阻辨识状态机，由 ADC 注入转换回调逐次推进。 */
typedef enum {
    CAL_IDLE = 0,     /* 空闲，未进行辨识。 */
    CAL_SET_DUTY,     /* 设置本档占空比并开启 PWM。 */
    CAL_SETTLE,       /* 等待电流稳定，不累计测量值。 */
    CAL_SAMPLE,       /* 累计电流与母线电压，形成平均测量点。 */
    CAL_DONE,         /* 三组回路辨识完成，结果待输出。 */
    CAL_ERROR         /* 过流或异常退出，错误待输出。 */
} MotorCalState_t;

/* 一次 Rs 辨识的状态、采样缓存与结果。 */
typedef struct {
    volatile MotorCalState_t state; /* 中断和主循环共用的当前状态。 */
    CalPhase_t phase;               /* 当前测量回路：AB、BC 或 CA。 */

    float duty;         /* 当前 PWM 占空比，范围 0~1。 */
    uint32_t count;     /* 当前稳定等待或采样阶段的 ADC 回调计数。 */
    float current_sum;  /* 当前采样窗口的线电流累加值。 */
    float vbus_sum;     /* 当前采样窗口的母线电压累加值。 */

    uint16_t point_count;                /* 当前回路已保存的测量点数。 */
    MotorCalPoint_t point[CAL_MAX_POINTS]; /* 当前回路的测量点，切相时复用。 */

    float r_ab; /* A-B 线间电阻，单位 Ω。 */
    float r_bc; /* B-C 线间电阻，单位 Ω。 */
    float r_ca; /* C-A 线间电阻，单位 Ω。 */
    float r_a;  /* 换算后的 A 相电阻，单位 Ω。 */
    float r_b;  /* 换算后的 B 相电阻，单位 Ω。 */
    float r_c;  /* 换算后的 C 相电阻，单位 Ω。 */
    float rs;   /* 三相电阻平均值，作为 FOC 定子相电阻 Rs。 */
} MotorCalibration_t;

/* 全局辨识实例：ADC 回调更新，主循环读取状态和结果。 */
extern volatile MotorCalibration_t motor_cal;

/* 开始 A-B 电阻测试。调用前确保 ADC 注入采样已经正常运行。 */
void MotorCalibration_Start(void);

/* ADC 注入中断中调用：ia/ib 单位 A，vbus 单位 V。 */
void MotorCalibration_Run(float ia, float ib, float ic, float vbus);

/* 主循环：通用日志排空后，DONE/ERROR 时输出最终结果。 */
void MotorCalibration_DebugProcess(void);

/* 立即关功率输出并回到 IDLE。 */
void MotorCalibration_Stop(void);

#endif
