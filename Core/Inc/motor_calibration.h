#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

#include "main.h"
#include <stdint.h>

/* ===== 可调参数 ===== */
#define CAL_DUTY_START       0.010f   /* 1% */
#define CAL_DUTY_STEP        0.005f   /* 每档 +0.5% */
#define CAL_DUTY_MAX         0.200f   /* 最大 20% */

#define CAL_TARGET_CURRENT   5.0f     /* 平均线电流到 5A 正常结束 */
#define CAL_HARD_CURRENT     5.5f     /* 瞬时硬保护 */

#define CAL_SETTLE_COUNT     250U     /* 25kHz 下 10ms */
#define CAL_SAMPLE_COUNT     250U     /* 25kHz 下平均 10ms */

#define CAL_MAX_POINTS       50U
#define CAL_FIT_POINTS       5U       /* 最后 5 点拟合 */

typedef struct {
    float duty;
    float voltage;
    float current;
} MotorCalPoint_t;

typedef enum {
    CAL_PHASE_AB = 0,
    CAL_PHASE_BC,
    CAL_PHASE_CA
} CalPhase_t;

typedef enum {
    CAL_IDLE = 0,
    CAL_SET_DUTY,
    CAL_SETTLE,
    CAL_SAMPLE,
    CAL_DONE,
    CAL_ERROR
} MotorCalState_t;

typedef struct {
    volatile MotorCalState_t state;
    CalPhase_t phase;

    float duty;
    uint32_t count;
    float current_sum;
    float vbus_sum;

    uint16_t point_count;
    MotorCalPoint_t point[CAL_MAX_POINTS];

    float r_ab;
    float r_bc;
    float r_ca;
    float r_a;
    float r_b;
    float r_c;
    float rs;
} MotorCalibration_t;

extern volatile MotorCalibration_t motor_cal;

/* 开始 A-B 电阻测试。调用前确保 ADC 注入采样已经正常运行。 */
void MotorCalibration_Start(void);

/* ADC 注入中断中调用：ia/ib 单位 A，vbus 单位 V。 */
void MotorCalibration_Run(float ia, float ib, float ic, float vbus);

/* 主循环调用；DONE/ERROR 时用 DebugConsole_Printf 打印一次。 */
void MotorCalibration_DebugProcess(void);

/* 立即关功率输出并回到 IDLE。 */
void MotorCalibration_Stop(void);

#endif
