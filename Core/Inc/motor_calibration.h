#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

/**
 * @file motor_calibration.h
 * @brief 静止状态下的电机参数辨识接口。
 * @details 按四个命令电压档位注入，累计已施加 Duty、母线电压和相电流，
 *          并拟合线间电阻。该模块独占
 *          TIM1 相输出和 ADC 回调期间的 CCR1 控制权。
 */

#include "foc_math.h"
#include "motor_calibration_ls.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MOTOR_CALIBRATION_IDLE = 0U, /**< 未运行，可接受新的辨识请求。 */
    MOTOR_CALIBRATION_STARTING,  /**< 正在准备定时器和输出，MOE保持关闭。 */
    MOTOR_CALIBRATION_RUNNING,   /**< 正在注入并采集，ADC中断执行逐样本处理。 */
    MOTOR_CALIBRATION_STOPPING,  /**< 已关断相输出，等待主循环停止定时器。 */
    MOTOR_CALIBRATION_DONE,      /**< 辨识与有效性检查通过，结果可读取。 */
    MOTOR_CALIBRATION_ERROR     /**< 辨识失败或被用户中止，结果无效。 */
} MotorCalibrationState_t;

/**
 * @brief 以默认定电压模式启动静止 Rs 辨识。
 * @return HAL_OK 表示已启动；HAL_BUSY 表示已有辨识运行；HAL_ERROR 表示前置条件失败。
 * @note 相输出在配置比较寄存器和通道前保持关闭；此函数不等待辨识完成。
 */
HAL_StatusTypeDef MotorCalibration_Start(void);
/**
 * @brief 请求立即停止辨识并关断相功率输出。
 * @return 无。
 * @note 清 MOE 后由主循环完成 CH4 和 TIM1 停机及最终状态报告。
 */
void MotorCalibration_Stop(void);
/**
 * @brief 处理一次注入 ADC 回调中的电流样本。
 * @return 无。
 * @note 在中断上下文执行，仅检查、更新控制和累计数据，不打印或进行拟合；
 *       软件过流时立即清 MOE 关断功率输出。
 */
void MotorCalibration_AdcStep(void);
/**
 * @brief 主循环服务函数：看门狗、定时器安全停机、拟合及日志输出。
 * @return 无。
 * @note 必须由主循环周期调用；ADC 停更和单档超时在此检测。
 */
void MotorCalibration_Process(void);
/**
 * @brief 查询模块是否占有辨识执行状态。
 * @return 1 表示启动、运行或关断流程尚未结束；0 表示不活跃。
 */
uint8_t MotorCalibration_IsActive(void);
/**
 * @brief 读取当前辨识状态。
 * @return MotorCalibrationState_t 状态枚举值。
 */
MotorCalibrationState_t MotorCalibration_GetState(void);
/**
 * @brief 读取有效的单相等效 Rs 结果。
 * @return 成功时为欧姆；尚未成功或已失败时返回 NAN。
 */
float MotorCalibration_GetResultOhm(void);
/**
 * @brief 读取最近一次启动或运行的错误码。
 * @return 0 表示无错误，非零值对应实现中的中止原因表。
 */
uint8_t MotorCalibration_GetErrorCode(void);

/**
 * @brief 启动一次硬件限时 A-B 单脉冲及 ADC2 规则组 DMA 采样。
 * @return HAL_OK 表示已开始，HAL_BUSY 表示已有辨识在运行，HAL_ERROR 表示前置检查失败。
 * @note MCU 引脚与 ADC 采集已实机验证；MOS 栅极波形及看门狗关断延迟仍需示波器确认。
 */
HAL_StatusTypeDef MotorCalibration_LsStart(void);
void MotorCalibration_LsProcess(void);
void MotorCalibration_LsTim1Update(void);
void MotorCalibration_LsTim1Compare(void);
void MotorCalibration_LsDmaHalf(void);
void MotorCalibration_LsDmaComplete(void);
void MotorCalibration_LsDmaError(void);
void MotorCalibration_LsOvercurrent(void);
uint8_t MotorCalibration_LsIsActive(void);

#ifdef __cplusplus
}
#endif

#endif
