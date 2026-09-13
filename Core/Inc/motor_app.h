#ifndef MOTOR_APP_H
#define MOTOR_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/**
 * @brief HAL初始化前将六路栅极驱动输出置于安全低电平
 */
void MotorApp_ForcePowerStageSafe(void);

/**
 * @brief 初始化控制器、调试接口、模拟前端和ADC触发
 * @note FOC_Data_Init须在MX_TIM1_Init之前调用，保证定时器参数有效。
 * @note 外设初始化完成后调用一次；成功返回后先执行电流零偏校准，
 *       校准后保持IDLE，由Process响应串口run启动请求并开启PWM。
 */
HAL_StatusTypeDef MotorApp_Init(void);

/**
 * @brief 主循环任务：更新规则组ADC并处理串口启动请求和调试命令
 */
void MotorApp_Process(void);

/**
 * @brief ADC注入组完成回调中的25kHz电机控制入口
 * @param hadc HAL回调传入的ADC句柄；只处理ADC1，忽略ADC2和空指针。
 */
void MotorApp_OnInjectedConversion(ADC_HandleTypeDef *hadc);

#ifdef __cplusplus
}
#endif

#endif
