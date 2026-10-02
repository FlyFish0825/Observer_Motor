#ifndef MOTOR_APP_H
#define MOTOR_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "controller.h"

/**
 * @file motor_app.h
 * @brief 电机应用层接口：上电安全、初始化、主循环任务与25 kHz控制中断入口。
 * 本模块是唯一同时接触"主循环域"和"ADC中断域"的模块，启动状态机、
 * 运行请求仲裁与故障闩锁都集中在这里。
 * 控制算法本身在controller.c/observer.c/foc_math.c，本层负责决定
 * 每个状态用什么控制角和什么dq电流参考。
 */

/**
 * @brief HAL初始化前将六路栅极驱动输出置于安全低电平。
 * @note 在main()最开头调用，防止上电时PWM引脚不确定状态误导通MOSFET。
 *       后续由CubeMX生成的GPIO/TIM初始化重新配置为复用功能。
 *       此时外设时钟尚未配置，因此直接写GPIO寄存器而不走HAL。
 */
void MotorApp_ForcePowerStageSafe(void);

/**
 * @brief 初始化控制器、调试接口、模拟前端和ADC触发
 * @note FOC_Data_Init须在MX_TIM1_Init之前调用，保证定时器参数有效。
 * @note 外设初始化完成后调用一次；成功返回后先执行电流零偏校准，
 *       校准后保持IDLE，由Process响应串口run启动请求并开启PWM。
 */
/** @return HAL_OK初始化成功；HAL_ERROR任一步骤失败。 */
HAL_StatusTypeDef MotorApp_Init(void);

/**
 * @brief 主循环任务：更新规则组ADC并处理串口启动请求和调试命令
 */
/** @note 包含ADC轮询和串口解析，不可在中断中调用。 */
void MotorApp_Process(void);

/**
 * @brief ADC注入组完成回调中的25kHz电机控制入口
 * @param hadc HAL回调传入的ADC句柄；只处理ADC1，忽略ADC2和空指针。
 * @note 单拍预算40 µs，函数内完成零偏校准、观测器、状态机、电流环与SVPWM，
 *       并写TIM1->CCR1~3。不得在其中调用阻塞式HAL函数。
 */
void MotorApp_OnInjectedConversion(ADC_HandleTypeDef *hadc);

/**
 * @brief 获取应用层正在使用的FOC控制器，供CAN协议层更新目标参数。
 */
/** @return 指向应用层FOC控制器实例的指针，永不返回NULL。 */
FOC_Control_t *MotorApp_GetControl(void);

/**
 * @brief 设置运行请求；真正的启动/停机仍由MotorApp_Process和ADC控制节拍执行。
 * @param run 0=请求停机，非0=请求运行。
 * @note 只置请求，不做实际启停；故障闩锁后必须先发run 0再发run 1才能重启。
 */
void MotorApp_RequestRun(uint8_t run);

#ifdef __cplusplus
}
#endif

#endif
