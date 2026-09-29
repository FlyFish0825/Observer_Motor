#ifndef MOTOR_APP_H
#define MOTOR_APP_H

#include "stm32g4xx_hal.h"
#include "controller.h"

/* 初始化控制器、调试、模拟前端和零偏校准。 */
HAL_StatusTypeDef MotorApp_Init(void);
/* EXTI中只请求按键操作并关闭MOE；耗时启停在主循环完成。 */
void MotorApp_RequestButton(uint16_t pin);
/* 主循环低优先级业务，不在实时中断中调用。 */
void MotorApp_Process(void);
/* ADC1注入回调的25kHz实时入口，保留Rs辨识分支。 */
void MotorApp_OnInjectedConversion(ADC_HandleTypeDef *hadc);
/* CAN协议共用同一个控制器，不建立参数副本。 */
FOC_Control_t *MotorApp_GetControl(void);

#endif
