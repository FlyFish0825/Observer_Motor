#ifndef MOTOR_PROTOCOL_H
#define MOTOR_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "controller.h"
#include "stm32g4xx_hal.h"

/* 固定节点协议：上位机预先知道节点号，不做发现和注册。 */
#define MOTOR_PROTOCOL_NODE_MIN             1U
#define MOTOR_PROTOCOL_NODE_MAX             8U

#define MOTOR_PROTOCOL_ID_BOOT              0x000U
#define MOTOR_PROTOCOL_ID_CONTROL           0x100U
#define MOTOR_PROTOCOL_ID_RESPONSE_BASE     0x180U
#define MOTOR_PROTOCOL_ID_FEEDBACK_BASE     0x200U
#define MOTOR_PROTOCOL_ID_HEARTBEAT_BASE    0x280U
#define MOTOR_PROTOCOL_ID_DEBUG_BASE        0x300U
#define MOTOR_PROTOCOL_ID_HELLO_BASE        0x380U

#define MOTOR_PROTOCOL_CONTROL_DLC          FDCAN_DLC_BYTES_24
#define MOTOR_PROTOCOL_FEEDBACK_DLC         FDCAN_DLC_BYTES_12
#define MOTOR_PROTOCOL_DEBUG_DLC            FDCAN_DLC_BYTES_64

/* 当前测试总线数据段也是1 Mbit/s，FD帧关闭BRS；切换8M时改为1。 */
#define MOTOR_PROTOCOL_CANFD_BRS_ENABLED    0U

/* 0x100广播控制帧命令。 */
#define MOTOR_PROTOCOL_CMD_SPEED_VECTOR     0x10U
#define MOTOR_PROTOCOL_CMD_RUN_VECTOR       0x11U
#define MOTOR_PROTOCOL_CMD_DEBUG_SELECT     0x20U
#define MOTOR_PROTOCOL_CMD_STATUS_ONCE      0x30U

/* 0x000上的APP管理命令。 */
#define MOTOR_PROTOCOL_CMD_ENTER_BOOT       0x04U

/* 调度器由TIM6的100 us更新中断调用。 */
void MotorProtocol_TimerTick(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef MotorProtocol_Init(FDCAN_HandleTypeDef *hfdcan,
                                     FOC_Control_t *control);
void MotorProtocol_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PROTOCOL_H */
