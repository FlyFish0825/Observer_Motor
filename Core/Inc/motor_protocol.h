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

/* 当前板卡的CAN收发器只支持1 Mbit/s；CAN FD当前关闭BRS。 */
#define MOTOR_PROTOCOL_CANFD_BRS_ENABLED    0U

/* 0x100广播控制帧命令。 */
#define MOTOR_PROTOCOL_CMD_SPEED_VECTOR     0x10U
#define MOTOR_PROTOCOL_CMD_RUN_VECTOR       0x11U
#define MOTOR_PROTOCOL_CMD_DEBUG_SELECT     0x20U
#define MOTOR_PROTOCOL_CMD_STATUS_ONCE      0x30U

/* 0x000上的APP管理命令。 */
#define MOTOR_PROTOCOL_CMD_ENTER_BOOT       0x04U

/**
 * @brief 初始化FDCAN协议：配置过滤器、读取节点号、启动CAN并发送HELLO。
 * @param hfdcan  CubeMX生成的FDCAN句柄指针。
 * @param control 电机控制器指针，协议层通过它写入速度命令。
 * @return HAL_OK初始化成功；HAL_ERROR参数无效或FDCAN配置失败。
 */
HAL_StatusTypeDef MotorProtocol_Init(FDCAN_HandleTypeDef *hfdcan,
                                     FOC_Control_t *control);
/**
 * @brief 主循环周期调用：消费CAN接收中断环形队列并发送低频心跳。
 * @note 控制命令只在主循环解析；周期反馈由TIM6中断调度。
 */
void MotorProtocol_Process(void);
/**
 * @brief 处理TIM6的1 ms节拍，调度普通或高速CAN反馈。
 * @param htim HAL定时器句柄；仅处理TIM6。
 */
void MotorProtocol_TimerTick(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PROTOCOL_H */
