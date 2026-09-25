#include "motor_protocol.h"

#include "app_memory.h"
#include "foc_math.h"
#include "motor_calibration.h"
#include "tim.h"
#include <math.h>
#include <string.h>

#define MOTOR_PROTOCOL_BOOT_MAGIC       0x544F4F42UL /* Bootloader跳转魔数。 */
#define MOTOR_PROTOCOL_CONFIG_NODE_ADDR 0x0801F808UL /* Flash中的节点号配置地址。 */
#define MOTOR_PROTOCOL_BROADCAST_MASK   0xFFU /* 控制帧中的广播节点掩码。 */
#define MOTOR_PROTOCOL_RX_RING_SIZE     8U /* ISR到主循环的接收环形队列深度。 */
#define MOTOR_PROTOCOL_SLOT_COUNT       10U /* TIM6调度时隙总数。 */
#define MOTOR_PROTOCOL_DEBUG_DIVIDER    2U /* 调试帧相对TIM6节拍的分频。 */
#define MOTOR_PROTOCOL_BOOT_ACK_TIMEOUT_MS 20U /* Boot应答发送等待上限。 */
#define MOTOR_PROTOCOL_HELLO_TX_TIMEOUT_MS  10U /* HELLO物理发送等待上限。 */
#define MOTOR_PROTOCOL_BUS_CURRENT_MAX_A    10.0f
#define MOTOR_PROTOCOL_BUS_CURRENT_SCALE \
  (65535.0f / MOTOR_PROTOCOL_BUS_CURRENT_MAX_A)
#define MOTOR_PROTOCOL_TEMPERATURE_MIN_C   (-20.0f)
#define MOTOR_PROTOCOL_TEMPERATURE_MAX_C   150.0f
#define MOTOR_PROTOCOL_TEMPERATURE_SCALE \
  (65535.0f / (MOTOR_PROTOCOL_TEMPERATURE_MAX_C - \
               MOTOR_PROTOCOL_TEMPERATURE_MIN_C))
/* 基础运行反馈：每个节点由TIM6错开发送普通反馈，当前目标为100 Hz。 */
#define MOTOR_PROTOCOL_PERIODIC_FD_FEEDBACK_ENABLED 1U
/* 每秒发送一次Classic CAN心跳，便于上位机判断节点在线。 */
#define MOTOR_PROTOCOL_HEARTBEAT_ENABLED 1U

typedef struct {
  FDCAN_RxHeaderTypeDef header; /* ISR接收的完整FDCAN头。 */
  uint8_t data[64]; /* 按最大CAN FD帧保留的载荷。 */
} MotorProtocol_RxItem_t;

typedef struct {
  FDCAN_HandleTypeDef *fdcan; /* 绑定的FDCAN外设。 */
  FOC_Control_t *control; /* 协议命令写入的FOC控制器。 */
  uint8_t node_id; /* 本节点号，参与所有节点相关ID计算。 */
  volatile uint8_t feedback_sequence; /* 普通反馈的递增序号。 */
  volatile uint8_t debug_enabled; /* 本节点是否发送调试反馈。 */
  volatile uint8_t debug_suppressed; /* 调试选择期间是否抑制普通反馈。 */
  volatile uint8_t timer_slot; /* 当前TIM6调度时隙。 */
  volatile uint8_t debug_divider; /* 调试反馈分频计数。 */
  MotorCalibrationState_t calibration_reported_state;
  volatile uint8_t rx_head; /* ISR生产者索引。 */
  volatile uint8_t rx_tail; /* 主循环消费者索引。 */
  uint32_t heartbeat_next_tick; /* 下一次心跳的HAL tick。 */
  MotorProtocol_RxItem_t rx_ring[MOTOR_PROTOCOL_RX_RING_SIZE]; /* 接收快照队列。 */
} MotorProtocol_Context_t;

static MotorProtocol_Context_t motor_protocol;

/**
 * @brief 计算电机CAN控制帧使用的CRC8校验值。
 */
static uint8_t MotorProtocol_CRC8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0U; /* CRC-8/多项式0x07的初值。 */
  uint8_t bit; /* 当前字节内的位计数。 */

  while (len-- != 0U) {
    crc ^= *data++;
    for (bit = 0U; bit < 8U; ++bit) {
      crc = ((crc & 0x80U) != 0U)
                ? (uint8_t)((crc << 1U) ^ 0x07U)
                : (uint8_t)(crc << 1U);
    }
  }
  return crc;
}

/**
 * @brief 按指定比例把浮点量转换为有符号16位协议数据，并进行饱和保护。
 */
static int16_t MotorProtocol_S16(float value, float scale)
{
  float scaled = value * scale;
  if (scaled > 32767.0f) {
    return 32767;
  }
  if (scaled < -32768.0f) {
    return -32768;
  }
  return (int16_t)scaled;
}

/**
 * @brief 按指定比例把浮点量转换为无符号16位协议数据，并进行范围保护。
 */
static uint16_t MotorProtocol_U16(float value, float scale)
{
  float scaled = value * scale;
  if (scaled < 0.0f) {
    return 0U;
  }
  if (scaled > 65535.0f) {
    return 65535U;
  }
  return (uint16_t)scaled;
}

/**
 * @brief 把浮点量转换为有符号8位协议数据，用于低精度状态反馈。
 */
static int8_t MotorProtocol_S8(float value)
{
  if (value > 127.0f) {
    return 127;
  }
  if (value < -128.0f) {
    return -128;
  }
  return (int8_t)value;
}

/**
 * @brief 以小端格式把有符号16位数据写入协议发送缓冲区。
 */
static void MotorProtocol_PutS16(uint8_t *dst, int16_t value)
{
  memcpy(dst, &value, sizeof(value));
}

/**
 * @brief 以小端格式把无符号16位数据写入协议发送缓冲区。
 */
static void MotorProtocol_PutU16(uint8_t *dst, uint16_t value)
{
  memcpy(dst, &value, sizeof(value));
}

/**
 * @brief 按照指定ID、长度和CAN/CAN FD格式发送一帧电机协议数据。
 */
static HAL_StatusTypeDef MotorProtocol_Send(uint32_t identifier,
                                            uint32_t data_length,
                                            uint32_t fd_format,
                                            uint8_t *data)
{
  FDCAN_TxHeaderTypeDef header = {0};
  HAL_StatusTypeDef status;
  uint8_t timer_irq_locked = 0U;

  /* 主循环和TIM6中断都会发送，避免同时操作HAL的Tx FIFO索引。 */
  if (__get_IPSR() == 0U) {
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
    timer_irq_locked = 1U;
  }

  if ((motor_protocol.fdcan == NULL) ||
      (HAL_FDCAN_GetTxFifoFreeLevel(motor_protocol.fdcan) == 0U)) {
    if (timer_irq_locked != 0U) {
      HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }
    return HAL_BUSY;
  }

  header.Identifier = identifier;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = data_length;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = ((fd_format == FDCAN_FD_CAN) &&
                          (MOTOR_PROTOCOL_CANFD_BRS_ENABLED != 0U))
                             ? FDCAN_BRS_ON
                             : FDCAN_BRS_OFF;
  header.FDFormat = fd_format;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0U;

  status = HAL_FDCAN_AddMessageToTxFifoQ(motor_protocol.fdcan, &header, data);
  if (timer_irq_locked != 0U) {
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  }
  return status;
}

#define MOTOR_PROTOCOL_IDENTIFY_STATUS_ACCEPTED  0U
#define MOTOR_PROTOCOL_IDENTIFY_STATUS_DONE      1U
#define MOTOR_PROTOCOL_IDENTIFY_STATUS_BUSY      2U
#define MOTOR_PROTOCOL_IDENTIFY_STATUS_REJECTED  3U
#define MOTOR_PROTOCOL_IDENTIFY_STATUS_ABORTED   4U

static void MotorProtocol_SendIdentificationResponse(uint8_t status)
{
  uint8_t data[8] = {0};
  float result_ohm = MotorCalibration_GetResultOhm();
  uint16_t result_mohm;

  if ((!isfinite(result_ohm)) || (result_ohm <= 0.0f)) {
    result_mohm = 0U;
  } else if (result_ohm >= 65.535f) {
    result_mohm = 65535U;
  } else {
    result_mohm = (uint16_t)(result_ohm * 1000.0f + 0.5f);
  }

  data[0] = motor_protocol.node_id;
  data[1] = MOTOR_PROTOCOL_CMD_MOTOR_IDENTIFY;
  data[2] = status;
  data[3] = MotorCalibration_GetErrorCode();
  MotorProtocol_PutU16(&data[4], result_mohm);
  data[6] = motor_protocol.feedback_sequence++;
  data[7] = MotorProtocol_CRC8(data, 7U);

  (void)MotorProtocol_Send(MOTOR_PROTOCOL_ID_RESPONSE_BASE +
                               motor_protocol.node_id,
                           FDCAN_DLC_BYTES_8,
                           FDCAN_CLASSIC_CAN,
                           data);
}

static void MotorProtocol_HandleIdentification(uint8_t action)
{
  HAL_StatusTypeDef result;

  if (action != 0U) {
    result = MotorCalibration_Start();
    motor_protocol.calibration_reported_state =
        MotorCalibration_GetState();
    if (result == HAL_OK) {
      MotorProtocol_SendIdentificationResponse(
          MOTOR_PROTOCOL_IDENTIFY_STATUS_ACCEPTED);
    } else if (result == HAL_BUSY) {
      MotorProtocol_SendIdentificationResponse(
          MOTOR_PROTOCOL_IDENTIFY_STATUS_BUSY);
    } else {
      MotorProtocol_SendIdentificationResponse(
          MOTOR_PROTOCOL_IDENTIFY_STATUS_REJECTED);
    }
  } else {
    MotorCalibration_Stop();
    MotorProtocol_SendIdentificationResponse(
        MOTOR_PROTOCOL_IDENTIFY_STATUS_ACCEPTED);
  }
}

/**
 * @brief 上电后先发送经典CAN HELLO，确认APP已启动且CAN物理链路可通信。
 * @note 在周期反馈定时器启动前调用，避免CAN FD反馈抢先占用总线。
 */
static void MotorProtocol_SendPowerOnHello(void)
{
  uint8_t data[8] = {'H', 'E', 'L', 'L', 'O', 0U, 1U, 0U};
  uint32_t tx_request;
  uint32_t start_tick;

  data[5] = motor_protocol.node_id;
  data[7] = MotorProtocol_CRC8(data, 7U);
  if (MotorProtocol_Send(MOTOR_PROTOCOL_ID_HELLO_BASE +
                             motor_protocol.node_id,
                         FDCAN_DLC_BYTES_8, FDCAN_CLASSIC_CAN,
                         data) != HAL_OK) {
    return;
  }

  /* 等待物理发送完成；总线异常时限时退出，不阻塞电机主循环。 */
  tx_request = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(motor_protocol.fdcan);
  start_tick = HAL_GetTick();
  while (((motor_protocol.fdcan->Instance->TXBTO & tx_request) == 0U) &&
         ((HAL_GetTick() - start_tick) < MOTOR_PROTOCOL_HELLO_TX_TIMEOUT_MS)) {
  }
}

/**
 * @brief 更新运行请求并同步FOC状态机，负责启动、停止时的安全处理。
 */
static void MotorProtocol_SetRun(uint8_t run)
{
  if (foc_motor_state == FOC_MOTOR_CALIBRATION) {
    if (run == 0U) {
      MotorCalibration_Stop();
    }
    return;
  }

  if (run == 0U) {
    foc_motor_state = FOC_MOTOR_IDLE;
    FOC_PWM_Stop();
    FOC_Control_Reset(motor_protocol.control);
    return;
  }

  if ((run == 1U) && (foc_motor_state == FOC_MOTOR_IDLE)) {
    FOC_Control_Reset(motor_protocol.control);
    FOC_Control_EnableSpeedLoop(motor_protocol.control, 1U);
    foc_motor_state = FOC_MOTOR_OPEN_LOOP;
    FOC_PWM_Start();
  }
}

/**
 * @brief 按普通模式周期发送当前电机的简要状态反馈。
 */
static void MotorProtocol_SendNormalFeedback(void)
{
  uint8_t data[12] = {0}; /* 普通反馈固定12字节，字段采用小端序。 */
  uint8_t flags = 0U; /* Byte9状态位：校准、速度环和SVPWM限幅。 */

  /* Byte0..1：机械转速，rpm，S16*1。 */
  MotorProtocol_PutS16(&data[0],
                       MotorProtocol_S16(foc.observer.state.speed_rpm, 1.0f));
  /* Byte2..3：母线电流，0..10 A映射到无符号16位。 */
  MotorProtocol_PutU16(&data[2],
                       MotorProtocol_U16(foc.state.ibus_filter,
                                         MOTOR_PROTOCOL_BUS_CURRENT_SCALE));
  /* Byte4..5：母线电压，单位 V，比例100。 */
  MotorProtocol_PutU16(&data[4],
                       MotorProtocol_U16(foc.state.vbus, 100.0f));
  /* Byte6..7：温度，-20..150 C线性映射到无符号16位。 */
  MotorProtocol_PutU16(&data[6],
                       MotorProtocol_U16(foc.state.temperature_c -
                                             MOTOR_PROTOCOL_TEMPERATURE_MIN_C,
                                         MOTOR_PROTOCOL_TEMPERATURE_SCALE));

  if (foc.calibration.calibrated != 0U) {
    flags |= 0x01U;
  }
  if (motor_protocol.control->speed_loop_enable != 0U) {
    flags |= 0x02U;
  }
  if (foc.svpwm.limited != 0U) {
    flags |= 0x04U;
  }

  data[8] = (uint8_t)foc_motor_state; /* 状态枚举。 */
  data[9] = flags; /* 运行状态位。 */
  data[10] = motor_protocol.feedback_sequence++; /* 反馈序号，8位回绕。 */
  data[11] = 0U; /* 预留字节，保持协议长度稳定。 */

  (void)MotorProtocol_Send(MOTOR_PROTOCOL_ID_FEEDBACK_BASE +
                               motor_protocol.node_id,
                           MOTOR_PROTOCOL_FEEDBACK_DLC,
                           FDCAN_FD_CAN,
                           data);
}

/**
 * @brief 向被选中的调试节点发送高速详细反馈，其余节点保持静默。
 */
static void MotorProtocol_SendDebugFeedback(void)
{
  uint8_t data[64] = {0}; /* 调试帧按64字节CAN FD载荷发送。 */

  MotorProtocol_PutS16(&data[0],
                       MotorProtocol_S16(foc.observer.state.speed_rpm, 1.0f));
  MotorProtocol_PutS16(&data[2],
                       MotorProtocol_S16(foc.observer.state.pll_omega_e, 1.0f));
  MotorProtocol_PutS16(&data[4], MotorProtocol_S16(foc.state.i_abc.a, 100.0f));
  MotorProtocol_PutS16(&data[6], MotorProtocol_S16(foc.state.i_abc.b, 100.0f));
  MotorProtocol_PutS16(&data[8], MotorProtocol_S16(foc.state.i_abc.c, 100.0f));
  MotorProtocol_PutS16(&data[10], MotorProtocol_S16(foc.state.i_dq.d, 100.0f));
  MotorProtocol_PutS16(&data[12], MotorProtocol_S16(foc.state.i_dq.q, 100.0f));
  MotorProtocol_PutS16(&data[14], MotorProtocol_S16(foc.state.u_dq.d, 100.0f));
  MotorProtocol_PutS16(&data[16], MotorProtocol_S16(foc.state.u_dq.q, 100.0f));
  MotorProtocol_PutU16(&data[18], MotorProtocol_U16(foc.state.vbus, 100.0f));
  MotorProtocol_PutS16(&data[20],
                       MotorProtocol_S16(foc.state.temperature_c, 10.0f));

  MotorProtocol_PutS16(
      &data[22],
      MotorProtocol_S16(foc.observer.state.phase_raw * RAD_TO_DEG_F, 100.0f));
  data[24] = (uint8_t)foc_motor_state;
  data[25] = (foc.calibration.calibrated != 0U) ? 0x01U : 0U;
  data[26] = (motor_protocol.control->speed_loop_enable != 0U) ? 0x02U : 0U;
  data[27] = (foc.svpwm.limited != 0U) ? 0x04U : 0U;
  data[28] = motor_protocol.feedback_sequence++;

  (void)MotorProtocol_Send(MOTOR_PROTOCOL_ID_DEBUG_BASE +
                               motor_protocol.node_id,
                           MOTOR_PROTOCOL_DEBUG_DLC,
                           FDCAN_FD_CAN,
                           data);
}

/**
 * @brief 发送本节点低频在线心跳，供上位机判断节点是否在线。
 */
static void MotorProtocol_SendHeartbeat(void)
{
  uint8_t data[8] = {0}; /* Classic CAN心跳固定8字节，末字节为CRC。 */

  data[0] = motor_protocol.node_id;
  data[1] = (uint8_t)foc_motor_state;
  data[2] = (foc.calibration.calibrated != 0U) ? 1U : 0U;
  data[3] = motor_protocol.debug_enabled;
  data[4] = (uint8_t)MotorProtocol_S8(foc.state.temperature_c);
  data[5] = (uint8_t)foc.svpwm.limited;
  data[6] = motor_protocol.feedback_sequence;
  data[7] = MotorProtocol_CRC8(data, 7U);

  (void)MotorProtocol_Send(MOTOR_PROTOCOL_ID_HEARTBEAT_BASE +
                               motor_protocol.node_id,
                           FDCAN_DLC_BYTES_8,
                           FDCAN_CLASSIC_CAN,
                           data);
}

/**
 * @brief 校验进入Bootloader命令，停止PWM、写入备份寄存器并触发系统复位。
 */
static void MotorProtocol_HandleBoot(const uint8_t data[8])
{
  if ((data[0] != motor_protocol.node_id) &&
      (data[0] != MOTOR_PROTOCOL_BROADCAST_MASK)) {
    return;
  }
  if ((data[1] != MOTOR_PROTOCOL_CMD_ENTER_BOOT) ||
      (MotorProtocol_CRC8(data, 7U) != data[7])) {
    return;
  }

#if APP_WITH_BOOTLOADER
  uint8_t ack[8] = {0};
  uint32_t tx_request;
  uint32_t start_tick;

  foc_motor_state = FOC_MOTOR_IDLE;
  FOC_PWM_Stop();
  /* 停止周期反馈，避免ACK排队期间继续占用发送FIFO。 */
  (void)HAL_TIM_Base_Stop_IT(&htim6);

  /* APP确认帧：0x180+NodeID，Byte2=1表示命令已通过校验。 */
  ack[0] = motor_protocol.node_id;
  ack[1] = MOTOR_PROTOCOL_CMD_ENTER_BOOT;
  ack[2] = 1U;
  ack[7] = MotorProtocol_CRC8(ack, 7U);

  start_tick = HAL_GetTick();
  while ((HAL_FDCAN_GetTxFifoFreeLevel(motor_protocol.fdcan) == 0U) &&
         ((HAL_GetTick() - start_tick) < MOTOR_PROTOCOL_BOOT_ACK_TIMEOUT_MS)) {
  }

  if (HAL_FDCAN_GetTxFifoFreeLevel(motor_protocol.fdcan) != 0U &&
      MotorProtocol_Send(MOTOR_PROTOCOL_ID_RESPONSE_BASE +
                             motor_protocol.node_id,
                         FDCAN_DLC_BYTES_8, FDCAN_CLASSIC_CAN,
                         ack) == HAL_OK) {
    /* 入队不等于已上总线；等待本次发送完成后再复位。 */
    tx_request = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(motor_protocol.fdcan);
    start_tick = HAL_GetTick();
    while (((motor_protocol.fdcan->Instance->TXBTO & tx_request) == 0U) &&
           ((HAL_GetTick() - start_tick) < MOTOR_PROTOCOL_BOOT_ACK_TIMEOUT_MS)) {
    }
  }

  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
#if defined(__HAL_RCC_RTCAPB_CLK_ENABLE)
  __HAL_RCC_RTCAPB_CLK_ENABLE();
#endif
  TAMP->BKP0R = MOTOR_PROTOCOL_BOOT_MAGIC;
  __DSB();
  __ISB();
  NVIC_SystemReset();
  while (1) {
  }
#endif
}

/**
 * @brief 解析广播的多节点转速向量，并更新本节点的目标转速。
 */
static void MotorProtocol_HandleVector(const uint8_t data[24])
{
  uint8_t mask;
  uint8_t index;

  if (data[0] != 1U) {
    return;
  }

  /* 调试选择是广播管理命令，所有节点都必须执行清除/选中动作。 */
  if (data[1] == MOTOR_PROTOCOL_CMD_DEBUG_SELECT) {
    uint8_t debug_mask = data[2];
    uint8_t valid_selection =
        ((debug_mask != 0U) &&
         ((debug_mask & (uint8_t)(debug_mask - 1U)) == 0U) &&
         (data[3] != 0U))
            ? 1U
            : 0U;
    motor_protocol.debug_suppressed = valid_selection;
    motor_protocol.debug_enabled =
        ((valid_selection != 0U) &&
         ((debug_mask & (uint8_t)(1U << (motor_protocol.node_id - 1U))) != 0U))
            ? 1U
            : 0U;
    motor_protocol.debug_divider = 0U;
    return;
  }

  mask = data[2];
  if (mask == 0U) {
    return;
  }
  if ((mask & (uint8_t)(1U << (motor_protocol.node_id - 1U))) == 0U) {
    return;
  }

  if ((data[1] == MOTOR_PROTOCOL_CMD_SPEED_VECTOR) ||
      (data[1] == MOTOR_PROTOCOL_CMD_RUN_VECTOR)) {
    index = (uint8_t)(motor_protocol.node_id - 1U);
    if (data[1] == MOTOR_PROTOCOL_CMD_SPEED_VECTOR) {
      int16_t speed_rpm;
      memcpy(&speed_rpm, &data[8U + (2U * index)], sizeof(speed_rpm));
      if ((speed_rpm < -10000) || (speed_rpm > 10000)) {
        /* 非法速度不更新给定，也不执行同一帧中的启动请求。 */
        return;
      }
      motor_protocol.control->speed_ref_rpm = (float)speed_rpm;
      FOC_Control_EnableSpeedLoop(motor_protocol.control, 1U);
    }

    if ((data[3] & (uint8_t)(1U << index)) != 0U) {
      MotorProtocol_SetRun(1U);
    } else if (data[1] == MOTOR_PROTOCOL_CMD_RUN_VECTOR) {
      MotorProtocol_SetRun(0U);
    }
  } else if (data[1] == MOTOR_PROTOCOL_CMD_MOTOR_IDENTIFY) {
    MotorProtocol_HandleIdentification(data[3]);
  } else if (data[1] == MOTOR_PROTOCOL_CMD_STATUS_ONCE) {
    MotorProtocol_SendNormalFeedback();
  }
}

/**
 * @brief 兼容旧版单节点控制帧，转换为当前控制状态。
 */
static void MotorProtocol_HandleLegacy(const uint8_t data[8])
{
  if ((data[0] != motor_protocol.node_id) &&
      (data[0] != MOTOR_PROTOCOL_BROADCAST_MASK)) {
    return;
  }
  if (MotorProtocol_CRC8(data, 7U) != data[7]) {
    return;
  }

  if (data[1] == 0x01U) {
    MotorProtocol_SetRun(data[3]);
  } else if (data[1] == 0x02U) {
    float speed_rpm;
    memcpy(&speed_rpm, &data[3], sizeof(speed_rpm));
    if ((speed_rpm == speed_rpm) &&
        (speed_rpm >= -10000.0f) && (speed_rpm <= 10000.0f)) {
      motor_protocol.control->speed_ref_rpm = speed_rpm;
      FOC_Control_EnableSpeedLoop(motor_protocol.control, 1U);
    }
  } else if (data[1] == MOTOR_PROTOCOL_CMD_MOTOR_IDENTIFY) {
    MotorProtocol_HandleIdentification(data[3]);
  }
}

/**
 * @brief 根据接收帧类型分发到Bootloader、速度控制或调试命令处理函数。
 */
static void MotorProtocol_HandleRx(const MotorProtocol_RxItem_t *item)
{
  const FDCAN_RxHeaderTypeDef *header = &item->header;

  if ((header->IdType != FDCAN_STANDARD_ID) ||
      (header->RxFrameType != FDCAN_DATA_FRAME)) {
    return;
  }

  if (header->Identifier == MOTOR_PROTOCOL_ID_BOOT) {
    if ((header->FDFormat == FDCAN_CLASSIC_CAN) &&
        (header->DataLength == FDCAN_DLC_BYTES_8)) {
      MotorProtocol_HandleBoot(item->data);
    }
  } else if (header->Identifier == MOTOR_PROTOCOL_ID_CONTROL) {
    if ((header->FDFormat == FDCAN_FD_CAN) &&
        (header->DataLength == MOTOR_PROTOCOL_CONTROL_DLC)) {
      MotorProtocol_HandleVector(item->data);
    } else if ((header->FDFormat == FDCAN_CLASSIC_CAN) &&
               (header->DataLength == FDCAN_DLC_BYTES_8)) {
      MotorProtocol_HandleLegacy(item->data);
    }
  }
}

/**
 * @brief 在FDCAN接收中断中快速搬运数据到软件环形缓冲区，避免中断内执行控制逻辑。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
  uint8_t next_head;

  if ((hfdcan != motor_protocol.fdcan) ||
      ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)) {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
    next_head = (uint8_t)((motor_protocol.rx_head + 1U) %
                          MOTOR_PROTOCOL_RX_RING_SIZE);
    if (next_head == motor_protocol.rx_tail) {
      /* 队列满时丢弃新帧；生产者不修改消费者索引，避免ISR竞争。 */
      if (HAL_FDCAN_GetRxMessage(
              hfdcan,
              FDCAN_RX_FIFO0,
              &motor_protocol.rx_ring[motor_protocol.rx_head].header,
              motor_protocol.rx_ring[motor_protocol.rx_head].data) != HAL_OK) {
        return;
      }
      continue;
    }
    if (HAL_FDCAN_GetRxMessage(hfdcan,
                               FDCAN_RX_FIFO0,
                               &motor_protocol.rx_ring[motor_protocol.rx_head].header,
                               motor_protocol.rx_ring[motor_protocol.rx_head].data)
        != HAL_OK) {
      return;
    }
    __DMB();
    motor_protocol.rx_head = next_head;
  }
}

/**
 * @brief 初始化电机协议上下文、FDCAN过滤器、接收中断和反馈调度定时器。
 */
HAL_StatusTypeDef MotorProtocol_Init(FDCAN_HandleTypeDef *hfdcan,
                                     FOC_Control_t *control)
{
  FDCAN_FilterTypeDef filter = {0};
  uint8_t node_id;

  if ((hfdcan == NULL) || (control == NULL)) {
    return HAL_ERROR;
  }

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID2 = 0x7FFU;

  filter.FilterIndex = 0U;
  filter.FilterID1 = MOTOR_PROTOCOL_ID_BOOT;
  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) {
    return HAL_ERROR;
  }
  filter.FilterIndex = 1U;
  filter.FilterID1 = MOTOR_PROTOCOL_ID_CONTROL;
  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) {
    return HAL_ERROR;
  }
  if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK) {
    return HAL_ERROR;
  }

  node_id = *(const volatile uint8_t *)MOTOR_PROTOCOL_CONFIG_NODE_ADDR;
  motor_protocol.node_id =
      ((node_id >= MOTOR_PROTOCOL_NODE_MIN) &&
       (node_id <= MOTOR_PROTOCOL_NODE_MAX))
          ? node_id
          : MOTOR_PROTOCOL_NODE_MIN;
  motor_protocol.fdcan = hfdcan;
  motor_protocol.control = control;
  motor_protocol.rx_head = 0U;
  motor_protocol.rx_tail = 0U;
  motor_protocol.feedback_sequence = 0U;
  motor_protocol.debug_enabled = 0U;
  motor_protocol.debug_suppressed = 0U;
  motor_protocol.timer_slot = 0U;
  motor_protocol.debug_divider = 0U;
  motor_protocol.calibration_reported_state = MOTOR_CALIBRATION_IDLE;
  motor_protocol.heartbeat_next_tick =
      HAL_GetTick() + 1000U + ((uint32_t)motor_protocol.node_id * 10U);

  if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
    return HAL_ERROR;
  }
  if (HAL_FDCAN_ActivateNotification(
          hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK) {
    return HAL_ERROR;
  }
  MotorProtocol_SendPowerOnHello();
  return HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief 处理协议调度时基，按时隙安排心跳、普通反馈和调试反馈发送。
 */
void MotorProtocol_TimerTick(TIM_HandleTypeDef *htim)
{
  uint8_t current_slot;

  if ((htim != &htim6) || (motor_protocol.fdcan == NULL)) {
    return;
  }
  if (MOTOR_PROTOCOL_PERIODIC_FD_FEEDBACK_ENABLED == 0U) return;

  current_slot = motor_protocol.timer_slot;
  motor_protocol.timer_slot =
      (uint8_t)((motor_protocol.timer_slot + 1U) % MOTOR_PROTOCOL_SLOT_COUNT);

  /* 调试期间未被选中的节点只保留心跳，不发送普通反馈。 */
  if (motor_protocol.debug_suppressed != 0U) {
    if (motor_protocol.debug_enabled != 0U) {
      motor_protocol.debug_divider++;
      if (motor_protocol.debug_divider >= MOTOR_PROTOCOL_DEBUG_DIVIDER) {
        motor_protocol.debug_divider = 0U;
        MotorProtocol_SendDebugFeedback();
      }
    }
    return;
  }

  if (current_slot == (uint8_t)(motor_protocol.node_id - 1U)) {
    MotorProtocol_SendNormalFeedback();
  }
}

/**
 * @brief 接收TIM6周期中断并转发给电机协议调度器。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if ((htim != NULL) && (htim->Instance == TIM1)) {
    MotorCalibration_LsTim1Update();
    return;
  }
  MotorProtocol_TimerTick(htim);
}

/**
 * @brief 在主循环中消费接收环形缓冲区并执行控制命令。
 */
void MotorProtocol_Process(void)
{
  MotorProtocol_RxItem_t item;
  uint32_t now;

  if ((motor_protocol.fdcan == NULL) || (motor_protocol.control == NULL)) {
    return;
  }

  while (motor_protocol.rx_tail != motor_protocol.rx_head) {
    item = motor_protocol.rx_ring[motor_protocol.rx_tail];
    __DMB();
    motor_protocol.rx_tail =
        (uint8_t)((motor_protocol.rx_tail + 1U) % MOTOR_PROTOCOL_RX_RING_SIZE);
    MotorProtocol_HandleRx(&item);
  }

  {
    MotorCalibrationState_t state = MotorCalibration_GetState();
    if ((state == MOTOR_CALIBRATION_DONE) &&
        (motor_protocol.calibration_reported_state !=
         MOTOR_CALIBRATION_DONE)) {
      MotorProtocol_SendIdentificationResponse(
          MOTOR_PROTOCOL_IDENTIFY_STATUS_DONE);
      motor_protocol.calibration_reported_state = state;
    } else if ((state == MOTOR_CALIBRATION_ERROR) &&
               (motor_protocol.calibration_reported_state !=
                MOTOR_CALIBRATION_ERROR)) {
      MotorProtocol_SendIdentificationResponse(
          (MotorCalibration_GetErrorCode() == 1U) ?
              MOTOR_PROTOCOL_IDENTIFY_STATUS_ABORTED :
              MOTOR_PROTOCOL_IDENTIFY_STATUS_REJECTED);
      motor_protocol.calibration_reported_state = state;
    }
  }

  #if MOTOR_PROTOCOL_HEARTBEAT_ENABLED
    now = HAL_GetTick();

    if ((int32_t)(now - motor_protocol.heartbeat_next_tick) >= 0) {
        motor_protocol.heartbeat_next_tick += 1000U;
        MotorProtocol_SendHeartbeat();
    }
  #endif
}
