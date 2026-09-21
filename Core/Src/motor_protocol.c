/**
 * @file motor_protocol.c
 * @brief Observer_Motor 的 FDCAN 协议实现。
 *
 * 本文件属于应用层代码，负责节点地址读取、控制帧解析、反馈帧组包、
 * 接收环形队列以及周期调度。CubeMX 只负责生成 FDCAN 外设句柄和底层
 * 初始化代码；协议状态机和 PCB 相关行为均保留在本文件中，避免被重新
 * 生成的初始化代码覆盖。
 */
#include "motor_protocol.h"

#include "app_memory.h"
#include "foc_math.h"
#include "motor_app.h"
#include <string.h>

/* 写入备份寄存器、请求 Bootloader 接管时使用的魔数。 */
#define MOTOR_PROTOCOL_BOOT_MAGIC          0x544F4F42UL
/* 现有 PCB 的节点地址配置字节所在的 Flash 地址。 */
#define MOTOR_PROTOCOL_CONFIG_NODE_ADDR    0x0801F808UL
/* 协议中用于表示“所有节点”的广播地址。 */
#define MOTOR_PROTOCOL_BROADCAST_MASK      0xFFU
/* FDCAN FIFO0 到应用层之间的定长接收环形队列容量。 */
#define MOTOR_PROTOCOL_RX_RING_SIZE        8U
/* 周期反馈时隙数量，节点地址 1～10 分别占用一个时隙。 */
#define MOTOR_PROTOCOL_SLOT_COUNT          10U
/* 调试反馈相对于调度节拍的分频值。 */
#define MOTOR_PROTOCOL_DEBUG_DIVIDER       2U
/* Bootloader 应答帧允许等待发送完成的最长时间。 */
#define MOTOR_PROTOCOL_BOOT_ACK_TIMEOUT_MS 20U
/* 上电 HELLO 帧允许等待发送完成的最长时间。 */
#define MOTOR_PROTOCOL_HELLO_TIMEOUT_MS    10U
/* 1 表示保留main分支验证过的周期FD反馈；0可用于总线诊断时暂时停发。 */
#define MOTOR_PROTOCOL_PERIODIC_FD_FEEDBACK_ENABLED 1U

/* 协议状态码沿用main分支：IDLE=0、OPEN_LOOP=1、CLOSED_LOOP=2。
 * 当前CBT6应用只实现IDLE和CLOSED_LOOP，因此不能直接发送本地枚举值。 */
#define MOTOR_PROTOCOL_STATE_IDLE         0U
#define MOTOR_PROTOCOL_STATE_CLOSED_LOOP  2U

/** @brief 把当前PCB的本地电机状态映射为协议状态码。 */
static uint8_t MotorProtocol_StateCode(void) {
  /* 将本地状态压缩为上位机协议规定的有限状态集合。 */
  return (foc_motor_state == FOC_MOTOR_CLOSED_LOOP)
             ? MOTOR_PROTOCOL_STATE_CLOSED_LOOP
             : MOTOR_PROTOCOL_STATE_IDLE;
}

/** @brief 一项从HAL FIFO取出的接收帧及其完整数据缓存。 */
typedef struct {
  /* HAL 接收头，包含标准 ID、Classic/FD 格式和有效数据长度。 */
  FDCAN_RxHeaderTypeDef header;
  /* CAN FD 最大 64 字节数据区；Classic CAN 只使用前 8 字节。 */
  uint8_t data[64];
} MotorProtocol_RxItem_t;

/** @brief 协议模块运行时状态，主循环独占解析和发送。 */
typedef struct {
  /* CubeMX 生成并由应用层传入的 FDCAN 外设句柄。 */
  FDCAN_HandleTypeDef *fdcan;
  /* 电机控制器上下文，用于写入速度命令和查询运行状态。 */
  FOC_Control_t *control;
  /* 本节点的有效地址，初始化时从现有 PCB 的配置 Flash 地址读取。 */
  uint8_t node_id;
  /* 反馈帧序号，每发送一帧反馈后递增，溢出按 uint8_t 回绕。 */
  uint8_t feedback_sequence;
  /* 当前节点是否允许输出调试反馈。 */
  uint8_t debug_enabled;
  /* 是否处于“调试节点独占”模式；置位时抑制普通反馈。 */
  uint8_t debug_suppressed;
  /* 普通周期反馈使用的节点时隙计数器。 */
  uint8_t timer_slot;
  /* 调试反馈发送分频计数器。 */
  uint8_t debug_divider;
  /* 接收环形队列写指针。 */
  uint8_t rx_head;
  /* 接收环形队列读指针。 */
  uint8_t rx_tail;
  /* 上次运行调度器时的 HAL tick。 */
  uint32_t last_tick;
  /* 下一次发送 1 秒心跳帧的绝对 tick。 */
  uint32_t heartbeat_next_tick;
  /* 将 HAL FIFO 接收与主循环解析解耦的应用层环形队列。 */
  MotorProtocol_RxItem_t rx_ring[MOTOR_PROTOCOL_RX_RING_SIZE];
} MotorProtocol_Context_t;

/* 协议模块唯一的运行时上下文；所有字段均由本文件串行访问。 */
static MotorProtocol_Context_t motor_protocol;

/**
 * @brief 计算协议使用的 CRC-8。
 * @param data 待计算数据首地址。
 * @param len 参与计算的字节数，不包含帧尾 CRC 字节。
 * @return CRC-8 结果，使用多项式 0x07。
 */
static uint8_t MotorProtocol_CRC8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0U; /* CRC 累加寄存器。 */
  uint8_t bit;     /* 当前输入字节内的位循环计数。 */
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

/** @brief 将浮点量按比例量化为有符号 16 位值并饱和。 */
static int16_t MotorProtocol_S16(float value, float scale) {
  /* scaled保存量化后的中间值，先饱和再转换以避免窄类型溢出。 */
  float scaled = value * scale;
  if (scaled > 32767.0f) return 32767;
  if (scaled < -32768.0f) return -32768;
  return (int16_t)scaled;
}

/** @brief 将浮点量按比例量化为无符号 16 位值并饱和。 */
static uint16_t MotorProtocol_U16(float value, float scale) {
  /* 无符号字段的负值统一钳到零，防止转换产生大正数。 */
  float scaled = value * scale;
  if (scaled < 0.0f) return 0U;
  if (scaled > 65535.0f) return 65535U;
  return (uint16_t)scaled;
}

/** @brief 将浮点量量化为有符号 8 位值并饱和。 */
static int8_t MotorProtocol_S8(float value) {
  /* 8位状态或保留量使用有符号范围进行饱和。 */
  if (value > 127.0f) return 127;
  if (value < -128.0f) return -128;
  return (int8_t)value;
}

/** @brief 按 MCU 小端序把有符号 16 位值写入协议数据区。 */
static void MotorProtocol_PutS16(uint8_t *dst, int16_t value) {
  memcpy(dst, &value, sizeof(value));
}

/** @brief 按 MCU 小端序把无符号 16 位值写入协议数据区。 */
static void MotorProtocol_PutU16(uint8_t *dst, uint16_t value) {
  memcpy(dst, &value, sizeof(value));
}

/**
 * @brief 统一发送一帧 FDCAN 数据。
 * @note 发送前检查 FIFO 空间，避免在主循环中阻塞等待硬件队列。
 */
static HAL_StatusTypeDef MotorProtocol_Send(uint32_t identifier,
                                            uint32_t data_length,
                                            uint32_t fd_format,
                                            uint8_t *data) {
  /* header描述本次发送的标准ID、帧格式、DLC和位速率切换。 */
  FDCAN_TxHeaderTypeDef header = {0}; /* HAL 发送头，固定为标准数据帧。 */

  if ((motor_protocol.fdcan == NULL) ||
      (HAL_FDCAN_GetTxFifoFreeLevel(motor_protocol.fdcan) == 0U)) {
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
  return HAL_FDCAN_AddMessageToTxFifoQ(motor_protocol.fdcan, &header, data);
}

/** @brief 发送上电 HELLO 帧，并在短超时内确认硬件已完成发送。 */
static void MotorProtocol_SendPowerOnHello(void) {
  uint8_t data[8] = {'H', 'E', 'L', 'L', 'O', 0U, 1U, 0U}; /* HELLO 载荷。 */
  uint32_t request; /* HAL 返回的发送请求位。 */
  uint32_t start;   /* 发送确认超时起点。 */

  data[5] = motor_protocol.node_id;
  data[7] = MotorProtocol_CRC8(data, 7U);
  if (MotorProtocol_Send(MOTOR_PROTOCOL_ID_HELLO_BASE + motor_protocol.node_id,
                         FDCAN_DLC_BYTES_8, FDCAN_CLASSIC_CAN, data) != HAL_OK) {
    return;
  }

  request = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(motor_protocol.fdcan);
  start = HAL_GetTick();
  while (((motor_protocol.fdcan->Instance->TXBTO & request) == 0U) &&
         ((HAL_GetTick() - start) < MOTOR_PROTOCOL_HELLO_TIMEOUT_MS)) {
  }
}

/** @brief 把协议层运行请求转交给电机应用层。 */
static void MotorProtocol_SetRun(uint8_t run) {
  if (motor_protocol.control == NULL) return;
  if (run != 0U) {
    FOC_Control_EnableSpeedLoop(motor_protocol.control, 1U);
  }
  /* MotorApp保留当前PCB的校准、IDLE复位和PWM启动时序。 */
  MotorApp_RequestRun(run);
}

/**
 * @brief 发送短周期状态反馈。
 * @note 数据布局由上位机协议固定；未使用的温度字段保持为零。
 */
static void MotorProtocol_SendNormalFeedback(void) {
  /* data按协议固定偏移打包，数组清零同时保证保留字段为零。 */
  uint8_t data[12] = {0}; /* 反馈帧缓冲区。 */
  /* flags逐位报告校准、速度环和SVPWM限幅状态。 */
  uint8_t flags = 0U;    /* 校准、速度环、SVPWM 限幅状态位。 */

  MotorProtocol_PutS16(&data[0],
                       MotorProtocol_S16(foc.observer.state.speed_rpm, 1.0f));
  MotorProtocol_PutS16(&data[2], MotorProtocol_S16(foc.state.i_dq.q, 100.0f));
  MotorProtocol_PutU16(&data[4], MotorProtocol_U16(foc.state.vbus, 100.0f));
  /* 当前CBT6分支没有温度字段，Byte6~7保持零，协议布局不变。 */
  if (foc.calibration.calibrated != 0U) flags |= 0x01U;
  if ((motor_protocol.control != NULL) &&
      (motor_protocol.control->speed_loop_enable != 0U)) flags |= 0x02U;
  if (foc.svpwm.limited != 0U) flags |= 0x04U;

  data[8] = MotorProtocol_StateCode();
  data[9] = flags;
  data[10] = motor_protocol.feedback_sequence++;
  (void)MotorProtocol_Send(MOTOR_PROTOCOL_ID_FEEDBACK_BASE +
                               motor_protocol.node_id,
                           MOTOR_PROTOCOL_FEEDBACK_DLC, FDCAN_FD_CAN, data);
}

/** @brief 发送包含电流、dq 量、相位和状态的调试反馈帧。 */
static void MotorProtocol_SendDebugFeedback(void) {
  /* 调试帧容量按CAN FD最大长度分配，实际发送长度由协议常量决定。 */
  uint8_t data[64] = {0}; /* CAN FD 调试帧，未使用的尾部保持为零。 */

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
  MotorProtocol_PutS16(&data[20], 0);
  MotorProtocol_PutS16(&data[22],
                       MotorProtocol_S16(foc.observer.state.phase_raw *
                                             RAD_TO_DEG_F,
                                         100.0f));
  data[24] = MotorProtocol_StateCode();
  data[25] = (foc.calibration.calibrated != 0U) ? 0x01U : 0U;
  data[26] = ((motor_protocol.control != NULL) &&
              (motor_protocol.control->speed_loop_enable != 0U)) ? 0x02U : 0U;
  data[27] = (foc.svpwm.limited != 0U) ? 0x04U : 0U;
  data[28] = motor_protocol.feedback_sequence++;
  (void)MotorProtocol_Send(MOTOR_PROTOCOL_ID_DEBUG_BASE +
                               motor_protocol.node_id,
                           MOTOR_PROTOCOL_DEBUG_DLC, FDCAN_FD_CAN, data);
}

/** @brief 发送 Classic CAN 心跳帧，供网关监测节点在线状态。 */
static void MotorProtocol_SendHeartbeat(void) {
  /* 心跳载荷包括节点状态、调试状态、限幅状态和序号。 */
  uint8_t data[8] = {0}; /* 7 字节状态数据 + 1 字节 CRC。 */
  data[0] = motor_protocol.node_id;
  data[1] = MotorProtocol_StateCode();
  data[2] = (foc.calibration.calibrated != 0U) ? 1U : 0U;
  data[3] = motor_protocol.debug_enabled;
  data[4] = (uint8_t)MotorProtocol_S8(0.0f);
  data[5] = (uint8_t)foc.svpwm.limited;
  data[6] = motor_protocol.feedback_sequence;
  data[7] = MotorProtocol_CRC8(data, 7U);
  (void)MotorProtocol_Send(MOTOR_PROTOCOL_ID_HEARTBEAT_BASE +
                               motor_protocol.node_id,
                           FDCAN_DLC_BYTES_8, FDCAN_CLASSIC_CAN, data);
}

/** @brief 校验ENTER_BOOT并在Boot构建中发送确认后请求复位。 */
static void MotorProtocol_HandleBoot(const uint8_t data[8]) {
  if (((data[0] != motor_protocol.node_id) &&
       (data[0] != MOTOR_PROTOCOL_BROADCAST_MASK)) ||
      (data[1] != MOTOR_PROTOCOL_CMD_ENTER_BOOT) ||
      (MotorProtocol_CRC8(data, 7U) != data[7])) {
    return;
  }

#if APP_WITH_BOOTLOADER
  /* ack是进入Boot前发送给上位机的确认帧。 */
  uint8_t ack[8] = {0};
  /* request保存HAL发送队列中该确认帧对应的完成位。 */
  uint32_t request;
  /* start用于限制等待发送完成的阻塞时间。 */
  uint32_t start;

  MotorApp_RequestRun(0U);
  foc_motor_state = FOC_MOTOR_IDLE;
  FOC_PWM_Stop();

  ack[0] = motor_protocol.node_id;
  ack[1] = MOTOR_PROTOCOL_CMD_ENTER_BOOT;
  ack[2] = 1U;
  ack[7] = MotorProtocol_CRC8(ack, 7U);

  start = HAL_GetTick();
  while ((HAL_FDCAN_GetTxFifoFreeLevel(motor_protocol.fdcan) == 0U) &&
         ((HAL_GetTick() - start) < MOTOR_PROTOCOL_BOOT_ACK_TIMEOUT_MS)) {
  }
  if (HAL_FDCAN_GetTxFifoFreeLevel(motor_protocol.fdcan) != 0U &&
      MotorProtocol_Send(MOTOR_PROTOCOL_ID_RESPONSE_BASE +
                             motor_protocol.node_id,
                         FDCAN_DLC_BYTES_8, FDCAN_CLASSIC_CAN, ack) == HAL_OK) {
    request = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(motor_protocol.fdcan);
    start = HAL_GetTick();
    while (((motor_protocol.fdcan->Instance->TXBTO & request) == 0U) &&
           ((HAL_GetTick() - start) < MOTOR_PROTOCOL_BOOT_ACK_TIMEOUT_MS)) {
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
#else
  /* standalone构建只校验并忽略ENTER_BOOT，不写备份寄存器也不复位。 */
#endif
}

/** @brief 解析固定24字节多节点控制帧并更新本节点目标。 */
static void MotorProtocol_HandleVector(const uint8_t data[24]) {
  /* index指向本节点在速度数组和运行位图中的零基索引。 */
  uint8_t index;
  /* mask是控制帧携带的目标节点位图。 */
  uint8_t mask;

  if (data[0] != 1U) return;
  if (data[1] == MOTOR_PROTOCOL_CMD_DEBUG_SELECT) {
    /* debug_mask描述独占调试节点；valid用于拒绝多节点或空位图。 */
    uint8_t debug_mask = data[2];
    /* valid为1表示调试选择帧格式正确且只选择了一个节点。 */
    uint8_t valid = ((debug_mask != 0U) &&
                     ((debug_mask & (uint8_t)(debug_mask - 1U)) == 0U) &&
                     (data[3] != 0U)) ? 1U : 0U;
    motor_protocol.debug_suppressed = valid;
    motor_protocol.debug_enabled =
        (valid != 0U &&
         (debug_mask & (uint8_t)(1U << (motor_protocol.node_id - 1U))) != 0U)
            ? 1U
            : 0U;
    motor_protocol.debug_divider = 0U;
    return;
  }

  mask = data[2];
  if ((mask == 0U) ||
      ((mask & (uint8_t)(1U << (motor_protocol.node_id - 1U))) == 0U)) {
    return;
  }

  index = (uint8_t)(motor_protocol.node_id - 1U);
  if ((data[1] == MOTOR_PROTOCOL_CMD_SPEED_VECTOR) ||
      (data[1] == MOTOR_PROTOCOL_CMD_RUN_VECTOR)) {
    if (data[1] == MOTOR_PROTOCOL_CMD_SPEED_VECTOR) {
      /* speed_rpm是本节点在向量帧中对应的有符号转速目标。 */
      int16_t speed_rpm;
      memcpy(&speed_rpm, &data[8U + (2U * index)], sizeof(speed_rpm));
      if ((speed_rpm < -10000) || (speed_rpm > 10000)) return;
      motor_protocol.control->speed_command_rpm = (float)speed_rpm;
      motor_protocol.control->speed_ref_rpm = (float)speed_rpm;
      FOC_Control_EnableSpeedLoop(motor_protocol.control, 1U);
    }
    if ((data[3] & (uint8_t)(1U << index)) != 0U) {
      MotorProtocol_SetRun(1U);
    } else if (data[1] == MOTOR_PROTOCOL_CMD_RUN_VECTOR) {
      MotorProtocol_SetRun(0U);
    }
  } else if (data[1] == MOTOR_PROTOCOL_CMD_STATUS_ONCE) {
    MotorProtocol_SendNormalFeedback();
  }
}

/** @brief 兼容旧版8字节单节点运行和速度控制帧。 */
static void MotorProtocol_HandleLegacy(const uint8_t data[8]) {
  if (((data[0] != motor_protocol.node_id) &&
       (data[0] != MOTOR_PROTOCOL_BROADCAST_MASK)) ||
      (MotorProtocol_CRC8(data, 7U) != data[7])) {
    return;
  }
  if (data[1] == 0x01U) {
    MotorProtocol_SetRun(data[3]);
  } else if (data[1] == 0x02U) {
    /* 旧版帧直接携带IEEE754 float速度，兼容历史上位机。 */
    float speed_rpm;
    memcpy(&speed_rpm, &data[3], sizeof(speed_rpm));
    if ((speed_rpm == speed_rpm) && speed_rpm >= -10000.0f &&
        speed_rpm <= 10000.0f) {
      motor_protocol.control->speed_command_rpm = speed_rpm;
      motor_protocol.control->speed_ref_rpm = speed_rpm;
      FOC_Control_EnableSpeedLoop(motor_protocol.control, 1U);
    }
  }
}

/** @brief 按标准ID、帧格式和长度分派一项接收帧。 */
static void MotorProtocol_HandleRx(const MotorProtocol_RxItem_t *item) {
  /* header只读引用接收项中的HAL元数据，避免复制大结构体。 */
  const FDCAN_RxHeaderTypeDef *header = &item->header;
  if ((header->IdType != FDCAN_STANDARD_ID) ||
      (header->RxFrameType != FDCAN_DATA_FRAME)) {
    return;
  }
  if (header->Identifier == MOTOR_PROTOCOL_ID_BOOT &&
      header->FDFormat == FDCAN_CLASSIC_CAN &&
      header->DataLength == FDCAN_DLC_BYTES_8) {
    MotorProtocol_HandleBoot(item->data);
  } else if (header->Identifier == MOTOR_PROTOCOL_ID_CONTROL &&
             header->FDFormat == FDCAN_FD_CAN &&
             header->DataLength == MOTOR_PROTOCOL_CONTROL_DLC) {
    MotorProtocol_HandleVector(item->data);
  } else if (header->Identifier == MOTOR_PROTOCOL_ID_CONTROL &&
             header->FDFormat == FDCAN_CLASSIC_CAN &&
             header->DataLength == FDCAN_DLC_BYTES_8) {
    MotorProtocol_HandleLegacy(item->data);
  }
}

/** @brief 从当前PCB已有的FDCAN FIFO0轮询搬运接收帧。 */
static void MotorProtocol_PollRx(void) {
  while (HAL_FDCAN_GetRxFifoFillLevel(motor_protocol.fdcan,
                                      FDCAN_RX_FIFO0) > 0U) {
    /* next为写入一帧后的候选位置；等于tail时保留一格表示队列满。 */
    uint8_t next = (uint8_t)((motor_protocol.rx_head + 1U) %
                             MOTOR_PROTOCOL_RX_RING_SIZE);
    /* item指向当前写入槽，HAL直接填充其头和数据区。 */
    MotorProtocol_RxItem_t *item = &motor_protocol.rx_ring[motor_protocol.rx_head];
    if (HAL_FDCAN_GetRxMessage(motor_protocol.fdcan, FDCAN_RX_FIFO0,
                               &item->header, item->data) != HAL_OK) {
      return;
    }
    if (next != motor_protocol.rx_tail) {
      motor_protocol.rx_head = next;
    }
  }
}

/** @brief 使用HAL tick执行1ms时隙反馈调度，不引入新的底层定时器。 */
static void MotorProtocol_RunScheduler(uint32_t now) {
  /* elapsed是自上次调度以来经过的毫秒数，过大时限制补偿步数。 */
  uint32_t elapsed = now - motor_protocol.last_tick;
  if (elapsed > 20U) elapsed = 20U;
  while (elapsed-- != 0U) {
    motor_protocol.last_tick++;
    if (MOTOR_PROTOCOL_PERIODIC_FD_FEEDBACK_ENABLED != 0U) {
      /* slot是当前毫秒在节点轮询时隙中的位置。 */
      uint8_t slot = motor_protocol.timer_slot++;
      if (motor_protocol.timer_slot >= MOTOR_PROTOCOL_SLOT_COUNT) {
        motor_protocol.timer_slot = 0U;
      }
      if (motor_protocol.debug_suppressed == 0U &&
          slot == (uint8_t)(motor_protocol.node_id - 1U)) {
        MotorProtocol_SendNormalFeedback();
      } else if (motor_protocol.debug_suppressed != 0U &&
                 motor_protocol.debug_enabled != 0U) {
        if (++motor_protocol.debug_divider >= MOTOR_PROTOCOL_DEBUG_DIVIDER) {
          motor_protocol.debug_divider = 0U;
          MotorProtocol_SendDebugFeedback();
        }
      }
    }
  }
}

/** @brief 配置协议过滤器、启动FDCAN并发送上电HELLO。 */
HAL_StatusTypeDef MotorProtocol_Init(FDCAN_HandleTypeDef *hfdcan,
                                     FOC_Control_t *control) {
  /* filter描述当前PCB唯一的标准ID掩码过滤器。 */
  FDCAN_FilterTypeDef filter = {0};
  /* node_id保存从Flash读取并经过范围校验后的节点地址。 */
  uint8_t node_id;
  if ((hfdcan == NULL) || (control == NULL)) return HAL_ERROR;

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  /* 当前PCB只配置了一个标准过滤器；掩码同时覆盖0x000和0x100。
   * 其它被放行的同组ID仍会在应用层按协议ID丢弃。 */
  filter.FilterID2 = 0x6FFU;
  filter.FilterIndex = 0U;
  filter.FilterID1 = MOTOR_PROTOCOL_ID_BOOT;
  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) return HAL_ERROR;
  if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK) {
    return HAL_ERROR;
  }

  node_id = *(const volatile uint8_t *)MOTOR_PROTOCOL_CONFIG_NODE_ADDR;
  motor_protocol.node_id = (node_id >= MOTOR_PROTOCOL_NODE_MIN &&
                             node_id <= MOTOR_PROTOCOL_NODE_MAX)
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
  motor_protocol.last_tick = HAL_GetTick();
  motor_protocol.heartbeat_next_tick = motor_protocol.last_tick + 1000U;

  if (HAL_FDCAN_Start(hfdcan) != HAL_OK) return HAL_ERROR;
  MotorProtocol_SendPowerOnHello();
  return HAL_OK;
}

/** @brief 保留的定时器接口；当前PCB不使用TIM中断调度。 */
void MotorProtocol_TimerTick(TIM_HandleTypeDef *htim) {
  (void)htim;
  /* 保留接口，当前PCB不改底层定时器；调度由主循环HAL_GetTick完成。 */
}

/** @brief 在主循环轮询接收队列并运行反馈、心跳调度。 */
void MotorProtocol_Process(void) {
  /* item是从应用层接收环形队列取出的临时帧副本。 */
  MotorProtocol_RxItem_t item;
  /* now用于驱动周期反馈和1秒心跳调度。 */
  uint32_t now;
  if ((motor_protocol.fdcan == NULL) || (motor_protocol.control == NULL)) return;

  MotorProtocol_PollRx();
  while (motor_protocol.rx_tail != motor_protocol.rx_head) {
    item = motor_protocol.rx_ring[motor_protocol.rx_tail];
    motor_protocol.rx_tail = (uint8_t)((motor_protocol.rx_tail + 1U) %
                                       MOTOR_PROTOCOL_RX_RING_SIZE);
    MotorProtocol_HandleRx(&item);
  }

  now = HAL_GetTick();
  MotorProtocol_RunScheduler(now);
  if ((int32_t)(now - motor_protocol.heartbeat_next_tick) >= 0) {
    motor_protocol.heartbeat_next_tick = now + 1000U;
    MotorProtocol_SendHeartbeat();
  }
}
