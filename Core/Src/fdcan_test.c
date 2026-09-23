#include "fdcan_test.h"

#include "fdcan.h"

/**
 * @brief  单次发送一帧 CAN FD 测试帧。
 * @note
 *  - 标准 ID = 0x555
 *  - 数据长度 = 8 Byte
 *  - CAN FD
 *  - BRS 关闭，便于先用同一比特率验证报文路径
 *  - Nominal/Data 波特率由 MX_FDCAN1_Init() 决定
 *  - 本函数每调用一次只向 TX FIFO 加入一帧
 */
HAL_StatusTypeDef CANFD_SendSingleTestFrame(void)
{
    /* HAL 发送头：描述标准 ID、FD 格式、DLC 和速率切换策略。 */
    FDCAN_TxHeaderTypeDef tx_header = {0};

    /* 固定模式字节，便于示波器、分析仪或另一节点逐字节比对。 */
    uint8_t tx_data[8] =
    {
        0x11,
        0x22,
        0x33,
        0x44,
        0x55,
        0x66,
        0x77,
        0x88
    };

    tx_header.Identifier = 0x555U;

    /* 标准 11-bit ID */
    tx_header.IdType = FDCAN_STANDARD_ID;

    /* 数据帧 */
    tx_header.TxFrameType = FDCAN_DATA_FRAME;

    /* 8 字节 */
    tx_header.DataLength = FDCAN_DLC_BYTES_8;

    /* ESI：主动错误状态 */
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;

    /*
     * 关键：
     * 测试阶段关闭 BRS，数据段同样为 1 Mbps。
     *
     * 如果当前配置是 Nominal = 1 Mbps，
     * Data = 1 Mbps，
     * 那么：
     *
     * 仲裁阶段 = 1 Mbps
     * 数据阶段 = 1 Mbps
     */
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;

    /* CAN FD 帧，而不是 Classic CAN */
    tx_header.FDFormat = FDCAN_FD_CAN;

    /* 本次测试不需要 Tx Event FIFO */
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    /* 不使用 Tx Event FIFO，因此消息标记保持默认值。 */
    tx_header.MessageMarker = 0U;

    return HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1,
        &tx_header,
        tx_data);
}
