#include "app_boot_control.h"
#include "app_memory.h"

#if APP_WITH_BOOTLOADER

#define APP_BOOT_CONFIG_ADDR       0x0801F800UL
#define APP_BOOT_CONFIG_MAGIC      0x31474643UL /* CFG1 */
#define APP_BOOT_CONFIG_VERSION    1U
#define APP_BOOT_CONFIG_LENGTH     84U

/* 只映射 Boot_Config_t 不会变动的头部，不把 Bootloader 模块链接进 APP。 */
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint8_t node_id;
} AppBootConfigPrefix_t;

/* 只保存现有 FDCAN 句柄，不拥有外设，也不建立第二套 CAN 队列。 */
static FDCAN_HandleTypeDef *g_boot_fdcan;
static uint8_t g_boot_node_id = APP_BOOT_NODE_ID;

static uint8_t AppBootControl_LoadNodeId(void)
{
    const volatile AppBootConfigPrefix_t *cfg =
        (const volatile AppBootConfigPrefix_t *)APP_BOOT_CONFIG_ADDR;

    /* Bootloader 在 Jump 前已完整校验 Config CRC；APP 只核对固定头部。 */
    if ((cfg->magic == APP_BOOT_CONFIG_MAGIC) &&
        (cfg->version == APP_BOOT_CONFIG_VERSION) &&
        (cfg->length == APP_BOOT_CONFIG_LENGTH) &&
        (cfg->node_id >= 1U) &&
        (cfg->node_id <= 8U))
    {
        return cfg->node_id;
    }

    return APP_BOOT_NODE_ID;
}

static uint8_t AppBootControl_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0U;
    uint8_t i;

    while (len-- != 0U)
    {
        crc ^= *data++;
        for (i = 0U; i < 8U; ++i)
        {
            crc = ((crc & 0x80U) != 0U)
                ? (uint8_t)((crc << 1U) ^ 0x07U)
                : (uint8_t)(crc << 1U);
        }
    }

    return crc;
}

static void App_RequestBootloader(void)
{
    /* 与 Bootloader 的 Boot_RequestBootloader() 使用同一个 BKP0R 语义。 */
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

#if defined(__HAL_RCC_RTCAPB_CLK_ENABLE)
    __HAL_RCC_RTCAPB_CLK_ENABLE();
#endif

    TAMP->BKP0R = APP_BOOT_REQUEST_MAGIC;
    __DSB();
    NVIC_SystemReset();

    while (1)
    {
        /* NVIC_SystemReset() 正常不会返回。 */
    }
}

HAL_StatusTypeDef AppBootControl_Init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    if (hfdcan == NULL)
    {
        return HAL_ERROR;
    }

    /* 唯一的标准过滤器精确接收 Boot 控制 ID 0x000。 */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = APP_BOOT_CONTROL_RX_ID;
    filter.FilterID2 = 0x7FFU;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 其它标准/扩展帧和所有远程帧不进入本 APP 的 Boot 接收路径。 */
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    g_boot_node_id = AppBootControl_LoadNodeId();
    g_boot_fdcan = hfdcan;
    return HAL_FDCAN_Start(hfdcan);
}

void AppBootControl_Process(void)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];

    if (g_boot_fdcan == NULL)
    {
        return;
    }

    /*
     * 主循环轮询现有 FIFO0。先用 64 字节临时区安全取帧，再严格筛选
     * 标准数据帧、经典 CAN、DLC=8、ID=0x000，避免异常 FD 帧越界。
     */
    while (HAL_FDCAN_GetRxFifoFillLevel(g_boot_fdcan, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(g_boot_fdcan,
                                   FDCAN_RX_FIFO0,
                                   &header,
                                   data) != HAL_OK)
        {
            return;
        }

        if ((header.Identifier != APP_BOOT_CONTROL_RX_ID) ||
            (header.IdType != FDCAN_STANDARD_ID) ||
            (header.RxFrameType != FDCAN_DATA_FRAME) ||
            (header.FDFormat != FDCAN_CLASSIC_CAN) ||
            (header.DataLength != FDCAN_DLC_BYTES_8))
        {
            continue;
        }

        if (AppBootControl_CRC8(data, 7U) != data[7])
        {
            continue;
        }

        if ((data[0] != g_boot_node_id) && (data[0] != 0xFFU))
        {
            continue;
        }

        if (data[1] == APP_BOOT_ENTER_CMD)
        {
            /* Trial 成功不依赖 ACK；真正复位回 Bootloader 就是成功响应。 */
            App_RequestBootloader();
        }
    }
}

#else

/* standalone 构建不带 Bootloader 行为；LTO 会移除这两个空入口。 */
HAL_StatusTypeDef AppBootControl_Init(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    return HAL_OK;
}

void AppBootControl_Process(void)
{
}

#endif /* APP_WITH_BOOTLOADER */
