/**
 * @file app_boot_control.c
 * @brief 应用程序侧 Bootloader 请求处理实现。
 *
 * 本文件属于用户维护的应用层代码，负责从 FDCAN 接收进入 Bootloader
 * 的控制帧，并通过备份寄存器记录请求后复位系统。Bootloader 本身不在
 * 此处实现，应用只依赖公开的配置布局和请求魔数，避免把 Bootloader
 * 模块链接进应用镜像。所有代码均位于独立的用户源文件中，不会被
 * STM32CubeMX 重新生成时覆盖。
 */
#include "app_boot_control.h"
#include "app_memory.h"

#if APP_WITH_BOOTLOADER

/* Flash 中 Boot 配置结构的固定地址，由链接布局和 Bootloader 共同约定。 */
#define APP_BOOT_CONFIG_ADDR       0x0801F800UL
/* 配置头的识别魔数，ASCII 形式为 CFG1，用于排除无效 Flash 内容。 */
#define APP_BOOT_CONFIG_MAGIC      0x31474643UL /* CFG1 */
/* 当前应用支持的配置结构版本号。 */
#define APP_BOOT_CONFIG_VERSION    1U
/* Boot_Config_t 的固定总长度，校验头部时用于识别布局是否匹配。 */
#define APP_BOOT_CONFIG_LENGTH     84U

/* 只映射 Boot_Config_t 不会变动的头部，不把 Bootloader 模块链接进 APP。 */
typedef struct
{
    /* 配置有效性魔数。 */
    uint32_t magic;
    /* 配置结构版本。 */
    uint16_t version;
    /* 配置结构总字节数。 */
    uint16_t length;
    /* 当前电机节点编号，合法范围为1到8。 */
    uint8_t node_id;
} AppBootConfigPrefix_t;

/* 只保存现有 FDCAN 句柄，不拥有外设，也不建立第二套 CAN 队列。 */
/* 应用共用的 FDCAN 外设句柄。 */
static FDCAN_HandleTypeDef *g_boot_fdcan;
/* 当前节点的 Boot 控制地址。 */
static uint8_t g_boot_node_id = APP_BOOT_NODE_ID;

/**
 * @brief 从固定 Flash 配置区读取并校验本机节点号。
 * @return 合法的节点号；配置无效时返回编译期默认节点号。
 */
static uint8_t AppBootControl_LoadNodeId(void)
{
    /* 仅按固定头部解释 Flash，避免依赖 Bootloader 的完整私有结构。 */
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

/**
 * @brief 计算 Boot 控制帧使用的 CRC-8。
 * @param data 待计算的数据起始地址。
 * @param len 参与计算的字节数，不包含帧尾 CRC 字节。
 * @return 按多项式0x07计算得到的 CRC-8 值。
 */
static uint8_t AppBootControl_CRC8(const uint8_t *data, uint8_t len)
{
    /* CRC 累加寄存器，初值为0。 */
    uint8_t crc = 0U;
    /* 当前数据字节内的位计数器。 */
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

/**
 * @brief 设置 Bootloader 请求标志并触发系统复位。
 *
 * 该函数正常不会返回；系统复位后由 Bootloader 检查备份寄存器中的
 * 请求魔数，并决定是否进入升级或维护流程。
 */
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

/**
 * @brief 初始化应用侧 Boot 控制接收路径。
 * @param hfdcan 当前板卡已经初始化的 FDCAN 句柄。
 * @return HAL_OK 表示过滤器配置和启动成功，否则返回 HAL_ERROR。
 */
HAL_StatusTypeDef AppBootControl_Init(FDCAN_HandleTypeDef *hfdcan)
{
    /* Boot 控制帧使用的标准帧过滤器。 */
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

/**
 * @brief 轮询并处理 Boot 控制帧。
 *
 * 函数只消费已经进入 FIFO0 的帧，并逐项检查 ID、帧类型、格式、长度、
 * CRC 和节点号。它应由应用主循环调用，不会改变现有电流环中断路径。
 */
void AppBootControl_Process(void)
{
    /* 当前待处理接收帧的 FDCAN 元数据。 */
    FDCAN_RxHeaderTypeDef header;
    /* 最大 FD 数据区，避免异常长度导致越界。 */
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
/**
 * @brief 无 Bootloader 构建下的兼容初始化入口。
 * @param hfdcan 未使用的 FDCAN 句柄，仅为保持统一接口。
 * @return 始终返回 HAL_OK。
 */
HAL_StatusTypeDef AppBootControl_Init(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    return HAL_OK;
}

/**
 * @brief 无 Bootloader 构建下的兼容处理入口。
 * 该配置不接收 Boot 控制帧，因此函数保持空实现。
 */
void AppBootControl_Process(void)
{
}

#endif /* APP_WITH_BOOTLOADER */
