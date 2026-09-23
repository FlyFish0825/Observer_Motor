#include "debug_console.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* DMA 一次事件搬运的临时接收区；数据随后复制到环形缓冲区。 */
#define DC_DMA_RX_SIZE       64U

/*
 * 中断与主循环之间的环形缓冲区。
 * 必须是2的整数次幂。
 */
#define DC_RING_SIZE         256U

/* 单条文本命令最大长度，包含结尾 '\0'。 */
#define DC_LINE_SIZE         128U

/* 一条命令最多允许的参数数量，超出部分会被拒绝或截断处理。 */
#define DC_MAX_ARGS          10

/* 静态注册表容量，避免运行时动态分配。 */
#define DC_MAX_VARIABLES     32U

/* 自定义命令注册表容量；help/list/get/set 为内置命令。 */
#define DC_MAX_COMMANDS      16U

/* DebugConsole_Printf 的格式化临时缓冲区大小。 */
#define DC_TX_BUFFER_SIZE    192U

#if ((DC_RING_SIZE & (DC_RING_SIZE - 1U)) != 0U)
#error "DC_RING_SIZE must be a power of two"
#endif

/* 注册变量的存储类型；类型决定解析、范围检查和回读格式。 */
typedef enum
{
    DC_VAR_F32 = 0, /* 单精度浮点变量。 */
    DC_VAR_I32,     /* 有符号 32 位整数变量。 */
    DC_VAR_U32,     /* 无符号 32 位整数变量。 */
    DC_VAR_BOOL     /* 以 uint32_t 存储的布尔变量。 */
} DC_VariableType_t;

/* 按注册变量类型保存上下限；BOOL 使用 u32 成员。 */
typedef union
{
    float f32;       /* F32 的最小值或最大值。 */
    int32_t i32;     /* I32 的最小值或最大值。 */
    uint32_t u32;    /* U32/BOOL 的最小值或最大值。 */
} DC_Value_t;

/* 一个可被 get/set 访问的变量描述；address 指向调用者拥有的实时存储。 */
typedef struct
{
    const char *name;           /* 持久有效的变量名，不复制字符串。 */
    volatile void *address;     /* 实际变量地址，允许中断或控制环更新。 */

    DC_VariableType_t type;     /* 解析和打印时使用的变量类型。 */
    bool read_only;             /* true 时允许 get/list，但禁止 set。 */

    DC_Value_t minimum;         /* 写入下限；只读变量仍用于显示元数据。 */
    DC_Value_t maximum;         /* 写入上限。 */
} DC_Variable_t;

/* 一个自定义命令描述；help 可为 NULL，表示不提供帮助文本。 */
typedef struct
{
    const char *name;                  /* 持久有效的命令名。 */
    DebugConsole_CommandFn_t handler;  /* 收到命令后在主循环中调用。 */
    const char *help;                  /* help 命令显示的简短说明。 */
} DC_Command_t;

/* ======================== 串口与DMA ======================== */

static UART_HandleTypeDef *dc_uart = NULL;       /* 当前绑定的 UART。 */
static DebugConsole_TxFn_t dc_tx_function = NULL;/* 可选文本发送回调。 */

static uint8_t dc_dma_rx_buffer[DC_DMA_RX_SIZE]; /* HAL DMA 当前接收区。 */

/* ======================== 环形缓冲区 ======================== */

static uint8_t dc_ring_buffer[DC_RING_SIZE];     /* ISR 到主循环的字节队列。 */

static volatile uint16_t dc_ring_head = 0U;      /* ISR 写入位置。 */
static volatile uint16_t dc_ring_tail = 0U;      /* 主循环读取位置。 */

static volatile uint32_t dc_overflow_count = 0U; /* 队列满时丢弃字节的计数。 */
static volatile bool dc_rx_restart_pending = false; /* 错误回调请求重启 DMA。 */

/* ======================== 行缓冲区 ======================== */

static char dc_line_buffer[DC_LINE_SIZE];        /* 从字节流拼出的当前命令行。 */
static uint16_t dc_line_length = 0U;             /* 未含 '\0' 的当前长度。 */
static bool dc_line_overflow = false;            /* 当前行过长，等待换行后丢弃。 */

/* ======================== 变量和命令表 ======================== */

static DC_Variable_t dc_variables[DC_MAX_VARIABLES]; /* 变量注册表。 */
static uint16_t dc_variable_count = 0U;              /* 已用变量项数。 */

static DC_Command_t dc_commands[DC_MAX_COMMANDS];   /* 自定义命令注册表。 */
static uint16_t dc_command_count = 0U;              /* 已用命令项数。 */

/* ======================== 内部函数声明 ======================== */

static HAL_StatusTypeDef DC_StartReceive(void);

static void DC_RingPushFromISR(uint8_t data);
static bool DC_RingPop(uint8_t *data);

static bool DC_StringEqualIgnoreCase(
    const char *left,
    const char *right);

static int DC_Tokenize(
    char *line,
    char *argv[],
    int maximum_arguments);

static void DC_ParseLine(char *line);

static DC_Variable_t *DC_FindVariable(const char *name);
static DC_Command_t *DC_FindCommand(const char *name);

static bool DC_ParseF32(const char *text, float *result);
static bool DC_ParseI32(const char *text, int32_t *result);
static bool DC_ParseU32(const char *text, uint32_t *result);
static bool DC_ParseBool(const char *text, uint32_t *result);

static void DC_PrintVariable(const DC_Variable_t *variable);
static void DC_CommandHelp(void);
static void DC_CommandList(void);
static void DC_CommandGet(int argc, char *argv[]);
static void DC_CommandSet(int argc, char *argv[]);

/* ======================== 初始化 ======================== */

/* 清空所有运行时状态并启动第一次 DMA 空闲接收。 */
HAL_StatusTypeDef DebugConsole_Init(
    UART_HandleTypeDef *huart,
    DebugConsole_TxFn_t tx_fn)
{
    if ((huart == NULL) || (huart->hdmarx == NULL))
    {
        return HAL_ERROR;
    }

    dc_uart = huart;
    dc_tx_function = tx_fn;

    dc_ring_head = 0U;
    dc_ring_tail = 0U;
    dc_overflow_count = 0U;

    dc_line_length = 0U;
    dc_line_overflow = false;
    dc_rx_restart_pending = false;

    dc_variable_count = 0U;
    dc_command_count = 0U;

    memset(dc_dma_rx_buffer, 0, sizeof(dc_dma_rx_buffer));
    memset(dc_ring_buffer, 0, sizeof(dc_ring_buffer));
    memset(dc_line_buffer, 0, sizeof(dc_line_buffer));

    return DC_StartReceive();
}

/**
 * @brief 启动一次串口DMA空闲接收，供后续持续接收命令行数据。
 */
static HAL_StatusTypeDef DC_StartReceive(void)
{
    HAL_StatusTypeDef status;

    if ((dc_uart == NULL) || (dc_uart->hdmarx == NULL))
    {
        return HAL_ERROR;
    }

    status = HAL_UARTEx_ReceiveToIdle_DMA(
        dc_uart,
        dc_dma_rx_buffer,
        sizeof(dc_dma_rx_buffer));

    if (status == HAL_OK)
    {
        /*
         * 命令接收不需要DMA半传输回调。
         *
         * 保留：
         * 1. UART IDLE事件
         * 2. DMA传输完成事件
         */
        __HAL_DMA_DISABLE_IT(
            dc_uart->hdmarx,
            DMA_IT_HT);
    }

    return status;
}

/* ======================== HAL回调入口 ======================== */

void DebugConsole_OnRxEvent(
    UART_HandleTypeDef *huart,
    uint16_t size)
{
    /* 回调只负责把 DMA 本批次字节放入环形队列，命令解析留给主循环。 */
    uint16_t index;

    if ((dc_uart == NULL) || (huart != dc_uart))
    {
        return;
    }

    if (size > DC_DMA_RX_SIZE)
    {
        size = DC_DMA_RX_SIZE;
    }

    /*
     * 中断中只把字节放入环形缓冲区。
     * 不在这里调用strtof、printf或执行控制命令。
     */
    for (index = 0U; index < size; index++)
    {
        DC_RingPushFromISR(dc_dma_rx_buffer[index]);
    }

    /*
     * Normal DMA模式下，每次事件后重新启动接收。
     */
    if (DC_StartReceive() != HAL_OK)
    {
        dc_rx_restart_pending = true;
    }
}

/**
 * @brief 处理串口接收错误并重新启动DMA接收，避免一次错误导致控制台永久停止。
 */
void DebugConsole_OnError(
    UART_HandleTypeDef *huart)
{
    /* UART 错误不在中断中做恢复操作，交给主循环重新启动接收。 */
    if ((dc_uart == NULL) || (huart != dc_uart))
    {
        return;
    }

    /*
     * 不在错误中断里执行阻塞式恢复。
     * 交给主循环处理。
     */
    dc_rx_restart_pending = true;
}

/* ======================== 环形缓冲区 ======================== */

static void DC_RingPushFromISR(uint8_t data)
{
    /* 环形队列满时保留已有数据，并记录溢出而不是覆盖未处理命令。 */
    uint16_t head;
    uint16_t next;

    head = dc_ring_head;

    next = (uint16_t)(
        (head + 1U) &
        (DC_RING_SIZE - 1U));

    if (next == dc_ring_tail)
    {
        /*
         * 环形缓冲区已满。
         * 当前字节丢弃。
         */
        dc_overflow_count++;
        return;
    }

    dc_ring_buffer[head] = data;

    __DMB();

    dc_ring_head = next;
}

/**
 * @brief 从调试串口环形缓冲区取出一个字节，返回当前是否成功取到数据。
 */
static bool DC_RingPop(uint8_t *data)
{
    /* 主循环以单字节粒度消费队列，返回 false 表示当前没有新数据。 */
    uint16_t tail;

    if (data == NULL)
    {
        return false;
    }

    tail = dc_ring_tail;

    if (tail == dc_ring_head)
    {
        return false;
    }

    *data = dc_ring_buffer[tail];

    dc_ring_tail = (uint16_t)(
        (tail + 1U) &
        (DC_RING_SIZE - 1U));

    return true;
}

/* ======================== 主循环处理 ======================== */

void DebugConsole_Process(void)
{
    /* 处理顺序：恢复 DMA -> 取字节 -> 组行 -> 遇换行后分词并执行。 */
    uint8_t data;

    /*
     * UART出错或DMA重新启动失败时，在主循环恢复。
     */
    if (dc_rx_restart_pending)
    {
        dc_rx_restart_pending = false;

        (void)HAL_UART_AbortReceive(dc_uart);

        if (DC_StartReceive() != HAL_OK)
        {
            dc_rx_restart_pending = true;
        }
    }

    while (DC_RingPop(&data))
    {
        /*
         * CRLF中的'\r'直接忽略。
         */
        if (data == '\r')
        {
            continue;
        }

        /*
         * 一行命令结束。
         */
        if (data == '\n')
        {
            if (dc_line_overflow)
            {
                DebugConsole_Printf(
                    "ERR line too long\r\n");

                dc_line_overflow = false;
                dc_line_length = 0U;
                continue;
            }

            if (dc_line_length > 0U)
            {
                dc_line_buffer[dc_line_length] = '\0';

                DC_ParseLine(dc_line_buffer);
            }

            dc_line_length = 0U;
            continue;
        }

        /*
         * 支持终端退格键：
         * 0x08 = Backspace
         * 0x7F = Delete
         */
        if ((data == 0x08U) || (data == 0x7FU))
        {
            if (dc_line_length > 0U)
            {
                dc_line_length--;
            }

            continue;
        }

        if (dc_line_overflow)
        {
            continue;
        }

        /*
         * 仅保存普通可打印字符和Tab。
         */
        if ((isprint((int)data) == 0) && (data != '\t'))
        {
            continue;
        }

        if (dc_line_length <
            (DC_LINE_SIZE - 1U))
        {
            dc_line_buffer[dc_line_length] = (char)data;
            dc_line_length++;
        }
        else
        {
            dc_line_overflow = true;
        }
    }
}

/* ======================== 命令解析 ======================== */

static void DC_ParseLine(char *line)
{
    /* 一行先分词，再分派内置命令或用户注册命令。 */
    char *argv[DC_MAX_ARGS];
    int argc;

    DC_Command_t *command;

    argc = DC_Tokenize(
        line,
        argv,
        DC_MAX_ARGS);

    if (argc <= 0)
    {
        return;
    }

    if (DC_StringEqualIgnoreCase(argv[0], "help") ||
        DC_StringEqualIgnoreCase(argv[0], "?"))
    {
        DC_CommandHelp();
        return;
    }

    if (DC_StringEqualIgnoreCase(argv[0], "list"))
    {
        DC_CommandList();
        return;
    }

    if (DC_StringEqualIgnoreCase(argv[0], "get"))
    {
        DC_CommandGet(argc, argv);
        return;
    }

    if (DC_StringEqualIgnoreCase(argv[0], "set"))
    {
        DC_CommandSet(argc, argv);
        return;
    }

    command = DC_FindCommand(argv[0]);

    if ((command != NULL) &&
        (command->handler != NULL))
    {
        command->handler(argc, argv);
        return;
    }

    DebugConsole_Printf(
        "ERR unknown command: %s\r\n",
        argv[0]);
}

/**
 * @brief 按空白和注释规则切分命令行，生成参数数组并返回参数个数。
 */
static int DC_Tokenize(
    char *line,
    char *argv[],
    int maximum_arguments)
{
    /* 原地把空白字符改为 '\0'，argv 指向 line 内部，不产生堆分配。 */
    char *position;
    int argc = 0;

    if ((line == NULL) ||
        (argv == NULL) ||
        (maximum_arguments <= 0))
    {
        return 0;
    }

    position = line;

    while (*position != '\0')
    {
        /*
         * 跳过空格。
         */
        while ((*position != '\0') &&
               isspace((unsigned char)*position))
        {
            position++;
        }

        if (*position == '\0')
        {
            break;
        }

        /*
         * 支持：
         * # this is comment
         */
        if (*position == '#')
        {
            break;
        }

        if (argc >= maximum_arguments)
        {
            break;
        }

        argv[argc] = position;
        argc++;

        while ((*position != '\0') &&
               !isspace((unsigned char)*position))
        {
            position++;
        }

        if (*position != '\0')
        {
            *position = '\0';
            position++;
        }
    }

    return argc;
}

/* ======================== 内置命令 ======================== */

static void DC_CommandHelp(void)
{
    /* 输出内置命令和已注册自定义命令的使用提示。 */
    uint16_t index;

    DebugConsole_Printf(
        "Commands:\r\n"
        "  help\r\n"
        "  list\r\n"
        "  get <name>\r\n"
        "  set <name> <value>\r\n");

    for (index = 0U;
         index < dc_command_count;
         index++)
    {
        DebugConsole_Printf(
            "  %s%s%s\r\n",
            dc_commands[index].name,
            dc_commands[index].help != NULL ? " - " : "",
            dc_commands[index].help != NULL ?
                dc_commands[index].help : "");
    }
}

/**
 * @brief 输出当前所有可读写变量及其属性。
 */
static void DC_CommandList(void)
{
    /* 输出变量名、当前值和只读状态，便于现场查看控制量。 */
    uint16_t index;

    DebugConsole_Printf(
        "Variables: %u\r\n",
        (unsigned int)dc_variable_count);

    for (index = 0U;
         index < dc_variable_count;
         index++)
    {
        DC_PrintVariable(&dc_variables[index]);
    }
}

/**
 * @brief 解析get命令并读取指定变量的当前值。
 */
static void DC_CommandGet(
    int argc,
    char *argv[])
{
    /* get 只读回变量，不修改其底层地址指向的数据。 */
    DC_Variable_t *variable;

    if (argc != 2)
    {
        DebugConsole_Printf(
            "ERR usage: get <name>\r\n");
        return;
    }

    variable = DC_FindVariable(argv[1]);

    if (variable == NULL)
    {
        DebugConsole_Printf(
            "ERR unknown variable: %s\r\n",
            argv[1]);
        return;
    }

    DC_PrintVariable(variable);
}

/**
 * @brief 解析set命令，校验范围后写入指定变量或布尔开关。
 */
static void DC_CommandSet(
    int argc,
    char *argv[])
{
    /* set 依次执行查找、只读检查、类型解析和范围检查后再写入。 */
    DC_Variable_t *variable;

    float value_f32;
    int32_t value_i32;
    uint32_t value_u32;

    if (argc != 3)
    {
        DebugConsole_Printf(
            "ERR usage: set <name> <value>\r\n");
        return;
    }

    variable = DC_FindVariable(argv[1]);

    if (variable == NULL)
    {
        DebugConsole_Printf(
            "ERR unknown variable: %s\r\n",
            argv[1]);
        return;
    }

    if (variable->read_only)
    {
        DebugConsole_Printf(
            "ERR variable is read-only\r\n");
        return;
    }

    switch (variable->type)
    {
        case DC_VAR_F32:

            if (!DC_ParseF32(argv[2], &value_f32))
            {
                DebugConsole_Printf(
                    "ERR invalid float\r\n");
                return;
            }

            if ((value_f32 < variable->minimum.f32) ||
                (value_f32 > variable->maximum.f32))
            {
                DebugConsole_Printf(
                    "ERR range: %.7g to %.7g\r\n",
                    (double)variable->minimum.f32,
                    (double)variable->maximum.f32);
                return;
            }

            *(volatile float *)variable->address =
                value_f32;

            break;

        case DC_VAR_I32:

            if (!DC_ParseI32(argv[2], &value_i32))
            {
                DebugConsole_Printf(
                    "ERR invalid int32\r\n");
                return;
            }

            if ((value_i32 < variable->minimum.i32) ||
                (value_i32 > variable->maximum.i32))
            {
                DebugConsole_Printf(
                    "ERR value out of range\r\n");
                return;
            }

            *(volatile int32_t *)variable->address =
                value_i32;

            break;

        case DC_VAR_U32:

            if (!DC_ParseU32(argv[2], &value_u32))
            {
                DebugConsole_Printf(
                    "ERR invalid uint32\r\n");
                return;
            }

            if ((value_u32 < variable->minimum.u32) ||
                (value_u32 > variable->maximum.u32))
            {
                DebugConsole_Printf(
                    "ERR value out of range\r\n");
                return;
            }

            *(volatile uint32_t *)variable->address =
                value_u32;

            break;

        case DC_VAR_BOOL:

            if (!DC_ParseBool(argv[2], &value_u32))
            {
                DebugConsole_Printf(
                    "ERR bool: use 0/1/on/off/true/false\r\n");
                return;
            }

            *(volatile uint32_t *)variable->address =
                value_u32;

            break;

        default:

            DebugConsole_Printf(
                "ERR unsupported variable type\r\n");
            return;
    }

    DebugConsole_Printf("OK ");

    DC_PrintVariable(variable);
}

/* ======================== 变量输出 ======================== */

static void DC_PrintVariable(
    const DC_Variable_t *variable)
{
    if (variable == NULL)
    {
        return;
    }

    switch (variable->type)
    {
        case DC_VAR_F32:

            DebugConsole_Printf(
                "%s=%.7g\r\n",
                variable->name,
                (double)(*(volatile float *)
                    variable->address));

            break;

        case DC_VAR_I32:

            DebugConsole_Printf(
                "%s=%ld\r\n",
                variable->name,
                (long)(*(volatile int32_t *)
                    variable->address));

            break;

        case DC_VAR_U32:

            DebugConsole_Printf(
                "%s=%lu\r\n",
                variable->name,
                (unsigned long)(*(volatile uint32_t *)
                    variable->address));

            break;

        case DC_VAR_BOOL:

            DebugConsole_Printf(
                "%s=%s\r\n",
                variable->name,
                (*(volatile uint32_t *)
                    variable->address) != 0U ?
                    "on" : "off");

            break;

        default:
            break;
    }
}

/* ======================== 数值解析 ======================== */

static bool DC_ParseF32(
    const char *text,
    float *result)
{
    /* 使用 strtof，并拒绝空串、尾随字符、溢出和非有限值。 */
    char *end;
    float value;

    if ((text == NULL) || (result == NULL))
    {
        return false;
    }

    errno = 0;
    end = NULL;

    value = strtof(text, &end);

    if ((end == text) ||
        (end == NULL) ||
        (*end != '\0') ||
        (errno == ERANGE) ||
        !isfinite(value))
    {
        return false;
    }

    *result = value;

    return true;
}

/**
 * @brief 把文本解析为有符号32位整数并检查转换错误。
 */
static bool DC_ParseI32(
    const char *text,
    int32_t *result)
{
    /* 使用 base=0，兼容十进制、负数和 0x 前缀整数。 */
    char *end;
    long value;

    if ((text == NULL) || (result == NULL))
    {
        return false;
    }

    errno = 0;
    end = NULL;

    /*
     * base=0支持：
     * 123
     * -123
     * 0x1234
     */
    value = strtol(text, &end, 0);

    if ((end == text) ||
        (end == NULL) ||
        (*end != '\0') ||
        (errno == ERANGE) ||
        (value < INT32_MIN) ||
        (value > INT32_MAX))
    {
        return false;
    }

    *result = (int32_t)value;

    return true;
}

/**
 * @brief 把文本解析为无符号32位整数并检查转换错误。
 */
static bool DC_ParseU32(
    const char *text,
    uint32_t *result)
{
    /* 无符号输入显式拒绝负号，再检查转换溢出。 */
    char *end;
    unsigned long value;

    if ((text == NULL) || (result == NULL))
    {
        return false;
    }

    if (text[0] == '-')
    {
        return false;
    }

    errno = 0;
    end = NULL;

    value = strtoul(text, &end, 0);

    if ((end == text) ||
        (end == NULL) ||
        (*end != '\0') ||
        (errno == ERANGE) ||
        (value > UINT32_MAX))
    {
        return false;
    }

    *result = (uint32_t)value;

    return true;
}

/**
 * @brief 把on/off、true/false或0/1等文本转换为布尔值。
 */
static bool DC_ParseBool(
    const char *text,
    uint32_t *result)
{
    /* 接受 0/1、on/off、true/false、yes/no，统一输出 0 或 1。 */
    if ((text == NULL) || (result == NULL))
    {
        return false;
    }

    if (DC_StringEqualIgnoreCase(text, "1") ||
        DC_StringEqualIgnoreCase(text, "on") ||
        DC_StringEqualIgnoreCase(text, "true") ||
        DC_StringEqualIgnoreCase(text, "yes"))
    {
        *result = 1U;
        return true;
    }

    if (DC_StringEqualIgnoreCase(text, "0") ||
        DC_StringEqualIgnoreCase(text, "off") ||
        DC_StringEqualIgnoreCase(text, "false") ||
        DC_StringEqualIgnoreCase(text, "no"))
    {
        *result = 0U;
        return true;
    }

    return false;
}

/* ======================== 注册变量 ======================== */

static bool DC_CanRegisterVariable(
    const char *name,
    const volatile void *address)
{
    /* 统一检查名称、地址、容量和重名，供四种变量注册函数复用。 */
    if ((name == NULL) ||
        (name[0] == '\0') ||
        (address == NULL))
    {
        return false;
    }

    if (dc_variable_count >= DC_MAX_VARIABLES)
    {
        return false;
    }

    if (DC_FindVariable(name) != NULL)
    {
        return false;
    }

    return true;
}

/**
 * @brief 注册一个浮点变量，并保存其上下限和只读属性。
 */
bool DebugConsole_RegisterF32(
    const char *name,
    volatile float *value,
    float minimum,
    float maximum,
    bool read_only)
{
    /* 保存浮点变量地址和上下限；控制台不拥有该变量的存储。 */
    DC_Variable_t *variable;

    if (!DC_CanRegisterVariable(name, value))
    {
        return false;
    }

    if ((!isfinite(minimum)) ||
        (!isfinite(maximum)) ||
        (minimum > maximum))
    {
        return false;
    }

    variable = &dc_variables[dc_variable_count];

    variable->name = name;
    variable->address = value;
    variable->type = DC_VAR_F32;
    variable->read_only = read_only;

    variable->minimum.f32 = minimum;
    variable->maximum.f32 = maximum;

    dc_variable_count++;

    return true;
}

/**
 * @brief 注册一个有符号32位变量，并保存其上下限和只读属性。
 */
bool DebugConsole_RegisterI32(
    const char *name,
    volatile int32_t *value,
    int32_t minimum,
    int32_t maximum,
    bool read_only)
{
    /* 保存有符号整数变量描述，写入前由 DC_CommandSet 做范围校验。 */
    DC_Variable_t *variable;

    if (!DC_CanRegisterVariable(name, value))
    {
        return false;
    }

    if (minimum > maximum)
    {
        return false;
    }

    variable = &dc_variables[dc_variable_count];

    variable->name = name;
    variable->address = value;
    variable->type = DC_VAR_I32;
    variable->read_only = read_only;

    variable->minimum.i32 = minimum;
    variable->maximum.i32 = maximum;

    dc_variable_count++;

    return true;
}

/**
 * @brief 注册一个无符号32位变量，并保存其上下限和只读属性。
 */
bool DebugConsole_RegisterU32(
    const char *name,
    volatile uint32_t *value,
    uint32_t minimum,
    uint32_t maximum,
    bool read_only)
{
    /* 保存无符号整数变量描述，拒绝 minimum 大于 maximum 的配置。 */
    DC_Variable_t *variable;

    if (!DC_CanRegisterVariable(name, value))
    {
        return false;
    }

    if (minimum > maximum)
    {
        return false;
    }

    variable = &dc_variables[dc_variable_count];

    variable->name = name;
    variable->address = value;
    variable->type = DC_VAR_U32;
    variable->read_only = read_only;

    variable->minimum.u32 = minimum;
    variable->maximum.u32 = maximum;

    dc_variable_count++;

    return true;
}

/**
 * @brief 注册一个布尔变量，供命令行读取或修改。
 */
bool DebugConsole_RegisterBool(
    const char *name,
    volatile uint32_t *value,
    bool read_only)
{
    /* BOOL 约定使用 uint32_t 地址，读取时非零显示为 on。 */
    DC_Variable_t *variable;

    if (!DC_CanRegisterVariable(name, value))
    {
        return false;
    }

    variable = &dc_variables[dc_variable_count];

    variable->name = name;
    variable->address = value;
    variable->type = DC_VAR_BOOL;
    variable->read_only = read_only;

    variable->minimum.u32 = 0U;
    variable->maximum.u32 = 1U;

    dc_variable_count++;

    return true;
}

/* ======================== 注册自定义命令 ======================== */

bool DebugConsole_RegisterCommand(
    const char *name,
    DebugConsole_CommandFn_t handler,
    const char *help)
{
    /* 注册表只保存指针；name/help 必须在控制台生命周期内保持有效。 */
    uint16_t index;

    if ((name == NULL) ||
        (name[0] == '\0') ||
        (handler == NULL))
    {
        return false;
    }

    if (dc_command_count >= DC_MAX_COMMANDS)
    {
        return false;
    }

    /*
     * 禁止覆盖内置命令。
     */
    if (DC_StringEqualIgnoreCase(name, "help") ||
        DC_StringEqualIgnoreCase(name, "list") ||
        DC_StringEqualIgnoreCase(name, "get") ||
        DC_StringEqualIgnoreCase(name, "set"))
    {
        return false;
    }

    for (index = 0U;
         index < dc_command_count;
         index++)
    {
        if (DC_StringEqualIgnoreCase(
                dc_commands[index].name,
                name))
        {
            return false;
        }
    }

    dc_commands[dc_command_count].name = name;
    dc_commands[dc_command_count].handler = handler;
    dc_commands[dc_command_count].help = help;

    dc_command_count++;

    return true;
}

/* ======================== 查找 ======================== */

static DC_Variable_t *DC_FindVariable(
    const char *name)
{
    /* 变量名比较不区分大小写，返回注册表项而非复制内容。 */
    uint16_t index;

    if (name == NULL)
    {
        return NULL;
    }

    for (index = 0U;
         index < dc_variable_count;
         index++)
    {
        if (DC_StringEqualIgnoreCase(
                dc_variables[index].name,
                name))
        {
            return &dc_variables[index];
        }
    }

    return NULL;
}

/**
 * @brief 按名称查找已注册命令，找不到时返回空指针。
 */
static DC_Command_t *DC_FindCommand(
    const char *name)
{
    /* 按不区分大小写的名称查找自定义命令。 */
    uint16_t index;

    if (name == NULL)
    {
        return NULL;
    }

    for (index = 0U;
         index < dc_command_count;
         index++)
    {
        if (DC_StringEqualIgnoreCase(
                dc_commands[index].name,
                name))
        {
            return &dc_commands[index];
        }
    }

    return NULL;
}

/* ======================== 字符串比较 ======================== */

static bool DC_StringEqualIgnoreCase(
    const char *left,
    const char *right)
{
    /* 仅比较 ASCII 风格命令名，直到双方同时到达字符串结尾。 */
    if ((left == NULL) || (right == NULL))
    {
        return false;
    }

    while ((*left != '\0') &&
           (*right != '\0'))
    {
        if (tolower((unsigned char)*left) !=
            tolower((unsigned char)*right))
        {
            return false;
        }

        left++;
        right++;
    }

    return (*left == '\0') &&
           (*right == '\0');
}

/* ======================== 文本发送 ======================== */

void DebugConsole_Printf(
    const char *format,
    ...)
{
    /* 格式化后立即调用发送回调；无回调或超长文本不会阻塞命令处理。 */
    static char tx_buffer[DC_TX_BUFFER_SIZE];

    va_list arguments;
    int length;

    if ((dc_tx_function == NULL) ||
        (format == NULL))
    {
        return;
    }

    va_start(arguments, format);

    length = vsnprintf(
        tx_buffer,
        sizeof(tx_buffer),
        format,
        arguments);

    va_end(arguments);

    if (length <= 0)
    {
        return;
    }

    if (length >= (int)sizeof(tx_buffer))
    {
        length = (int)sizeof(tx_buffer) - 1;
    }

    dc_tx_function(
        (const uint8_t *)tx_buffer,
        (uint16_t)length);
}

/**
 * @brief 返回串口接收环形缓冲区溢出计数，便于定位通信丢字节问题。
 */
uint32_t DebugConsole_GetOverflowCount(void)
{
    return dc_overflow_count;
}
