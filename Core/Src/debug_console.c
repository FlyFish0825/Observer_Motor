/*
 * @file debug_console.c
 * @brief 基于 UART DMA 接收和环形缓冲区的文本调试控制台实现。
 *
 * UART/DMA 回调只负责搬运字节，命令解析、变量访问和用户命令执行均在
 * 主循环 DebugConsole_Process() 中完成，避免在中断上下文执行耗时操作。
 * 本文件属于用户应用代码，不放置在 CubeMX 生成区内。
 */
#include "debug_console.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include "arm_math.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* DMA一次接收的临时缓冲区 */
#define DC_DMA_RX_SIZE       64U /* 单次 UART DMA 接收缓冲区大小（字节）。 */

/*
 * 中断与主循环之间的环形缓冲区。
 * 必须是2的整数次幂。
 */
#define DC_RING_SIZE         256U /* ISR 与主循环之间共享的环形缓冲区容量。 */

/* 单条文本命令最大长度，包含结尾'\0' */
#define DC_LINE_SIZE         128U /* 一行命令缓冲区大小，包含结尾空字符。 */

/* 一条命令最多允许的参数数量 */
#define DC_MAX_ARGS          10 /* 单条命令最多解析的参数个数。 */

/* 最大可注册变量数量 */
#define DC_MAX_VARIABLES     32U /* 可通过 get/set 访问的变量表容量。 */

/* 最大可注册自定义命令数量 */
#define DC_MAX_COMMANDS      16U /* 用户自定义命令表容量。 */

/* printf临时输出缓冲区 */
#define DC_TX_BUFFER_SIZE    192U /* printf 格式化时使用的临时发送缓冲区。 */

#if ((DC_RING_SIZE & (DC_RING_SIZE - 1U)) != 0U)
#error "DC_RING_SIZE must be a power of two"
#endif

typedef enum
{
    DC_VAR_F32 = 0, /* 单精度浮点变量。 */
    DC_VAR_I32,     /* 有符号 32 位整数变量。 */
    DC_VAR_U32,     /* 无符号 32 位整数变量。 */
    DC_VAR_BOOL     /* 用 0/1 表示的布尔变量。 */
} DC_VariableType_t;

typedef union
{
    float f32;      /* 浮点上下限。 */
    int32_t i32;    /* 有符号整数上下限。 */
    uint32_t u32;   /* 无符号整数或布尔上下限。 */
} DC_Value_t;

typedef struct
{
    const char *name;          /* 命令行中使用的变量名，指向静态字符串。 */
    volatile void *address;    /* 被访问的实际变量地址。 */

    DC_VariableType_t type;    /* 变量数据类型，决定解析和打印方式。 */
    bool read_only;            /* 为 true 时只允许读取，禁止 set。 */

    DC_Value_t minimum;        /* 可写变量允许的最小值。 */
    DC_Value_t maximum;        /* 可写变量允许的最大值。 */
} DC_Variable_t;

typedef struct
{
    const char *name;                 /* 用户命令名称。 */
    DebugConsole_CommandFn_t handler; /* 收到命令后的回调函数。 */
    const char *help;                 /* help 命令显示的说明文本。 */
} DC_Command_t;

/* ======================== 串口与DMA ======================== */

static UART_HandleTypeDef *dc_uart = NULL; /* 当前控制台绑定的 UART。 */
static DebugConsole_TxFn_t dc_tx_function = NULL; /* 字节发送回调。 */

static uint8_t dc_dma_rx_buffer[DC_DMA_RX_SIZE]; /* DMA 直接写入的接收缓存。 */

/* ======================== 环形缓冲区 ======================== */

static uint8_t dc_ring_buffer[DC_RING_SIZE]; /* ISR 到主循环的字节队列。 */

static volatile uint16_t dc_ring_head = 0U; /* 下一个写入位置。 */
static volatile uint16_t dc_ring_tail = 0U; /* 下一个读取位置。 */

static volatile uint32_t dc_overflow_count = 0U; /* 环形缓冲区溢出累计次数。 */
static volatile bool dc_rx_restart_pending = false; /* 请求主循环重启 DMA。 */

/* ======================== 行缓冲区 ======================== */

static char dc_line_buffer[DC_LINE_SIZE]; /* 当前正在接收的文本行。 */
static uint16_t dc_line_length = 0U; /* 当前行已写入的字符数。 */
static bool dc_line_overflow = false; /* 当前行是否超过缓冲区容量。 */

/* ======================== 变量和命令表 ======================== */

static DC_Variable_t dc_variables[DC_MAX_VARIABLES]; /* 注册变量表。 */
static uint16_t dc_variable_count = 0U; /* 已注册变量数量。 */

static DC_Command_t dc_commands[DC_MAX_COMMANDS]; /* 用户自定义命令表。 */
static uint16_t dc_command_count = 0U; /* 已注册自定义命令数量。 */

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

/* 绑定 UART 和发送回调，清空控制台状态并启动 DMA 空闲接收。 */
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

/* 启动一次 UART 接收至空闲事件的 DMA 操作，并关闭半传输中断。 */
static HAL_StatusTypeDef DC_StartReceive(void)
{
    HAL_StatusTypeDef status; /* DMA 启动操作返回的 HAL 状态。 */

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

/* HAL UART 接收事件回调：只把 DMA 收到的字节压入环形缓冲区。 */
void DebugConsole_OnRxEvent(
    UART_HandleTypeDef *huart,
    uint16_t size)
{
    uint16_t index; /* 遍历 DMA 缓冲区的字节下标。 */

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

/* HAL UART 错误回调：标记接收需要在主循环中恢复。 */
void DebugConsole_OnError(
    UART_HandleTypeDef *huart)
{
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

/* 在中断上下文写入一个字节；队列满时丢弃当前字节并计数。 */
static void DC_RingPushFromISR(uint8_t data)
{
    uint16_t head; /* 当前环形缓冲区写指针快照。 */
    uint16_t next; /* 写入当前字节后将使用的下一个位置。 */

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

/* 在主循环上下文取出一个字节，返回 false 表示队列为空。 */
static bool DC_RingPop(uint8_t *data)
{
    uint16_t tail; /* 当前环形缓冲区读指针快照。 */

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

/* 主循环服务函数：恢复 DMA、组装文本行并派发命令。 */
void DebugConsole_Process(void)
{
    uint8_t data; /* 从环形缓冲区取出的一个输入字节。 */

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
         * CR、LF均可结束一行；CRLF的第二个字符对应空行，不会重复执行。
         */
        if ((data == '\n') || (data == '\r'))
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

/* 将一行文本切分为参数并执行内置或用户注册命令。 */
static void DC_ParseLine(char *line)
{
    char *argv[DC_MAX_ARGS]; /* 指向本行各参数起始位置的指针数组。 */
    int argc;                 /* 本行实际解析出的参数数量。 */

    DC_Command_t *command;    /* 匹配到的用户自定义命令表项。 */

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

/* 按空白字符切分命令行；井号及其后的内容作为注释忽略。 */
static int DC_Tokenize(
    char *line,
    char *argv[],
    int maximum_arguments)
{
    char *position; /* 当前扫描位置。 */
    int argc = 0;   /* 已解析参数数量。 */

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

/* 输出内置命令和已注册用户命令的帮助信息。 */
static void DC_CommandHelp(void)
{
    uint16_t index; /* 命令表遍历下标。 */

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

/* 输出当前已注册变量及其实时值。 */
static void DC_CommandList(void)
{
    uint16_t index; /* 变量表遍历下标。 */

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

/* 处理 get 命令并打印指定变量。 */
static void DC_CommandGet(
    int argc,
    char *argv[])
{
    DC_Variable_t *variable; /* 根据命令参数查找到的变量表项。 */

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

/* 处理 set 命令，完成类型解析、范围检查和变量写入。 */
static void DC_CommandSet(
    int argc,
    char *argv[])
{
    DC_Variable_t *variable; /* 待写入的变量表项。 */

    float value_f32;   /* 解析出的浮点写入值。 */
    int32_t value_i32; /* 解析出的有符号整数写入值。 */
    uint32_t value_u32; /* 解析出的无符号整数或布尔写入值。 */

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

/* 按变量类型格式化输出一个变量当前值。 */
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

/* 将完整文本转换为有限的单精度浮点数。 */
static bool DC_ParseF32(
    const char *text,
    float *result)
{
    char *end;   /* strtof 停止解析的位置。 */
    float value; /* strtof 解析出的浮点值。 */

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

/* 将完整文本转换为范围内的有符号 32 位整数。 */
static bool DC_ParseI32(
    const char *text,
    int32_t *result)
{
    char *end;   /* strtol 停止解析的位置。 */
    long value;  /* strtol 返回的中间整数值。 */

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

/* 将完整文本转换为范围内的无符号 32 位整数。 */
static bool DC_ParseU32(
    const char *text,
    uint32_t *result)
{
    char *end;           /* strtoul 停止解析的位置。 */
    unsigned long value; /* strtoul 返回的中间整数值。 */

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

/* 解析 0/1、on/off、true/false、yes/no 等布尔拼写。 */
static bool DC_ParseBool(
    const char *text,
    uint32_t *result)
{
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

/* 检查变量名、地址、容量和重名条件是否允许注册。 */
static bool DC_CanRegisterVariable(
    const char *name,
    const volatile void *address)
{
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

/* 注册可读写或只读的浮点变量及其合法范围。 */
bool DebugConsole_RegisterF32(
    const char *name,
    volatile float *value,
    float minimum,
    float maximum,
    bool read_only)
{
    DC_Variable_t *variable; /* 新变量在注册表中的目标槽位。 */

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

/* 注册可读写或只读的有符号整数变量及其合法范围。 */
bool DebugConsole_RegisterI32(
    const char *name,
    volatile int32_t *value,
    int32_t minimum,
    int32_t maximum,
    bool read_only)
{
    DC_Variable_t *variable; /* 新变量在注册表中的目标槽位。 */

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

/* 注册可读写或只读的无符号整数变量及其合法范围。 */
bool DebugConsole_RegisterU32(
    const char *name,
    volatile uint32_t *value,
    uint32_t minimum,
    uint32_t maximum,
    bool read_only)
{
    DC_Variable_t *variable; /* 新变量在注册表中的目标槽位。 */

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

/* 注册以 0/1 存储的布尔变量。 */
bool DebugConsole_RegisterBool(
    const char *name,
    volatile uint32_t *value,
    bool read_only)
{
    DC_Variable_t *variable; /* 新变量在注册表中的目标槽位。 */

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

/* 注册一个用户命令，并拒绝覆盖内置命令或已有同名命令。 */
bool DebugConsole_RegisterCommand(
    const char *name,
    DebugConsole_CommandFn_t handler,
    const char *help)
{
    uint16_t index; /* 用于检测自定义命令重名的遍历下标。 */

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

/* 按不区分大小写的名称查找变量表项。 */
static DC_Variable_t *DC_FindVariable(
    const char *name)
{
    uint16_t index; /* 变量表遍历下标。 */

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

/* 按不区分大小写的名称查找自定义命令表项。 */
static DC_Command_t *DC_FindCommand(
    const char *name)
{
    uint16_t index; /* 命令表遍历下标。 */

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

/* 比较两个以空字符结尾的字符串，忽略 ASCII 字母大小写。 */
static bool DC_StringEqualIgnoreCase(
    const char *left,
    const char *right)
{
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

/* 使用已注册发送回调输出格式化文本；过长内容会被截断。 */
void DebugConsole_Printf(
    const char *format,
    ...)
{
    static char tx_buffer[DC_TX_BUFFER_SIZE]; /* 格式化文本的发送缓存。 */

    va_list arguments; /* 可变参数遍历状态。 */
    int length;        /* vsnprintf 生成的字符数量。 */

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

/* 返回环形接收缓冲区累计丢弃的字节数。 */
uint32_t DebugConsole_GetOverflowCount(void)
{
    return dc_overflow_count;
}
