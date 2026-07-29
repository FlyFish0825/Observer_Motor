#include "debug_console.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* DMA一次接收的临时缓冲区 */
#define DC_DMA_RX_SIZE       64U

/*
 * 中断与主循环之间的环形缓冲区。
 * 必须是2的整数次幂。
 */
#define DC_RING_SIZE         256U

/* 单条文本命令最大长度，包含结尾'\0' */
#define DC_LINE_SIZE         128U

/* 一条命令最多允许的参数数量 */
#define DC_MAX_ARGS          10

/* 最大可注册变量数量 */
#define DC_MAX_VARIABLES     32U

/* 最大可注册自定义命令数量 */
#define DC_MAX_COMMANDS      16U

/* printf临时输出缓冲区 */
#define DC_TX_BUFFER_SIZE    192U

#if ((DC_RING_SIZE & (DC_RING_SIZE - 1U)) != 0U)
#error "DC_RING_SIZE must be a power of two"
#endif

typedef enum
{
    DC_VAR_F32 = 0,
    DC_VAR_I32,
    DC_VAR_U32,
    DC_VAR_BOOL
} DC_VariableType_t;

typedef union
{
    float f32;
    int32_t i32;
    uint32_t u32;
} DC_Value_t;

typedef struct
{
    const char *name;
    volatile void *address;

    DC_VariableType_t type;
    bool read_only;

    DC_Value_t minimum;
    DC_Value_t maximum;
} DC_Variable_t;

typedef struct
{
    const char *name;
    DebugConsole_CommandFn_t handler;
    const char *help;
} DC_Command_t;

/* ======================== 串口与DMA ======================== */

static UART_HandleTypeDef *dc_uart = NULL;
static DebugConsole_TxFn_t dc_tx_function = NULL;

static uint8_t dc_dma_rx_buffer[DC_DMA_RX_SIZE];

/* ======================== 环形缓冲区 ======================== */

static uint8_t dc_ring_buffer[DC_RING_SIZE];

static volatile uint16_t dc_ring_head = 0U;
static volatile uint16_t dc_ring_tail = 0U;

static volatile uint32_t dc_overflow_count = 0U;
static volatile bool dc_rx_restart_pending = false;

/* ======================== 行缓冲区 ======================== */

static char dc_line_buffer[DC_LINE_SIZE];
static uint16_t dc_line_length = 0U;
static bool dc_line_overflow = false;

/* ======================== 变量和命令表 ======================== */

static DC_Variable_t dc_variables[DC_MAX_VARIABLES];
static uint16_t dc_variable_count = 0U;

static DC_Command_t dc_commands[DC_MAX_COMMANDS];
static uint16_t dc_command_count = 0U;

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

static void DC_RingPushFromISR(uint8_t data)
{
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

static bool DC_RingPop(uint8_t *data)
{
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

static int DC_Tokenize(
    char *line,
    char *argv[],
    int maximum_arguments)
{
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

static void DC_CommandList(void)
{
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

static void DC_CommandGet(
    int argc,
    char *argv[])
{
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

static void DC_CommandSet(
    int argc,
    char *argv[])
{
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

static bool DC_ParseI32(
    const char *text,
    int32_t *result)
{
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

static bool DC_ParseU32(
    const char *text,
    uint32_t *result)
{
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

bool DebugConsole_RegisterF32(
    const char *name,
    volatile float *value,
    float minimum,
    float maximum,
    bool read_only)
{
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

bool DebugConsole_RegisterI32(
    const char *name,
    volatile int32_t *value,
    int32_t minimum,
    int32_t maximum,
    bool read_only)
{
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

bool DebugConsole_RegisterU32(
    const char *name,
    volatile uint32_t *value,
    uint32_t minimum,
    uint32_t maximum,
    bool read_only)
{
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

bool DebugConsole_RegisterBool(
    const char *name,
    volatile uint32_t *value,
    bool read_only)
{
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

static DC_Command_t *DC_FindCommand(
    const char *name)
{
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

uint32_t DebugConsole_GetOverflowCount(void)
{
    return dc_overflow_count;
}