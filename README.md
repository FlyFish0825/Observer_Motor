# STM32G431CBT6 无感 FOC 电机控制

面向 STM32G431CBT6 电机控制板的无传感器 PMSM FOC 固件。控制器用磁链观测器和 SRF-PLL 估算转子位置与转速，以电流环和速度环驱动三相逆变桥。

## 项目基础

- MCU：STM32G431CBT6；默认板载 HSE 16 MHz，经 PLL 得到系统时钟 170 MHz
- PWM 与电流环：TIM1 中心对齐互补 PWM，25 kHz；ADC 注入组由 TIM1 CH4 触发
- 实时控制：ADC1 注入序列 JEOS 完成后进入 FOC 控制中断
- 主循环：处理串口命令、CAN 收发调度、规则组电压采样和运行请求
- 调试接口：USART1（PB6/PB7），2,000,000 baud；FDCAN1（PA11/PA12），仲裁段和数据段均为 1 Mbit/s，关闭 BRS
- 电压与电流采样的引脚、量程和换算参数见[采样与板级文档](doc/03-采样与板级.md)

控制原理可以先记成两条路径：

~~~mermaid
flowchart LR
    Host[上位机] -->|串口命令 / CAN 控制| Main["主循环：协议轮询 → 应用处理"]
    Main -->|发布运行请求 / 控制目标| IRQ[ADC1 JEOS 中断：FOC 闭环]
    Timer[TIM1 CH4 触发] --> ADC[ADC 注入组]
    ADC --> IRQ
    IRQ -->|CCR1~3| PWM[TIM1 PWM / 三相桥] --> Motor[电机]
    IRQ -->|状态与反馈| Main
~~~

这张图只说明主循环和实时中断的职责边界。启动过程、数据交接、采样、控制和通信分别展开在[第一层总览](doc/01-系统架构总览.md)及下方文档中。

## 分层文档

由大到小阅读：先了解固件如何运行，再按模块看机制，最后查具体函数和变量。

| 层级 | 解决的问题 | 文档 |
|---|---|---|
| 第一层：大结构 | 主循环、中断、启动链和数据如何连接 | [系统架构总览](doc/01-系统架构总览.md) |
| 第二层：中结构 | 控制算法、采样与板级、外部通信分别如何工作 | [控制层](doc/02-控制层.md) · [采样与板级](doc/03-采样与板级.md) · [通信与调试](doc/04-通信与调试.md) |
| 第三层：小结构 | 项目函数的用途、参数、输出和返回值；结构体字段、共享状态和配置宏的含义 | [底层参考](doc/05-底层参考.md) |
| 补充资料 | 历史开发批次与数值计算约定 | [开发记录](doc/06-开发记录.md) |

第二层文档的末尾链接到第三层对应符号；第三层目录可按模块定位函数、字段和宏。

## 代码结构

| 目录 / 模块 | 职责 |
|---|---|
| Core/Inc | 应用模块接口、共享结构体和 CubeMX 外设句柄声明 |
| Core/Src/main.c | 启动顺序、主循环和 HAL 回调转发 |
| Core/Src/motor_app.c、controller.c、observer.c、foc_math.c | 电机状态管理、PI 控制、磁链观测和 FOC 数学计算 |
| Core/Src/board_adc.c | 规则组电压采样、换算和一致快照发布 |
| Core/Src/debug_console.c、motor_protocol.c、app_boot_control.c | 串口控制台、FDCAN 电机协议和 APP 返回 Bootloader |
| Core/Src/adc.c、dma.c、fdcan.c、gpio.c、opamp.c、tim.c、usart.c、cordic.c | CubeMX 外设初始化与 HAL MSP 配置 |
| Core/Src/bsp_dwt.c、AS5600.c | DWT 周期计时工具；AS5600 是保留源码接口，当前未编入固件 |
| Drivers、Middlewares | STM32 HAL、CMSIS-DSP 等依赖库 |
| doc | 分层架构和符号参考文档 |

应用层实时入口是 ADC1 JEOS 中断；CAN 接收中断将帧放入队列，主循环解析；串口 DMA 回调只搬运字节，命令解析留在主循环。详细边界见[第一层总览](doc/01-系统架构总览.md)。

## 构建

使用 CMake 预设。独立烧录调试版：

~~~powershell
cmake --preset Debug
cmake --build --preset Debug
~~~

独立烧录优化版：

~~~powershell
cmake --preset Release
cmake --build --preset Release
~~~

由 Bootloader 加载的版本：

~~~powershell
cmake --preset Boot-Release
cmake --build --preset Boot-Release
~~~

在原 `main` 使用的 STM32G431CBU6、HSE 24 MHz 板上，只验证 CAN 通信与 Boot 返回时，构建专用镜像：

~~~powershell
cmake --preset Boot-Release-HSE24
cmake --build --preset Boot-Release-HSE24
~~~

两个 Boot 镜像的 PLL 分别使用 M=4（16 MHz）和 M=6（24 MHz），均得到 170 MHz。CBU6 测试板串口为 USART2（PB3/PB4），此分支串口为 USART1（PB6/PB7）；原板串口口上不会看到本镜像的 READY。跨板测试只用于验证 CAN/Boot 通信，不能据此确认电机采样与驱动功能。

| 预设 | HSE | APP 起始地址 | 输出 BIN | 用途 |
|---|---|---|---|---|
| Debug | 16 MHz | 0x08000000 | build/Debug/Observer_standalone.bin | CBT6 板 SWD 独立调试 |
| Release | 16 MHz | 0x08000000 | build/Release/Observer_standalone.bin | CBT6 板 SWD 独立运行 |
| Boot-Release | 16 MHz | 0x08005000 | build/Boot-Release/Observer_boot.bin | CBT6 板 Bootloader 升级 |
| Boot-Release-HSE24 | 24 MHz | 0x08005000 | build/Boot-Release-HSE24/Observer_boot_hse24.bin | CBU6 板 CAN/Boot 临时验证 |

地址布局和 Boot 返回协议见[通信与调试](doc/04-通信与调试.md)及[底层参考](doc/05-底层参考.md)。

## 串口快速调试

串口为 2,000,000 baud。上电完成零偏校准后不会自动启动：

~~~text
set speed 1500
set run 1
status
set run 0
~~~

无感启动首次验证应空载进行，并观察三相电流。运行状态、可调变量和 CAN 命令详见[通信与调试](doc/04-通信与调试.md)。
