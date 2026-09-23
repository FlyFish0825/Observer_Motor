# Observer_Motor

STM32G431CBT6 无位置传感器 FOC 电机控制工程。当前 CAN FD 测试使用 1 Mbit/s 仲裁段、1 Mbit/s 数据段并关闭 BRS，节点号固定为 1～8。

当前为 CAN 链路测试版本：普通 CAN FD 基础反馈约 100 Hz，保留经典 CAN 上电 HELLO、每秒心跳和进入 Bootloader 命令。切换到8 Mbit/s前需同步打开BRS并修改数据段时序。

## CAN FD 协议

网络中节点号固定为 Node1～Node8；上位机预先配置节点号，APP 不提供发现、注册或动态分配。FDCAN 接收使用环形缓冲区，反馈由 TIM6 硬件时隙调度。

### 总线参数

- 标准 11 位 CAN ID，多字节整数均为小端序；
- 仲裁段 1 Mbit/s，当前 CAN FD 数据段也是 1 Mbit/s，关闭 BRS；切换到 8 Mbit/s 数据段时，必须同步打开 BRS 并修改 FDCAN 数据段时序；
- CAN FD 帧不使用应用层 CRC；Bootloader、心跳和 HELLO 等经典 CAN 帧使用 CRC8：Poly=`0x07`、Init=`0x00`、无反射、XorOut=`0x00`；
- 调试帧三相电流、Id/Iq 的单位为 `0.01 A`，电压单位为 `0.01 V`，转速为有符号 `int16_t rpm`；
- 普通反馈的母线电流和温度使用独立的无符号满量程编码，详见下表。

### 固定 ID 分配

| ID | 方向 | 含义 | 长度 |
|---:|:---:|---|---:|
| `0x000` | 上位机 → 节点 | APP 进入 Bootloader | 8 字节经典 CAN |
| `0x100` | 上位机 → 全部节点 | 速度、运行、调试控制 | 24 字节 CAN FD |
| `0x180 + NodeID` | 节点 → 上位机 | ENTER_BOOT 接收确认 | 8 字节经典 CAN |
| `0x200 + NodeID` | 节点 → 上位机 | 普通运行反馈 | 12 字节 CAN FD |
| `0x280 + NodeID` | 节点 → 上位机 | 在线心跳 | 8 字节经典 CAN |
| `0x300 + NodeID` | 节点 → 上位机 | 调试反馈 | 64 字节 CAN FD |
| `0x380 + NodeID` | 节点 → 上位机 | APP 上电 HELLO（仅一次） | 8 字节经典 CAN |

例如 Node1 的普通反馈为 `0x201`，Node8 为 `0x208`。

### 广播控制帧 `0x100`

控制帧固定为 24 字节 CAN FD，当前关闭 BRS。固件同时兼容旧版 8 字节经典 CAN 的 RUN/SET_SPEED 调试帧；新上位机应使用下述广播帧。

| Byte | 字段 | 说明 |
|---:|---|---|
| 0 | Version | 固定为 `0x01` |
| 1 | Command | 命令码 |
| 2 | NodeMask | bit0～bit7 对应 Node1～Node8 |
| 3 | RunMask | `SPEED_VECTOR` 中 bit=1 请求启动；`RUN_VECTOR` 中 bit=1 启动、bit=0 停止 |
| 4～5 | Sequence | 上位机递增序号，固件不强制连续 |
| 6～7 | Flags | 保留，必须为 `0` |
| 8～23 | Speed1～Speed8 | 每节点一个小端 `int16_t`，单位 rpm |

速度有效范围为 `-10000～10000 rpm`；越界时相应节点不更新速度，也不执行同一帧的启动请求。

| Command | 名称 | 行为 |
|---:|---|---|
| `0x10` | `SPEED_VECTOR` | `NodeMask` 选中的节点更新各自速度，`RunMask` 对应 bit=1 时启动 |
| `0x11` | `RUN_VECTOR` | 使用 `RunMask` 同时启停选中的节点 |
| `0x20` | `DEBUG_SELECT` | 仅能选择一个调试节点；Byte3 非零启用，Byte3 为零关闭 |
| `0x30` | `STATUS_ONCE` | `NodeMask` 选中的节点立即发送一帧普通反馈 |

未选中的节点忽略 `0x10`、`0x11`、`0x30`；所有节点都会执行 `0x20`。`DEBUG_SELECT` 的掩码必须为单 bit，空掩码、多 bit 掩码或 Byte3=0 都会退出调试并恢复普通反馈。

控制示例：设置 Node1=`1000 rpm`、Node2=`-1500 rpm`、Node3=`3000 rpm`，并启动前三个节点：

```text
ID 0x100，DLC 24
01 10 07 07 01 00 00 00 E8 03 24 FA B8 0B 00 00 00 00 00 00 00 00 00 00
```

### 普通反馈 `0x200 + NodeID`

每节点约 100 Hz 发送一帧 12 字节 CAN FD、关闭 BRS 的实际状态反馈。参考速度不上传。

| Byte | 类型 | 内容 |
|---:|---|---|
| 0～1 | `int16_t` | 实际机械转速，rpm |
| 2～3 | `uint16_t` | 估算母线电流，0～10 A 对应 0～65535，约 `0.0001526 A/LSB` |
| 4～5 | `uint16_t` | 母线电压，`0.01 V/LSB` |
| 6～7 | `uint16_t` | STM32G431 内部温度，-20～150 ℃ 对应 0～65535，约 `0.002594 ℃/LSB` |
| 8 | `uint8_t` | 电机状态：0=`IDLE`，1=`OPEN_LOOP`，2=`CLOSED_LOOP` |
| 9 | 位标志 | bit0=电流校准完成，bit1=速度环启用，bit2=电压限幅 |
| 10 | `uint8_t` | 反馈序号 |
| 11 | `uint8_t` | 保留，固定为 `0` |

母线电流未增加独立 ADC 采样。FOC 每周期计算 `Pe = 1.5 × (Vd × Id + Vq × Iq)`、`Ibus_est = Pe / Vbus`，再以 `Ibus_filter = 0.95 × old + 0.05 × new` 低通滤波。上位机解码为 `Ibus(A) = raw × 10 / 65535`；负估算值钳为 0 A，超过 10 A 钳为 10 A，母线电压不高于 0.1 V 时按 0 A 处理。

温度来自 STM32G431 片内温度传感器，不是电机温度或外部 NTC。上位机解码为 `Temperature(℃) = raw × 170 / 65535 - 20`；低于 -20 ℃ 和高于 150 ℃ 的值钳位到量程端点。ADC1 规则组每 10 ms 更新母线电压和内部温度。

### 调试反馈 `0x300 + NodeID`

仅由 `DEBUG_SELECT` 选中的节点发送，周期约 5 ms（200 Hz）。调试期间其余节点停止普通反馈，只保留每秒心跳。

| Byte | 内容 | 单位 |
|---:|---|---|
| 0～1 | 实际转速 | rpm |
| 2～3 | PLL 电角速度 | rad/s |
| 4～9 | 三相电流 Iu/Iv/Iw | `0.01 A` |
| 10～13 | Id/Iq | `0.01 A` |
| 14～17 | Ud/Uq | `0.01 V` |
| 18～19 | 母线电压 | `0.01 V` |
| 20～21 | MCU 温度 | `0.1 ℃` |
| 22～23 | 观测器电角度 | `0.01°` |
| 24 | 电机状态 | 同普通反馈 |
| 25～27 | 状态标志 | 校准、速度环、电压限幅 |
| 28 | 反馈序号 | 递增 |
| 29～63 | 保留 | 固定为 `0` |

### 心跳、HELLO 与 Bootloader

心跳 `0x280 + NodeID` 每秒发送一次 8 字节经典 CAN 帧：Byte0=NodeID，Byte1=状态，Byte2=电流校准完成，Byte3=调试模式，Byte4=`int8_t` MCU 温度（整数 ℃），Byte5=电压限幅，Byte6=当前反馈序号，Byte7=前 7 字节 CRC8。它只用于在线和状态监视，不是 100 Hz 普通反馈。

APP 初始化 FDCAN 后、启动周期反馈前，发送一次 `0x380 + NodeID` 的 8 字节经典 CAN HELLO：Byte0～4 为 ASCII `HELLO`，Byte5 为 NodeID，Byte6 为协议版本 `0x01`，Byte7 为 CRC8。HELLO 最多等待 10 ms 完成发送，失败不会阻塞电机主循环。

进入 Bootloader 使用经典 CAN `0x000`，例如 Node1 命令为：

```text
01 04 00 00 00 00 00 7B
```

带 Bootloader 的 APP 校验通过后关闭 PWM，并以 `0x180 + NodeID` 发送接收确认；Node1 示例为 `01 04 01 00 00 00 00 19`。随后写入备份寄存器并复位。Standalone 布局仅校验并忽略该跳转请求。

### 频率与带宽

TIM6 以 1 MHz 计数基准每 1 ms 调度一个时隙，共 10 个时隙；Node1～Node8 分别占用时隙 0～7，因此每节点普通反馈约 100 Hz，不使用软件延时。普通反馈单节点约 `1.9 kbit/s`，8 节点合计约占 15.4% 总线负载；心跳合计约 0.7%。单节点调试模式为 200 Hz × 64 B，约占 17%，其余节点仅保留心跳，总负载低于 20%。

## 构建

```text
cmake --preset Debug
cmake --build --preset Debug

cmake --preset Boot-Release
cmake --build --preset Boot-Release
```

- `Debug`/`Release`：独立 APP，向量表位于 `0x08000000`；
- `Boot-Debug`/`Boot-Release`：配合 Bootloader 的 APP，向量表位于 `0x08005000`。

当前板卡使用 24 MHz 外部晶振。若使用 16 MHz 板卡，先修改
`Core/Inc/board_config.h` 中的 `BOARD_HSE_HZ`。
