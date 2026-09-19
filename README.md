# Observer_Motor

STM32G431CBT6 无位置传感器 FOC 电机控制工程。CAN FD 使用 1 Mbit/s 仲裁段、8 Mbit/s 数据段，节点号固定为 1～8。

当前为 CAN 诊断版本：周期 CAN FD 反馈暂时关闭，保留经典 CAN 上电 HELLO、每秒心跳和进入 Bootloader 命令。恢复周期反馈前需排查 8 Mbit/s 数据段 Bus-Off 原因；具体开关见协议文档。

完整的上位机接口定义见：

- [CANFD_MOTOR_PROTOCOL.md](CANFD_MOTOR_PROTOCOL.md)

当前协议特点：

- 一帧 CAN FD 广播控制 8 个节点，每个节点可设置不同转速；
- 普通反馈 12 字节，包含实际转速、Iq、母线电压、MCU 内部温度、状态和故障标志；
- 调试模式只允许一个节点发送 64 字节高速调试反馈；
- 所有节点每秒发送一次固定心跳；
- APP 上电先发送一次经典 CAN `HELLO`，便于独立确认 CAN 链路；
- FDCAN 接收使用环形缓冲区，反馈使用 TIM6 硬件时隙调度；
- APP 只保留进入 Bootloader 的复位命令，不包含固件下载协议。

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
