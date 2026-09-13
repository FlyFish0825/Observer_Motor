# STM32G431CBT6 无感FOC电机控制

本工程面向当前 STM32G431CBT6 电机控制板，使用三相电流采样、三相端电压采样、磁链观测器和速度/电流双闭环控制电机。

## 当前硬件配置

- MCU：STM32G431CBT6，LQFP48
- 系统时钟：170 MHz，外部高速晶振
- PWM：TIM1三相互补中心对齐PWM，控制周期40 μs（25 kHz）
- 电流采样：ADC注入组，由TIM1触发，在PWM中心附近采样
- ADC1/ADC2规则组和注入组均为12位、6.5T、无过采样；电流增益为0.0067138671875 A/计数
- ADC1注入序列结束JEOS产生控制中断；ADC2注入组不启用中断，结果由ADC1回调统一读取
- 母线电压：PA0 / ADC1_IN1
- U相端电压：PB12 / ADC1_IN11
- V相端电压：PA4 / ADC2_IN17
- W相端电压：PB11 / ADC1_IN14
- 调试串口：USART1，2,000,000 baud
- Boot通信：FDCAN1，名义速率500 kbit/s、数据速率5 Mbit/s；APP仅轮询接收进入Bootloader命令

当前电机模型参数：相电阻 `Rs=0.5 Ω`、相电感 `Ls=100 μH`、永磁磁链
`0.00284 Wb`、极对数 `7`。默认Id/Iq电流PI均为 `Kp=0.220、Ki=1257`；默认速度PI
为 `Kp=0.0005、Ki=0.005`，输出Iq参考限制为±5 A。
这组参数用于首次保守测试，实际上板仍需结合电流和阶跃响应继续微调。

磁链SRF-PLL默认使用 `Kp=400、Ki=40000`，对应自然频率约200 rad/s、阻尼比约1。
该配置降低鉴相噪声通过比例项直接形成的瞬时RPM波动，同时保留加减速跟踪能力；
PLL电角速度限幅按当前调试配置为±50000 rad/s。

电流PI不再使用固定的正负20 V输出限幅。每个控制周期根据实时母线电压计算
`Umax=Vbus×0.98/√3`，先限制Ud，再将圆形电压矢量的剩余幅值分配给Uq。
24 V母线时最大dq矢量约13.58 V；SVPWM仍保留最终母线跨度检查。该处理可以充分利用
正常线性调制区，同时防止PI在后级电压缩放时继续积分。速度环±5 A限流保持不变。

端电压采样电路为100 kΩ/5.1 kΩ分压和68 nF滤波。ADC端等效电阻约4.85 kΩ，对应截止频率约482 Hz。

## 控制流程

1. 上电后保持六路栅极驱动为低电平。
2. 校准三个内部运放和ADC。
3. 仅启动TIM1 CH4触发，采集静止状态下的三相电流零偏。
   `HAL_TIM_PWM_Start`会打开MOE，此时三相功率通道仍未使能。
4. 零偏校准完成后保持IDLE；收到串口 `set run 1` 后开启三相PWM，直接进入端电压观测器闭环。
5. 低速时观测器使用U/V/W实测端电压：
   - 绝对转速达到1200 rpm后，20 ms内渐变到占空比与母线电压重构值。
   - 绝对转速降到900 rpm以下后，20 ms内渐变回实测值。
   - 规则组ADC超过10 ms没有新数据时，自动退回重构电压。
6. 正常闭环调速使用20000 rpm/s速度斜坡，速度PI输出限制为±5 A。

进入IDLE后，下一个ADC1注入回调先关闭MOE与三相主/互补通道，保留CH4采样时基。
首次进入时清除PI积分、观测器历史、电压命令与SVPWM结果，并将CCR1~3置零；
IDLE期间不运行闭环或写回旧PWM，保留零偏、外部命令及电压测量。
直接修改状态变量由下一次采样中断执行关断，不是异步硬件急停。

> 静止状态没有可测反电动势，纯无感端电压直接闭环不能可靠辨识任意初始转子位置。首次测试应空载进行，并监视三相电流。

## 代码结构

- `Core/Src/main.c`：芯片与外设初始化、主循环、HAL回调转发。
- `Core/Src/motor_app.c`：电机应用流程、电流校准、直接闭环启动、FOC中断和VOFA发送。
- `Core/Src/board_adc.c`：母线及三相端电压规则组采样、换算和一致性快照。
- `Core/Src/controller.c`：通用PI、电流环、速度环和速度斜坡。
- `Core/Src/observer.c`：磁链观测器、PLL以及实测/重构电压融合。
- `Core/Src/foc_math.c`：Clarke/Park变换、SVPWM和电流换算。
- `Core/Src/debug_console.c`：串口参数读写控制台。
- `Core/Src/app_boot_control.c`：APP端Boot控制帧校验、节点号读取和软件复位返回Bootloader。

对应公共接口位于 `Core/Inc` 目录。CubeMX生成的外设文件继续保留在 `Core/Src`，应用逻辑应优先放入上述独立模块，避免再次扩大 `main.c`。

初始化顺序必须保持：`FOC_Data_Init()` → `MX_TIM1_Init()` → `MotorApp_Init()`。
TIM1初始化的用户代码会读取FOC中的ARR、触发位置和死区值；过晚初始化会将定时器配置成零值。
规则组保留逐通道EOC轮询，应用层在启动注入组后单独切换ADC1中断为JEOS。
ADC1规则组使用间断模式，每次软件触发一个Rank；读完DR后再触发下一Rank，
避免6.5T高速连续扫描时HAL轮询来不及读取导致丢失通道结果。
保留的ADC_Regular_Read_DMA接口不适用于当前间断配置，应用层使用BoardAdc_Update。

## 分批提交记录

### 第一批：电机驱动、串口启停与ADC规则组修复

提交 `653f0f9`，本批只处理已经上板验证的电机运行链路：

- `Core/Src/motor_app.c`：增加 `run` 启停请求、上电校准后保持IDLE、停止时安全关桥、
  `status` 分组诊断输出，以及文本回复与VOFA DMA发送互斥。状态切换仍在ADC1注入完成
  回调的控制节拍内执行，主循环只解析命令和发出请求。
- `Core/Src/debug_console.c`：允许CR、LF和CRLF三种命令结束方式，避免不同串口工具
  的换行配置导致命令没有响应。
- `Core/Src/adc.c`、`Core/Src/board_adc.c`、`Observer.ioc`：ADC1规则组改为间断模式，
  每次软件触发并读取一个Rank，依次获得母线、U相和W相电压；V相仍由ADC2规则组读取。
- `Core/Src/main.c`：删除临时的启动前阻塞串口发送，保留简洁的应用初始化、主循环和
  HAL回调转发结构。
- `Core/Inc/motor_app.h`：同步更新启动行为和主循环职责说明。

本批明确不包含APP下载、Bootloader、CAN下载协议、Flash偏移或链接脚本调整；这些内容
需要独立核对地址布局、升级失败恢复方式和CAN兼容性后，再按功能单独提交。

### 第二批：APP端CAN返回Bootloader控制

提交 `260cfdf`，增加APP运行期间返回Bootloader的最小控制路径：

- FDCAN1名义速率改为500 kbit/s，数据阶段保持5 Mbit/s，与现有 `CAN_FD_IAP`
  Bootloader一致；CubeMX的 `Observer.ioc` 同步保存相同参数。
- APP只接收标准ID `0x000`，使用掩码 `0x7FF` 精确匹配，其他标准帧、扩展帧和远程帧
  不进入该接收路径。
- 主循环轮询FIFO0并严格检查标准数据帧、经典CAN、8字节长度、目标节点、命令和CRC8。
- 收到合法 `ENTER_BOOT` 后写入 `TAMP->BKP0R` 的 `BOOT` 魔数并执行软件复位；APP不发送
  应答，重新进入Bootloader本身就是成功结果。
- APP不启动或喂IWDG，也不复制Bootloader状态机，只保留Trial Jump所需的最小返回能力。

### 第三批：独立版与Boot版双Flash布局

提交 `8ed201f`，同一套源码支持两种互不混淆的固件：

- `Debug`、`Release`：链接到 `0x08000000`，生成 `Observer_standalone.bin`，用于不带
  Bootloader的SWD直接烧录测试。
- `Boot-Release`：链接到 `0x08005000`，生成 `Observer_boot.bin`，用于Bootloader升级。
- Boot版APP区截止到 `0x0801F7FF`，不会覆盖 `0x0801F800~0x0801FFFF` 的2 KiB配置页。
- `APP_FLASH_START` 同时控制链接地址和 `SCB->VTOR`，避免代码地址与中断向量表地址不一致。
- `APP_WITH_BOOTLOADER` 只在Boot版启用CAN返回Bootloader逻辑；独立版编译为空入口。

## 数学计算约定

核心代码统一使用 `arm_math.h`，适合的数学运算优先使用CMSIS-DSP接口。
磁链幅值开方使用 `arm_sqrt_f32`，正余弦使用STM32硬件CORDIC，磁链角度使用现有快速atan2近似。
标量绝对值保留 `fabsf`，当前编译器将其生成FPU的 `VABS.F32` 指令；`isfinite` 保留用于排除NaN和无穷值。
当前版本的 `arm_math.h` 内部仍包含 `math.h`，GCC路径下的 `arm_sqrt_f32` 也会使用 `sqrtf`；更换头文件不等于移除全部数学库依赖，也不直接代表性能提升。

## 在线调试参数

串口波特率2,000,000，命令以换行结束。上电不自动启动：

```text
set speed 1500
set run 1
set run 0
```

`run 1`启动，`run 0`停止并重置控制历史；校准未完成时启动请求等待校准结束。
串口支持CR、LF或CRLF结尾。上电主循环输出`READY`，发送`status`可查看运行请求、
电机状态、校准完成标志、ADC中断次数和TIM1参数。IDLE不发送VOFA波形，便于读取文本；
`status`还输出DRIVE（母线及转速）、CURRENT（电流参考/反馈与输出电压）、
PWM（比较值与通道使能）和OBSERVER（磁链及电压权重），用于定位已启动但不转的问题。
这些诊断数据逐项读取，不是同一个控制周期的同步快照。
运行时可先发送`set just_float 0`关闭波形。文本回复会等待当前DMA帧结束，避免同时发送。
运行期间重复发送`run 1`不会重新启动。停止在命令解析后的下一次ADC控制中断执行。

实际端电压统一保存在 `foc.state.u_abc_measured`（a/b/c对应U/V/W，单位V），
其Clarke变换结果保存在 `foc.state.u_alpha_beta_measured`，母线电压仍为 `foc.state.vbus`。
`u_abc`、`u_alpha_beta`、`u_dq` 为控制器电压命令。BoardAdc保留采样发布缓冲，
控制中断接受完整快照后更新实测电压状态，观测器和phase_u/v/w调试变量均使用该状态。

- `id`、`iq`：电流参考值
- `speed`：目标机械转速，rpm
- `speed_kp`、`speed_ki`：速度PI参数
- `speed_slew`：运行调速斜率，rpm/s
- `speed_en`：速度环使能
- `run`：电机运行请求，0停止，1启动；与速度环使能独立
- `just_float`：VOFA JustFloat发送使能
- `vbus`：母线电压
- `phase_u`、`phase_v`、`phase_w`：三相端电压
- `volt_src`：1表示选择实测端电压，0表示选择重构电压
- `volt_weight`：实测端电压融合权重，范围0～1

## 构建

工程使用CMake预设构建。独立烧录调试版：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

独立烧录优化版：

```powershell
cmake --preset Release
cmake --build --preset Release
```

由Bootloader加载的偏移版：

```powershell
cmake --preset Boot-Release
cmake --build --preset Boot-Release
```

| 构建预设 | APP起始地址 | 可用Flash | 输出BIN | 用途 |
| --- | --- | --- | --- | --- |
| Debug | `0x08000000` | 128 KiB | `build/Debug/Observer_standalone.bin` | SWD独立调试 |
| Release | `0x08000000` | 128 KiB | `build/Release/Observer_standalone.bin` | SWD独立运行 |
| Boot-Release | `0x08005000` | 106 KiB | `build/Boot-Release/Observer_boot.bin` | Bootloader升级 |

不能把 `Observer_standalone.bin` 作为升级包写到 `0x08005000`，也不能把
`Observer_boot.bin` 直接烧到 `0x08000000`。修改 `.ioc` 并由CubeMX重新生成后，应检查
根目录 `CMakeLists.txt` 中的用户源文件列表仍包含 `board_adc.c`、`motor_app.c` 和
`app_boot_control.c`。

## APP返回Bootloader协议

该功能只在 `Boot-Release` 构建中启用。控制帧格式如下：

| 字节 | 内容 |
| --- | --- |
| Byte0 | 目标节点号 `1~8`，或广播地址 `0xFF` |
| Byte1 | `ENTER_BOOT = 0x04` |
| Byte2~6 | 保留协议字段，参与CRC计算 |
| Byte7 | Byte0~6的CRC8，初值 `0x00`，多项式 `0x07` |

APP从 `0x0801F800` 配置页固定头部读取节点号；仅当魔数 `CFG1`、配置版本、结构长度和
节点范围均有效时采用该节点号，否则回退到节点1。收到合法命令后APP开放备份域写权限，
写入与Bootloader一致的 `0x544F4F42` 魔数并立即软件复位。APP不擦写Flash、不修改升级
元数据，也不负责Trial判定；镜像校验、有效标记和升级恢复仍由Bootloader处理。
