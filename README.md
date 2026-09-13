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

对应公共接口位于 `Core/Inc` 目录。CubeMX生成的外设文件继续保留在 `Core/Src`，应用逻辑应优先放入上述独立模块，避免再次扩大 `main.c`。

初始化顺序必须保持：`FOC_Data_Init()` → `MX_TIM1_Init()` → `MotorApp_Init()`。
TIM1初始化的用户代码会读取FOC中的ARR、触发位置和死区值；过晚初始化会将定时器配置成零值。
规则组保留逐通道EOC轮询，应用层在启动注入组后单独切换ADC1中断为JEOS。
ADC1规则组使用间断模式，每次软件触发一个Rank；读完DR后再触发下一Rank，
避免6.5T高速连续扫描时HAL轮询来不及读取导致丢失通道结果。
保留的ADC_Regular_Read_DMA接口不适用于当前间断配置，应用层使用BoardAdc_Update。

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

工程使用CMake预设构建：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

生成文件位于 `build/Debug`。修改 `.ioc` 并由CubeMX重新生成后，应检查根目录 `CMakeLists.txt` 中的用户源文件列表是否仍包含 `board_adc.c` 和 `motor_app.c`。
