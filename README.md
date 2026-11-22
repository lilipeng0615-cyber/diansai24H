基于 **TI MSPM0G3507** 的小车嵌入式固件工程，面向 **2024 年全国大学生电子设计竞赛 H 题（自动行驶小车）**。

项目集成了 **ICM42688** 惯性测量单元与 **VQF** 6 轴四元数姿态解算、8 路模拟多路复用灰度巡线与现场白/黑校准、外部中断编码器测速与里程计、双轮速度环 + 转向/航向串级 PID 控制、**ST7735** SPI LCD 调试屏、四任务状态机（支持 `StartYaw` 基准航向锚定与里程保护），以及 **VOFA+** 实时波形监控与在线 PID 调参功能。

---

## 核心特性

- **VQF 6 轴姿态解算（`Algorithm/vqf_filter.c`、`Algorithm/imu_attitude.c`）**
  - 基于 ICM42688（±16g / ±2000 dps，50 Hz ODR，20 Hz UI 滤波 + 42 Hz 抗混叠滤波）。
  - 支持运动/静止在线陀螺仪零偏估计（小车静止待机时自动开启静止零偏估计，发车运行后自动关闭以防误估）。
  - 内置量程饱和保护与连续多圈偏航角解卷绕（`YawTotalAngle` / `YawRoundCount`）。
- **8 路灰度巡线与现场校准（`apply/GraySensor.c`）**
  - 通过单路 12 位 ADC（`ADC1_CH0`）配合 3 线模拟开关（MUX）轮询采集 8 路灰度值（每通道 8 次均值滤波）。
  - 支持按键触发的白底/黑线两点现场归一化校准，短暂丢线时自动保持上一次有效转向角。
- **闭环运动控制与安全保护（`apply/MotorControl.c`、`Task/task.c`）**
  - 左右轮编码器闭环速度环（位置式 PID）+ 灰度巡线转向环 / IMU 航向保持外环。
  - **基准航向锚定（`StartYaw`）**：任务 2/3/4 每次切换直行或斜线阶段时，基于发车初始航向角配合 `NearestSameYaw` / `NearestOppositeYaw` 计算目标航向，避免弯道出口误差累积。
  - **发车倒计时 & 急停**：选定任务后提供 1 秒倒计时（蜂鸣提示），倒计时或行驶过程中按任意键立即触发急停（`E-STOP!`）。
  - **里程保护**：离点盲区（`TASK_BLIND_DIST_CM = 15 cm`）抑制刚离点误检，单段最大里程上限（`TASK_MAX_LEG_DIST_CM = 250 cm`）防止丢线撞墙。
- **人机交互与在线调试（`Drive/LCD_ST7735.c`、`apply/vofa.c`）**
  - ST7735 LCD 在待机状态下以 20 Hz 逐行增量刷新灰度掩码、偏航角、任务状态、轮速与 PWM 输出。
  - 通过串口以 20 Hz 向 VOFA+ 发送 6 通道 `JustFloat` 实时波形，并支持 11 字节协议在线修改左右轮速度环 PID 参数。

---

## 1. 开发环境与工具链

- **主控芯片**：TI `MSPM0G3507`（LQFP-64，主频 80 MHz）
- **MSPM0 SDK**
- **SysConfig**
- **构建方式（二选一）**：
  - **Keil MDK（ARMCLANG V6）**：打开 `keil/M0.uvprojx`
  - **VS Code + CMake + Ninja + ARM GCC + OpenOCD**：使用 `CMakeLists.txt`、`CMakePresets.json`（`gcc` 预设）与 `openocd.cfg`（CMSIS-DAP / DAPLink SWD 烧录）建议使用方式2

---

## 2. 快速上手

### 第一步：编译与烧录

**方式 A — Keil MDK（推荐新手）**
1. 使用 Keil MDK 打开 `keil/M0.uvprojx`，选择目标 `M0`。
2. 确认工程中的 MSPM0 SDK 路径与本机一致（默认 `C:\ti\mspm0_sdk_2_10_00_04`）。
3. 按 `F7` 编译，随后点击 **Download** 下载程序。

**方式 B — CMake + ARM GCC（命令行 / VS Code）**
> 注意：请先核对 `CMakeLists.txt` 中的 `SDK` 路径，以及 `CMakePresets.json` 中的 `CMAKE_MAKE_PROGRAM`（Ninja 路径）是否与本机一致。

```powershell
cmake --preset gcc
cmake --build --preset gcc
```

编译产物输出至 `build/` 目录，包含 `M0.elf`、`M0.hex` 和 `M0.bin`，可使用 OpenOCD 配合 DAPLink/CMSIS-DAP 烧录：

```powershell
openocd -f openocd.cfg -c "program build/M0.elf verify reset exit"
```

### 第二步：上电初始化与自检

上电后请保持小车**静止水平放置约 2 秒**：
- `ICM42688_Init()` 进行陀螺仪静态零偏标定（自动检查加速度与角速度抖动，最多重试 3 次）；
- `IMU_Attitude_Init()` 静止采样 50 次（约 1 秒）取平均重力方向，完成初始倾角快速对齐。

观察 ST7735 LCD 最底行（第 9 行状态栏）：
- **`IMU: OK`（绿色）**：IMU 初始化正常。
- **`IMU ERR:0xXX`（红色）**：`0x01` 表示未检测到芯片或 `WHO_AM_I` 不匹配，`0x02` 表示寄存器配置失败，请检查 `SPI0` 接线与片选引脚 `PB21`。

### 第三步：灰度传感器现场校准（建议更换场地后执行）

1. 待机状态下，**长按 K1（`PB18`）约 2 秒**，直至 LCD 状态栏显示 `CAL:WHITE K1/K3` 并蜂鸣提示。
2. 将 8 路灰度传感器全部置于**白色背景**上方，**按下 K1** 采样（过程中随时按 **K3** 可取消校准）。
3. LCD 显示 `CAL:BLACK K1/K3` 后，将传感器置于**黑线**上方，**按下 K1** 采样。
4. 等待状态栏显示 `CAL: DONE!` 并听到确认提示音，校准完成。

### 第四步：按键操作与急停

| 按键 | 引脚 | 待机状态操作 | 运行/倒计时状态操作 |
| :--- | :--- | :--- | :--- |
| **K1** | `PB18` | **单击**：启动任务 1（A → B 直线）<br>**长按（2s）**：进入灰度白/黑校准 | 立即触发急停（`E-STOP!`） |
| **K2** | `PB20` | **单击**：启动任务 2（A → B → C → D → A 一圈） | 立即触发急停（`E-STOP!`） |
| **K3** | `PB22` | **单击**：启动任务 3（A → C → B → D → A，1 圈）<br>**双击**：启动任务 4（A → C → B → D → A，4 圈） | 立即触发急停（`E-STOP!`） |

选定任务后，LCD 状态栏显示 `T* ARMING...` 并进入 **1 秒发车倒计时**（在 0.5 秒和 0 秒时蜂鸣提示），方便手扶放车。

---

## 3. 项目目录结构与调度时序

```text
diansai24H/
├── main.c                  主函数与前后台调度循环（50Hz IMU、50Hz 控制、20Hz VOFA、待机 20Hz LCD）
├── M0.syscfg               SysConfig 外设与引脚配置文件
├── ti_msp_dl_config.c/.h   SysConfig 自动生成的底层初始化代码（请勿手动修改）
├── Algorithm/              核心算法层
│   ├── vqf_filter.c/.h       VQF 6 轴四元数姿态解算与在线陀螺仪零偏估计
│   ├── imu_attitude.c/.h     IMU 姿态封装、上电重力对齐与连续多圈偏航角解卷绕
│   └── pid.c/.h              位置式 / 增量式通用 PID 控制器
├── apply/                  应用模块层
│   ├── ICM42688.c/.h         ICM42688 SPI 驱动、硬件滤波器配置与上电零偏校准
│   ├── GraySensor.c/.h       8 路 ADC 灰度采样、归一化现场校准与加权转向偏差计算
│   ├── Encoder.c/.h          编码器速度/位移解算（一阶低通滤波 + 死区清零）
│   ├── MotorControl.c/.h     双轮速度环、灰度巡线外环、航向保持外环控制
│   ├── Time.c/.h             硬件定时器中断调度标志管理
│   └── vofa.c/.h             VOFA+ JustFloat 波形发送、FireWater 辅助输出与在线调参解析
├── Drive/                  底层硬件驱动与公共配置
│   ├── app_config.h          [核心配置文件] PID 参数、任务速度、轮径/轮距、里程保护参数
│   ├── databyte.h            全局结构体与协议枚举定义
│   ├── headfile.h            统一头文件汇总
│   ├── Key.c/.h              三按键状态机（单击/双击/长按/连发）与任务/急停处理
│   ├── LCD_ST7735.c/.h       ST7735 SPI LCD 驱动与增量调试界面
│   ├── lcdfont.h             ASCII 及中文字库
│   ├── motor.c/.h            L298N 方向与 PWM 输出驱动
│   ├── EncoderExti.c/.h      左右轮编码器上升沿外部中断计数
│   ├── Beep.c/.h / LED.c/.h  蜂鸣器与声光提示 LED 驱动
│   ├── spi.c/.h / uart.c/.h  SPI0/SPI1 与串口环形缓冲帧解析驱动
│   └── Delay.c/.h            基于 SysTick 的 millis() / micros() / 阻塞延时
├── Task/                   任务状态机层
│   └── task.c/.h             电赛 24H 题目任务 1～4 状态机实现
├── keil/                   Keil MDK 工程文件、分散加载文件与启动汇编
├── cmake/                  ARM GCC 工具链配置（arm-gcc.cmake）
├── CMakeLists.txt          CMake 构建脚本
├── CMakePresets.json       CMake 预设（gcc + Ninja）
└── openocd.cfg             OpenOCD CMSIS-DAP SWD 烧录配置
```

### 定时器与任务调度频率

| 定时源 | 硬件周期 | 实际任务频率 | 执行函数 / 功能说明 |
| :--- | :--- | :--- | :--- |
| `TIMER_0`（`TIMG0`） | 10 ms | **50 Hz**（2 分频，20 ms） | `duty_imu_50hz()`：读取 ICM42688 并更新 VQF 姿态 |
| `TIMER_1`（`TIMA1`） | 20 ms | **50 Hz**（20 ms） | `duty_50hz()`：更新蜂鸣器、编码器测速、发车倒计时、执行任务 1～4 状态机及电机 PID |
| `TIMER_2`（`TIMA0`） | 50 ms | **20 Hz**（50 ms） | `Vofa_SendJustFloat()`：向 VOFA+ 发送 6 通道速度环实时数据 |
| `SysTick` | 1 ms | **1000 Hz**（1 ms） | 提供 `millis()` / `micros()` 时基，每 20 ms 扫描一次按键（`Key_Tick()`） |
| 主循环空闲刷新 | — | **20 Hz**（50 ms） | 仅在 `Carstartflag == 0` 待机时刷新灰度传感器数据并逐行增量更新 LCD |

---

## 4. 硬件引脚分配参考

以下引脚定义均与 `M0.syscfg` 保持一致（封装：`MSPM0G3507 LQFP-64`）：

| 功能模块 | 外设实例 | 引脚分配 |
| :--- | :--- | :--- |
| **电机 PWM** | `TIMG7`（`PWM_0`） | 左轮：`PA17`（CH0），右轮：`PA18`（CH1） |
| **电机方向（L298N）** | GPIO（`Motor`） | 左轮：`PA23` / `PB25`，右轮：`PB27` / `PA24` |
| **左轮编码器（EXTI）** | GPIO（`Encoder1`） | A1（上升沿中断）：`PB23`，B1（方向）：`PB24` |
| **右轮编码器（EXTI）** | GPIO（`Encoder2`） | A2（上升沿中断）：`PA25`，B2（方向）：`PA26` |
| **8 路灰度传感器** | `ADC1_CH0` + GPIO MUX | ADC 输入：`PA15`，通道选择（AD0～AD2）：`PB13` / `PB15` / `PA16` |
| **IMU（`ICM42688`）** | `SPI0`（`SPI_0`） | SCLK：`PA12`，MOSI：`PB17`，MISO：`PB19`，CS：`PB21` |
| **ST7735 LCD** | `SPI1`（`SPI_1`） | SCLK：`PB9`，MOSI：`PB8`，CS：`PB14`，RES：`PB10`，DC：`PB11`，BL：`PB26` |
| **三按键（低电平有效）** | GPIO（`Key`） | K1：`PB18`，K2：`PB20`，K3：`PB22` |
| **LED 与蜂鸣器** | GPIO / `TIMG6`（`PWM_Buzzer`） | LED：`PB16`，蜂鸣器：`PA29` |
| **VOFA+ 调试串口** | `UART0`（`UART_2`，115200） | TX：`PA10`，RX：`PA11` |
| **通用协议串口** | `UART3`（`UART_0`，115200） | TX：`PA14`，RX：`PA13` |
| **预留串口** | `UART2`（`UART_1`）、`UART1`（`UART_3`） | `PA21`/`PA22`、`PA8`/`PA9`（均 115200） |
| **SWD 调试接口** | `DEBUGSS` | SWCLK：`PA20`，SWDIO：`PA19` |

---

## 5. 任务状态机说明

四个任务均在 `Task/task.c` 中实现，每次过点或状态切换均伴有蜂鸣器与 LED 声光提示：

- **任务 1（`Task1`）**：A → B 航向保持直行，离点后经过 15 cm 盲区检测到黑线（`BinaryData != 0xFF`）或达到最大保护里程（250 cm）即停车。
- **任务 2（`Task2`）**：A → B 航向直行 → B → C 灰度巡线 → 连续 5 个周期全白判定到达 C 点 → 锚定 `StartYaw ± π` 沿 C → D 航向直行 → D → A 灰度巡线 → 到达 A 点停车。
- **任务 3（`Task3`）与任务 4（`Task4`）**：共用 `Task34_Run(car, lap)`（任务 3 跑 1 圈，任务 4 跑 4 圈）：
  - A → C 按斜线基准航向行驶 → C → B 灰度巡线 → 到达 B 点后基于初始 AB 轴线锚定 `B → D` 斜线航向（默认对角线偏角 `TASK34_DIAG_ANGLE = 38.66°`）→ D → A 灰度巡线 → 到达 A 点累计圈数，未满圈数时重新锚定下一圈 `A → C` 航向继续行驶。

---

## 6. 参数整定与 VOFA+ 联调

### 关键参数配置（`Drive/app_config.h`）

实车调试前请优先核对以下宏定义：

- **机械与编码器参数**：`EncoderLines`（默认 `390.0f`）、`TireRadius`（默认 `6.5f` cm，请按实车核对半径/直径）、`WheelTrack`（默认 `15.0f` cm）。
- **内环速度 PID**：`BrushMotor_PID_KP / KI / KD`（默认 `10.0 / 0.8 / 5.0`，输出限幅 `1000.0`）。
- **灰度巡线转向环 PID**：`SelfTurn_PID_KP / KI / KD`（输入偏差范围约 `-30 ~ +30`）。
- **航向保持外环 PID**：`YawHold_PID_KP / KI / KD`（输入为偏航角误差，单位 **rad**）。
- **各任务速度与保护阈值**：`TASK1_SPEED`、`TASK2_STRAIGHT_SPEED`、`TASK2_TRACE_SPEED`、`TASK34_STRAIGHT_SPEED`、`TASK34_TRACE_SPEED`、`TASK_BLIND_DIST_CM`、`TASK_MAX_LEG_DIST_CM`。

### VOFA+ 波形监控与在线调参

使用 USB 转 TTL 模块连接 **`UART_2`（`PA10` TX / `PA11` RX）**，串口参数 **`115200 8N1`**：

1. **实时波形（JustFloat 协议，20 Hz）**
   在 VOFA+ 中选择 **JustFloat** 数据引擎，固件默认发送 **6 个浮点通道**：
   - `ch0` ~ `ch2`：左轮目标速度（`Left->set`）、左轮反馈速度（`Left->fdb`）、左轮 PID 输出（`Left->out`）
   - `ch3` ~ `ch5`：右轮目标速度（`Right->set`）、右轮反馈速度（`Right->fdb`）、右轮 PID 输出（`Right->out`）
   > 注：`apply/vofa.c` 中还保留了 `Vofa_SendEulerFireWater()` 与 `Vofa_SendGrayFireWater()` 两个 FireWater 文本输出辅助函数，可在需要单独调试姿态或灰度原始 ADC 时按需调用。

2. **在线修改 PID 参数（11 字节二进制帧）**
   ```text
   @ <Target> <Param> <Op> = <4-byte float (小端序)> ! #
   ```
   - `Target`：`L`（仅左轮速度环）、`R`（仅右轮速度环）、`S`（同时修改左右轮速度环）
   - `Param`：`P`（Kp）、`I`（Ki）、`D`（Kd）
   - `Op`：`1`（直接设定）、`2`（增加）、`3`（减少）
   - 合法数值范围为 `[0.0, 1000.0]`，帧超时时间为 `20 ms`，更新成功后自动调用 `PID_clear()` 清除历史积分。

---

## 7. ST7735 LCD 显示说明

| 行号 | 标签 | 显示内容说明 |
| :--- | :--- | :--- |
| 1 | `Gray:` | 8 位灰度传感器二值化掩码（十六进制，`0xFF` 为全白，`0` 位代表检测到黑线） |
| 2 | `Yaw :` | 归一化偏航角（`-180°` ~ `+180°`） |
| 3 | `YawT:` | 连续多圈累计偏航角（单位：度） |
| 4 | `Task:` | 当前任务编号 `T`、阶段状态 `S` 与已完成圈数 `L` |
| 5 | `Dyaw:` | 当前航向误差（单位：度） |
| 6 | `VL  :` | 左轮线速度（cm/s） |
| 7 | `VR  :` | 右轮线速度（cm/s） |
| 8 | `PWM :` | 左右轮电机当前 PWM 输出值 |
| 9 | *(状态栏)* | 启动自检（`IMU: OK` / `IMU ERR:0xXX`）、发车倒计时（`T* ARMING...`）、急停（`E-STOP!`）或灰度校准提示 |

---

## 8. SysConfig 与开发注意事项

- 修改引脚分配、时钟树或外设参数时，请使用 **SysConfig `1.28.0`** 编辑 `M0.syscfg` 并重新生成 `ti_msp_dl_config.c/.h`，不要直接手动修改自动生成的文件。
- `CMakeLists.txt` 中链接了 CMSIS-DSP 库（`arm_cortexM0l_math.a`）并定义了 `ARM_MATH_CM0PLUS`，便于后续扩展其他浮点信号处理算法。
