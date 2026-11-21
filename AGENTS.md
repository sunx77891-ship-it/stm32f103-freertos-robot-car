# AGENTS.md

STM32F103RC + FreeRTOS 双轮差速底盘固件。本文件是给 AI 编码 agent 的**唯一权威操作约束**，不是教程。

- §1–§9：改代码的约束（布局、构建、引脚、任务、协议、控制、风格、缺陷、硬规则）
- §10–§11：动硬件的闸门（上板顺序、安全红线）与本机环境、已踩过的坑
- 原理、采购、学习路线：见《STM32_FreeRTOS_机器人底盘_中文项目手册》
- 逐阶段执行细则（每步的目标/前置/验收/需补充）：见 `docs/skill/SKILL.md`，同一份内容以 skill `stm32-robot-car-bringup` 形式被自动加载；教程全文在 `docs/bringup-tutorial.md`
- 文档地图与同步方式：见 `docs/README.md`。**改 skill 只改 `docs/`，再跑 `python docs/sync_skill.py`**

三份文件（本文件 / 手册 / skill）冲突时，**代码事实以本文件为准，上板顺序以 §10 为准**。

**改动前必读第 8 节「已知缺陷」和第 9 节「硬规则」。事实一律以源码为准，不以注释和 README 为准。**

---

## 1. 源码布局

| 路径 | 职责 |
|---|---|
| `EIDE/RCT6_2motocar_PIDbymatlab.code-workspace` | **唯一工程入口** |
| `RCT6_2motocar_PIDbymatlab/` | CubeMX 生成层：`Core/Src`（main / freertos / tim / usart / gpio）、`Drivers`（HAL + CMSIS）、`Middlewares`（FreeRTOS） |
| `EIDE/Ctrl/` | 速度环、位置环、航向环、角速度环、矩形路径接口 |
| `EIDE/MotorDrive/` | PWM 输出、方向控制、蜂鸣器、LED |
| `EIDE/IMU/` | USART3 IMU 帧解析 + USART1 命令接收 |
| `EIDE/Key/` | 按键消抖与按下事件 |
| `EIDE/OLED/` | SSD1306 软件 I2C 显示 |
| `EIDE/YawID/` | 航向模型 M 序列辨识实验 |
| `EIDE/*.m` `EIDE/*.slx` | MATLAB 辨识与 PID 整定脚本（离线，非构建依赖） |
| `EIDE/.eide/` | `eide.yml` 工程配置、`env.ini`、`files.options.yml` |
| `Mmethods/` | 电机 M 序列辨识，独立实验代码，主工程不参与编译 |
| `motocontrl/` | **早期废弃代码，不要引用、不要修改** |

`RCT6_2motocar_PIDbymatlab/MDK-ARM/*.uvprojx` 是旧 Keil 工程，与当前源码关联不完整，**不要用它构建**。

---

## 2. 构建与烧录

全部通过 VS Code 的 EIDE 扩展（`cl.eide`）完成。**仓库里没有 Makefile 和 CMakeLists.txt，也没有 CLI 构建脚本。**

| 操作 | 方式 |
|---|---|
| 构建 | `Ctrl+Shift+B`，或 F1 → `EIDE: Build` |
| 烧录 | F1 → `EIDE: Upload to Device`，或运行 task `flash` |
| 构建 + 烧录 | task `build and flash` |
| 重新构建 | task `rebuild` |
| 清理 | task `clean` |
| 调试 | F5（cortex-debug + J-Link） |

- 工具链：**Arm Compiler 5 (AC5)**，`armcc` / `armasm` / `armlink`。AC5 不随仓库分发，需单独安装并让 EIDE 找到 `armcc.exe`（本机参考路径 `F:\arm_kile\ARM\ARMCC\`）
- 器件包：`STM32F1xx_DFP 2.3.0`，首次打开时 EIDE 会提示下载
- 编译选项：C99、`-O3`、`one-elf-section-per-function`、`--diag_suppress=1 --diag_suppress=1295`、宏 `STM32F10X_HD`
- 产物：`EIDE/build/RCT6_2motocar_PIDbymatlab/`，含 `.axf` / `.hex` / `.s19` / `.map` / `.sct`
- 链接脚本 `.sct` 由 EIDE 自动生成，**不要手写或提交**

> 禁止 `cmake`、`make`、`ninja`。禁止为了「更现代」而引入 CMake/GCC 工程 —— 会破坏 AC5 特有语法与 pragma。

**本机实测基线（2026-11-20，`a33df53` + `428fa6a`）**：构建耗时 3 秒；ROM 25.1 KB / 256 KB（9.8%），**RAM 25.2 KB / 48 KB（52.5%）**；产物 `.axf` 893 KB、`.hex` 71 KB、`.s19` 76 KB。

- Flash 余量充裕（远低于 AC5 评估版 32 KB 上限，换授权也不会被代码量卡住）
- **RAM 已用一半是真正的瓶颈** —— 新增缓冲区、队列、任务栈前先按这个基线算余量，堆只有 20 KB

---

## 3. 硬件映射（以宏定义和 `gpio.c` 为准）

| 外设 | 引脚 | 说明 |
|---|---|---|
| TIM1 CH1 / CH2 | PA8 / PA9 | 电机 B IN1 / IN2，20 kHz PWM |
| TIM1 CH3 / CH4 | PA10 / PA11 | 电机 A IN1 / IN2，20 kHz PWM |
| TIM2 CH1 / CH2 | PA0 / PA1 | 编码器 1，TI12 模式 |
| TIM3 CH1 / CH2 | PA6 / PA7 | 编码器 2，TI12 模式 |
| USART1 TX / RX | PB6 / PB7 | 115200-8N1，已 Remap，接 PC（命令 + CSV） |
| USART3 TX / RX | PB10 / PB11 | 115200，**IMU 输入帧**，PB11 接模块 TX |
| USART2 | PA2 / PA3 | 已初始化，主流程未使用 |
| OLED SCL | **PB4** | 软件 I2C，开漏 |
| OLED SDA | **PB5** | 软件 I2C，开漏 |
| 按键 K1~K4 | PC13 / PC14 / PC0 / PC1 | 上拉输入，低电平按下 |
| 按键 K5 / K6 | PC2 / PC3 | 代码中已禁用 |
| LED1~LED5 | PC6 / PB15 / PB14 / PB13 / PB12 | 高电平点亮 |
| 蜂鸣器 | PC15 | 低电平响 |
| TIM6 | — | HAL 时基（刻意避开 SysTick） |

> ⚠️ `EIDE/OLED/oled_i2c.h` 的**注释**写「PB5=SCL、PB4=SDA」，与实际宏定义相反。接线和改代码都以 `OLED_SCL_PIN = GPIO_PIN_4`、`OLED_SDA_PIN = GPIO_PIN_5` 为准。
> 时钟：HSE 8 MHz → PLL×9 → 72 MHz；APB1 36 MHz（定时器 72 MHz）、APB2 72 MHz、Flash latency 2；TIM1 分频 0 / 周期 3599 → 20 kHz。

---

## 4. FreeRTOS 任务

| 任务 | 优先级 | 栈 | 周期 | 作用 |
|---|---|---|---|---|
| `defaultTask` | Normal | 256 | 10 ms 快环 + 50 ms 慢环 | 外设初始化、速度/角速度内环、位置/航向外环、路径状态机 |
| `oledTask` | Low | 256 | 200 ms | OLED 刷新，并通过 USART1 输出 yaw |
| `keyTask` | Low | 128 | `osDelay(10)`（软件延时，有漂移） | 按键扫描、串口命令、路径状态机触发 |

- 实建 **3 个任务**（README 和旧文档写「2 个」是错的）
- Tick 1000 Hz，`configMAX_PRIORITIES = 7`，`heap_4` 共 20480 字节，Idle 任务静态分配
- 快环用 `vTaskDelayUntil`，慢环由快环计数分频（`CTRL_SLOW_MS / CTRL_FAST_MS`）
- 同步机制：**只有一个二值信号量**，无队列、无互斥锁，控制器状态靠全局变量 + 临界区传递 —— 这是主要技术债，新增并发访问必须沿用现有临界区风格，不要擅自引入无保护的全局读写

---

## 5. 串口协议

### IMU 帧（USART3，逐字节状态机）

```text
0x5A  TYPE  L  H  SUM
```

- `TYPE = 0xAA` → 角速度：`wz = raw / 32768 × 2000`（°/s）
- `TYPE = 0xBB` → 航向角：`yaw = raw / 32768 × 180`（°）
- `raw = (int16_t)((H << 8) | L)`；`SUM` 为前 4 字节累加取低 8 位
- 模块不是 MPU6050，**不走 I2C，不要去写 I2C 驱动**

### 命令（USART1，ASCII 行命令，`\r\n` 结尾）

`K1` 前进 / `K2` 后退 / `K3` 停止 / `K4` 启动或取消矩形路径 / `K5` 解析器支持但主逻辑未用

### 数据输出（USART1）

正常运行：每 200 ms 输出 `yaw` 单值。辨识实验另用两种 CSV：
`sample,time_ms,ccr,speed_a_count,speed_b_count` 和 `sample,time_ms,diff_ccr,yaw_x10,wz_x10`

---

## 6. 控制结构

```text
航向外环 50 ms：Yaw 误差 → 目标角速度（直行有 ±3° 死区）
角速度内环 10 ms：Wz 误差 → 左右轮速度差（增量式 PI）
位置外环 50 ms：位置误差 → 基础速度（TURN 模式锁位置、限速 20）
速度内环 10 ms：轮速误差 → PWM CCR（斜坡 + 前馈 + 积分抗饱和）
```

关键参数（`EIDE/Ctrl/Ctrl.c`）：`SPD_KP 5.0 / SPD_KI 8.0 / KFF 60.0`、`YAW_KP 5.0 / YAW_KD 0.2`、`WZ_KP 2.0 / WZ_KI 19.0`、`POS_KP 8.0 / POS_KI 0.2 / POS_KD 0.5`。
速度单位是**「编码器脉冲 / 10 ms」**，不是 RPM —— 改参数时不要按 RPM 换算。

路径状态机（`freertos.c`）：`RECT_PATH_IDLE → SIDE → BRAKE → TURN → AFTER_TURN`，每边 `RECT_SIDE_PULSES 20000`，减速点 1500 / 500 脉冲，转向 `RECT_CLOCKWISE_ANGLE 90.0f`（本工程正角度 = 顺时针）。

---

## 7. 代码风格

- `EIDE/.clang-format`：基于 Microsoft，4 空格缩进，大括号 Linux 风格，不排序 includes，不限列宽
- CubeMX 生成的 `.c` 只在 `/* USER CODE BEGIN */` 与 `/* USER CODE END */` 之间写自定义代码
- 新增 C 代码必须能被 **AC5** 编译（C99，无 GCC 扩展、无 `__attribute__((...))` 随意用法）

---

## 8. 已知缺陷（改动时不要复制同类写法）

1. **OLED 缓冲区溢出**：`char line[17]` 配 `sprintf(line, "P:%+6ld Tp:%+5ld", ...)`，第三行会越界 → 改 `char line[32]` + `snprintf`
2. **周期漂移**：`Key_ScanTask` 用 `osDelay(10)` → 应改 `vTaskDelayUntil`
3. **无失效保护**：IMU 没有超时检测、USART1 命令没有超时停车、`K3` 只是软件逻辑不是硬件急停
4. **DMA 效率**：USART 每收 1 字节就重启一次 DMA → 应改循环 DMA + IDLE 中断 + 环形缓冲
5. **并发模型**：全局变量 + 临界区为主，仅 1 个二值信号量 → 新增功能需自己保证一致性
6. **极性注释不可信**：`Mmethods/m_methods.c` 的注释与 `MotorDrive.c` 的极性描述不一致 → 实测轮子方向和编码器符号为准

---

## 9. 硬规则

**不要做：**

- 不要运行 `cmake` / `make`，不要新建 CMakeLists.txt / Makefile
- 不要改控制周期（10 ms / 50 ms）、FreeRTOS 堆大小、TIM6 时基 —— 会让已整定的 PID 参数全部失效
- 不要相信引脚和极性的**注释**，一律以 `gpio.c` 和 `oled_i2c.h` 宏定义为准
- 不要给 IMU 写 I2C 驱动，不要在 UART 上按 MPU6050 寄存器协议解析
- 不要让固件编译依赖 MATLAB，`.m` 脚本只做离线整定
- 不要动 `motocontrl/` 和 `MDK-ARM/` 下的旧工程
- 不要引入需要动态内存分配的大块结构（堆只有 20 KB）

**应该做：**

- 控制逻辑改 `EIDE/Ctrl/`，调度与状态机改 `Core/Src/freertos.c`，外设初始化改 `Core/Src/{tim,usart,gpio}.c`
- 新增任务一律 `vTaskDelayUntil` 固定周期，栈 ≥ 128 words 并按实测余量给
- 扩展串口协议时加帧头 + 版本 + 长度 + 序号 + CRC16，兼容 `K1~K5` 现有文本命令
- 改完必须确认 AC5 可编译；涉及引脚/时钟的改动要在说明里写清验证方法（如「架空轮子、先接单电机」）

---

## 10. 上板闸门（严格串行，不得跳步）

§1–§9 管「代码怎么改」，本节管「什么时候可以给什么上电」。用户要求跳步时，**先说明跳过的后果，再按闸门走**。

| # | 阶段 | 出口判据（不满足不进下一阶段） |
|---|---|---|
| 0 | 版本与采购边界 | 编码器是 A/B 两相；IMU 确认输出 `5A AA` + `5A BB`、115200；OLED 是 I2C `0x3C`；驱动是 A4950（否则先改代码）。知道仓库无 LICENSE |
| 1 | 工具链 | `armcc --version` = `5.06`；EIDE `Configure Toolchain` 显示 ✔；DFP 2.3.0 能选到 `STM32F103RC`。**编译通过才算过，不是「装好了」就算** |
| 2 | 首次构建 | `EIDE/build/RCT6_2motocar_PIDbymatlab/` 出 `.axf` / `.hex` |
| 3 | 电源（**不接电机**） | 12 V / 5 V / 3.3 V 都量过、无短路、共地 |
| 4 | 烧录（**不接电机**） | OLED 显 `System Ready`、蜂鸣三声、能暂停与单步 |
| 5 | USART1 命令 | 115200-8-N-1 + **行尾 `\r\n`**；`K1` / `K2` / `K3` 有可观察响应 |
| 6 | IMU 帧 | 帧头 `0x5A`、115200、转动模块时 Yaw / Wz 变化 |
| 7 | 架空·逐电机 | 两轮都朝车头、编码器正转为正、`K3` 能停、共地。单电机不过不得接第二个 |
| 8 | 架空·速度闭环 | 目标速度从 **10** 起步（不是 30），观察启动 / 超调 / 振荡 / 堵转 |
| 9 | 矩形路径 | 先架空看状态机 `IDLE→SIDE→BRAKE→TURN→AFTER_TURN`，再落地低速、留足转向空间 |
| 10 | MATLAB 辨识调参 | **可选**，不是跑通前置条件 |
| 11 | 工程化（CRC / 看门狗 / DMA / ROS2） | **属后续阶段**，不是原工程已有能力 |

**「能烧录」≠「闭环通过」。** 这条界线必须守住。

### 10.1 安全红线（用户明确要求也先拦一道）

出现下列情形必须阻止并说明理由：

- 电机未架空就测方向 / 跑闭环 / 跑路径
- 未确认三路电压、无短路、共地就接电机
- 把 `K3` 描述或验收成「硬件急停」—— 它只是 `Key_ScanTask` 里的软件停止
- IMU 帧协议不匹配却继续调 PID（应先改 `EIDE/IMU/Gyro.c`）
- 转向方向反了却不查 `RECT_CLOCKWISE_ANGLE`（`90.0f` ↔ `-90.0f`）
- 工具链缺失就改去跑 CMake / Makefile
- 只 clone 了仓库就声称「严格复现核对提交 `a33df53`」

急停改造方向（**均未实现，禁止说成已有能力**）：A4950 的 EN 脚串硬件急停开关；`K3` 改 EXTI 高优先级中断；USART1 命令 **>200 ms 无更新即刹车**。

### 10.2 上板阶段确实缺的信息（标「需补充」，不许编数值）

- 电机 A/B 与 TIM2/TIM3 编码器的明确对应关系
- 「目标速度从 30 改成 10」的代码入口（当前只有 `K1` 固定 30 脉冲/10 ms）
- 超调量、上升时间、稳态误差、堵转、「立即停」的量化阈值
- 12 V / 5 V / 3.3 V 的容差与测量点；A4950 端子定义与 EN/SLEEP 接法
- IMU 商品型号、安装朝向、Yaw 零点、输出频率
- 矩形路径总边数、一圈回到起点的判定、「低速」的具体数值

---

## 11. 本机环境与已踩过的坑

**环境指纹（2026-11-20 实测，换机需要重配）**

| 项 | 实况 |
|---|---|
| 系统 | Windows x64 |
| VS Code / Git | `1.147.0` / `2.55.0.windows.3` |
| EIDE | `cl.eide@3.27.2`，.NET 6.0.24 已具备 |
| AC5 | `D:\keil5\ARM\ARMCC`（MDK Plus 5.24 自带，**u5 build 528**，已授权） |
| DFP | `Keil.STM32F1xx_DFP` 2.3.0 → `EIDE/.pack/Keil/STM32F1xx_DFP.2.3.0` |
| 下载器 | ST-Link V2（`VID_0483&PID_3748`），驱动 v2.1.0.0，问题码 0 |
| 串口 | CH340 = `COM3`（COM4/5/7/8 是蓝牙虚拟口，**别选错**） |
| 串口助手 | `ms-vscode.vscode-serial-monitor@0.13.1`，`lineEnding = \r\n`；中文界面里叫「串行监视器」，在面板视图里，不在终端列表 |
| CubeMX | 6.18.1 已装，**只可查看 `.ioc`**，不得生成工程 |

**坑（按踩到的顺序，全是本机实测）**

1. AC5 u7 装到 `C:\Program Files (x86)` → 报 `C9555E`（找不到许可）。已装 MDK 的机器**直接用 MDK 自带的 `ARM\ARMCC`**，别另装一份。
2. 首次构建报 `cannot open source input file "Gyro.h"` / `Periph.h` → `eide.yml` 的 `incList` 缺 `EIDE/IMU`、`EIDE/MotorDrive`（已在 `428fa6a` 补上；在更早提交上构建会复现）。
3. 构建失败时 EIDE 弹「无法找到编译器 arm-none-eabi-gcc」是**误报** —— 本工程用 AC5，`[TOOL]` 行已打印 `ARM Compiler 5.06`；真错误在 `unify_builder.log` 里往下找 `Error: #5`。
4. 串口命令「没反应」→ 九成是行尾设成了 `\n`，必须是 `\r\n`。
5. `eclipse-cdt.serial-monitor` 没有行尾设置项（已核查源码），别用它。
6. `EIDE/.pack/`、`EIDE/.cmsis/`、`EIDE/build/` 被 `.gitignore` 排除 → 换机器后 DFP 要重装；`.pack` 本身是 zip，可直接解到 `.pack/Keil/STM32F1xx_DFP.2.3.0/`，与 `eide.yml` 的 `packDir` 对齐。
7. 未接电机时发 `K1` 车轮不转是**正常的**，不要误判为串口失败 —— 用 OLED 模式变化或 200 ms 的 `yaw` 输出确认命令已收到。
