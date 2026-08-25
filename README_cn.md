<!-- Chinese README -->
中文 | **[English](README.md)**

# stm32f103-freertos-robot-car

![STM32](https://img.shields.io/badge/MCU-STM32F103RC-03234B?style=flat-square&logo=stmicroelectronics&logoColor=fff)
![FreeRTOS](https://img.shields.io/badge/FreeRTOS-10.3.1-45B8AC?style=flat-square&logo=freertos&logoColor=fff)
![C](https://img.shields.io/badge/C-C99-A8B9CC?style=flat-square&logo=c&logoColor=fff)
![MATLAB](https://img.shields.io/badge/MATLAB-PID_tuning-E16737?style=flat-square&logo=mathworks&logoColor=fff)
![Arm Compiler](https://img.shields.io/badge/Arm_Compiler-5.06-0091BD?style=flat-square&logo=arm&logoColor=fff)

一台基于 STM32F103RC 的双轮小车：编码器闭环控速、IMU 航向控制、FreeRTOS 任务调度，还带了 MATLAB 系统辨识和 PID 整定脚本。

## 目录

- [这是什么](#这是什么)
- [控制结构](#控制结构)
- [硬件引脚](#硬件引脚)
- [按键功能](#按键功能)
- [跑起来](#跑起来)
- [项目结构](#项目结构)
- [MATLAB 整定流程](#matlab-整定流程)
- [参与贡献](#参与贡献)

## 这是什么

- **双轮速度闭环** — 每 10ms 采集一次编码器，速度环使用前馈、位置式 PI、目标斜坡和条件积分抗饱和。
- **航向保持与转向** — 把 IMU 的 yaw 和角速度反馈送入航向环和角速度环。
- **固定路径** — K4 启动顺时针四边路径，每边 20000 个双轮平均编码器脉冲，角点前分级减速，刹停后稳定转向 90°。
- **上电和运行反馈** — SSD1306 OLED 显示实时控制数据，蜂鸣器和 LED 提供上电、按键声光提示。
- **辨识与整定** — 项目自带 PRBS 数据采集、电机/转向建模和 MATLAB PID 整定脚本。

## 控制结构

```text
编码器 A/B --> 10ms 速度 PI + 前馈 --> A4950 PWM（20kHz）
IMU yaw      --> 50ms 航向环 -------------+
IMU wz       --> 10ms 角速度环 ----------+--> 左右轮差速修正
```

直行距离使用双轮编码器的平均值。靠近角点时，目标速度从 30 降到 20，再降到 10 脉冲/10ms。小车刹停后转向，只有航向误差小于 3°、角速度小于 5°/s 并连续稳定 200ms，才会进入下一条边。

## 硬件引脚

| 外设 | 引脚 | 功能 |
|---|---|---|
| TIM1 CH1–4 | PA8–PA11 | 两路 A4950 H 桥的 4 路 20kHz PWM |
| TIM2 CH1–2 | PA0–PA1 | 一路电机编码器输入 |
| TIM3 CH1–2 | PA6–PA7 | 另一路电机编码器输入 |
| USART1 | PB6/PB7 | 115200-8-N-1 调试和辨识数据输出 |
| USART3 | PB10/PB11 | 辅助串口 |
| SSD1306 OLED | PB5/PB4 | 软件 I2C，SCL/SDA |
| 按键 | PC13、PC14、PC0–PC3 | K1–K6，上拉输入，低电平有效 |
| LED | PB12–PB15、PC6 | 状态与按键反馈 |
| 蜂鸣器 | PC15 | 低电平有效，用于上电和按键提示 |

## 按键功能

| 按键 | 引脚 | 动作 |
|---|---|---|
| K1 | PC13 | 双轮以 30 脉冲/10ms 向前运行 |
| K2 | PC14 | 双轮以 30 脉冲/10ms 向后运行 |
| K3 | PC0 | 急停，优先级始终最高 |
| K4 | PC1 | 启动或取消顺时针固定路径 |
| K5 | PC2 | 禁用 |
| K6 | PC3 | 禁用 |

按 K1 或 K2 会先取消自动路径，再进入手动运动；再按一次 K4 也会取消路径。第一次测试转向时，请留出空间并随时准备按 K3。如果 IMU 安装方向导致实际向左转，将 `freertos.c` 中的 `RECT_CLOCKWISE_ANGLE` 从 `90.0f` 改为 `-90.0f`。

## 跑起来

### 需要的工具

- VS Code 和 [EIDE](https://marketplace.visualstudio.com/items?itemName=cl.eide) 扩展
- Arm Compiler 5.06（AC5）
- J-Link 或 OpenOCD 兼容下载器
- MATLAB（只有辨识和控制器整定需要）

### 编译和烧录

```bash
git clone https://github.com/wokaka209/stm32f103-freertos-robot-car.git
cd stm32f103-freertos-robot-car/EIDE
```

用 VS Code 打开 `RCT6_2motocar_PIDbymatlab.code-workspace`，在 EIDE 中选择 AC5 工具链。如果提示缺少器件包，安装 STM32F1 Device Family Pack。

| 操作 | VS Code / EIDE 命令 |
|---|---|
| 构建 | `Ctrl+Shift+B`，或 `EIDE: Build` |
| 重新构建 | 运行 task `rebuild` |
| 烧录 | `EIDE: Upload to Device`，或运行 task `flash` |
| 构建+烧录 | 运行 task `build and flash` |
| 调试 | 配置 cortex-debug 和 J-Link 后按 `F5` |

这不是 CMake/Makefile 工程，不要直接运行 `make` 或 `cmake`。当前 EIDE 配置使用 AC5、C99、`-O3` 以及每函数独立 ELF section。

## 项目结构

```text
.
├── EIDE/                              # EIDE 工作区、控制模块、OLED、按键和 MATLAB 文件
│   ├── Ctrl/                          # 速度、位置、航向和角速度控制
│   ├── MotorDrive/                    # A4950 PWM 与板级外设
│   ├── IMU/                           # 陀螺仪数据处理
│   ├── Key/                           # 按键消抖状态机
│   ├── OLED/                          # SSD1306 软件 I2C 驱动
│   └── *.m / *.mat / *.slx            # 辨识、PID 整定和 Simulink 文件
├── RCT6_2motocar_PIDbymatlab/         # STM32CubeMX 固件、HAL、FreeRTOS 和启动文件
├── Mmethods/                          # PRBS/M 序列辨识实验
└── motocontrl/                        # 早期的底层电机/编码器辅助代码
```

## MATLAB 整定流程

- `EIDE/identify_motor.m` — 根据编码器采样数据辨识电机模型。
- `EIDE/tune_motor_pid.m` — 整定双轮速度环。
- `EIDE/identify_yaw.m` 和 `EIDE/tune_yaw_pid.m` — 辨识并整定转向动力学。
- `EIDE/pso_pid.m` — 尝试使用粒子群优化 PID。
- `EIDE/PIDmotor.slx` — 在 Simulink 中查看电机控制环。

脚本使用的串口数据位于 `EIDE/csv/`。换了电机、编码器或电池后，先核对采样时间、PWM 范围、电机极性和编码器符号，不要直接套用旧模型。

## 参与贡献

修改固件前请先阅读 [`EIDE/AGENTS.md`](EIDE/AGENTS.md)。STM32CubeMX 生成文件里的自定义代码应放在 `USER CODE BEGIN/END` 区域；C 代码使用项目内的 `.clang-format`；提交 Pull Request 前请用 EIDE + AC5 完整构建一次。
