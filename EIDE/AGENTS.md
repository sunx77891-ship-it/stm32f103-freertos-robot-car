# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Project Overview

STM32F103RC 双电机小车项目，PID 控制算法由 MATLAB 整定。运行 FreeRTOS V10.3.1，使用 STM32F1xx HAL 库。

**目录结构:**
- `EIDE/` — Embedded IDE 工作区（VS Code 配置、构建输出）
- `../RCT6_2motocar_PIDbymatlab/` — 固件源代码

## Build / Flash / Debug

所有操作通过 VS Code 的 EIDE 扩展（`cl.eide`）完成，没有独立的 Makefile 或 CLI 构建脚本。

| 操作 | VS Code 命令/Task |
|------|------------------|
| 构建 | `Ctrl+Shift+B` → `build`，或 F1 → `EIDE: Build` |
| 烧录 | F1 → `EIDE: Upload to Device`，或运行 task `flash` |
| 构建+烧录 | 运行 task `build and flash` |
| 重新构建 | 运行 task `rebuild` |
| 清理 | 运行 task `clean` |
| 调试 | F5（使用 cortex-debug + J-Link） |

**工具链:** Arm Compiler 5 (AC5)，位于 `F:\arm_kile\ARM\ARMCC\`。编译器为 `armcc`，汇编器为 `armasm`，链接器为 `armlink`。

**构建输出:** `EIDE/build/RCT6_2motocar_PIDbymatlab/`，产物包括 `.axf` (ELF)、`.hex` (Intel HEX)、`.s19` (SREC/S19)、`.map`。

**编译选项要点:** C99、-O3 优化、`--split_sections`（每函数一个 section）、`--diag_suppress=1 --diag_suppress=1295`。

## 硬件外设映射

| 外设 | 引脚 | 功能 |
|------|------|------|
| TIM1 CH1-4 | PA8-PA11 | 4 路 PWM 输出（20kHz），A4950 电机驱动:<br>PA8(CH1)=电机 B IN1, PA9(CH2)=电机 B IN2<br>PA10(CH3)=电机 A IN1, PA11(CH4)=电机 A IN2 |
| TIM2 CH1-2 | PA0-PA1 | 编码器模式(TI12)，电机 1 速度反馈 |
| TIM3 CH1-2 | PA6-PA7 | 编码器模式(TI12)，电机 2 速度反馈 |
| TIM6 | — | 系统 HAL 滴答时基 |
| USART1 TX/RX | PB6/PB7 | 115200-8N1，M 序列 CSV 数据输出（已 Remap） |
| USART3 TX/RX | PB10/PB11 | 115200-8N1，辅助串口 |
| OLED SCL/SDA | PB5/PB4 | 软件 I2C，SSD1306 0.96 寸（开漏输出+内部上拉） |
| GPIOB | PB12-15 | 推挽输出（Low speed） |
| GPIOC | PC6 | 推挽输出 |
| GPIOC | PC0-PC3, PC13-PC14 | 上拉输入（按键） |

**M 序列辨识实验 (`Mmethods/m_methods.c`):**
- 10 阶 LFSR PRBS，1023 位序列，每位保持 50ms（5×10ms 采样）
- CCR low=0 / CCR high=2800（占空比约 78%），在 freertos.c 中调整
- 通过 USART1 输出 CSV: `sample,time_ms,ccr,speed_a_count,speed_b_count`
- 编码器每 10ms 采样一次并自动清零

**OLED 显示 (`OLED_DisplayTask`):**
- 低优先级任务，每 10 秒刷新一次
- 显示内容: 电机 A/B 累计编码器脉冲、最近 10 秒脉冲增量
- 8×16 ASCII 字库，4 行文本显示

**注意:** TIM1 实际配置为预分频 0、周期 3599（20kHz），非 AGENTS.md 旧版描述的 10kHz。

**系统时钟:** HSE 外部晶振 → PLL ×9 → 72MHz SYSCLK。APB1 = 36MHz，APB2 = 72MHz。Flash latency = 2。

## FreeRTOS 配置要点

- 静态+动态内存分配（`heap_4.c`），总堆 20480 字节
- Tick = 1000Hz，`configMAX_PRIORITIES = 7`，支持 Mutex
- SysTick 作为 RTOS 心跳（`xPortSysTickHandler = SysTick_Handler`）
- 空闲任务使用静态分配
- 两个任务: `defaultTask`（Normal 优先级，M 序列，栈 128 words）和 `oledTask`（Low 优先级，OLED 显示，栈 256 words）

## 代码风格

- `.clang-format`: BasedOnStyle Microsoft，4 空格缩进，大括号 Linux 风格，不排序 includes，不限制列宽
- 编码格式: `files.autoGuessEncoding: true`
- 源文件遵循 STM32CubeMX 生成的 `/* USER CODE BEGIN */` / `/* USER CODE END */` 标记区域，自定义代码写入标记之间

## 项目文件说明

- `.eide/eide.yml` — EIDE 项目配置（虚拟目录结构、目标、编译选项、上传配置）
- `.eide/env.ini` — 环境覆盖
- `.eide/files.options.yml` — 按文件的编译器选项覆盖
- `.pack/Keil/STM32F1xx_DFP.2.3.0/` — MCU 器件包（StdPeriph 头文件、启动文件、Flash 算法）
- `.cmsis/` — CMSIS 头文件

## 注意事项

- 此项目不是标准 CMake/Makefile 构建。不能用 `make` 或 `cmake` 构建，必须通过 EIDE 扩展。
- AC5 编译器（armcc）使用 Keil 特有语法和 pragma。如果是 GCC 移植，需注意 `__CC_ARM` vs `__GNUC__` 条件编译路径。
- `STM32F10X_HD` 宏在编译时定义，匹配 STM32F103RC 高密度器件（256KB Flash）。
- 链接脚本（scatter file）由 EIDE 自动生成，位于 `build/RCT6_2motocar_PIDbymatlab/RCT6_2motocar_PIDbymatlab.sct`。
