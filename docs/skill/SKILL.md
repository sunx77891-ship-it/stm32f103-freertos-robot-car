---
name: stm32-robot-car-bringup
description: 按阶段闸门推进 STM32F103RC + FreeRTOS 双轮机器人底盘（wokaka209/stm32f103-freertos-robot-car）的上板落地：工具链 AC5+EIDE、首次构建、无电机电源与烧录、USART1 命令、IMU 串口帧、架空电机与编码器、速度闭环、矩形路径、可选的 MATLAB 辨识。当用户询问该仓库的上板顺序、接线、烧录、串口/IMU 调试、电机测试、PID 前置条件，或问「下一步做什么 / 卡住了怎么办 / 能不能直接跑」时使用；也用于阻止跳步与危险操作（未架空测电机、把 K3 当硬件急停、协议不匹配就调 PID）。
agent_created: true
---

# STM32F103RC + FreeRTOS 机器人底盘：上板落地

## 定位

把「原仓库代码」变成「已验收的底盘」，按闸门串行推进。适用对象：STM32F103RC + FreeRTOS 双轮差速底盘，EIDE + AC5 工程（**不是** CubeIDE / CMake / Makefile）。

**使用时机**：用户问上板顺序、接线、烧录、EIDE/AC5 配置、串口或 IMU 调不通、电机不转、路径跑歪，或问「现在该做什么」。

**不适用**：纯原理讲解、硬件采购比价、ROS2 上层开发（这些属后续阶段，见 `references/bringup-tutorial.md` §12.8）。

## 事实来源约定（先读，再回答）

任何论断必须能归到下面四类之一，**不许混成一句「应该是」**：

| 标记 | 含义 |
|---|---|
| 〔手册〕 | 《STM32_FreeRTOS_机器人底盘_中文项目手册》有依据 |
| 〔源码〕 | 仓库代码/宏定义有依据（引脚、参数、任务、协议） |
| 〔外部〕 | 手册没写但可公开查证，必须给来源 |
| **需补充** | 无依据，**不补造**，直接告诉用户缺什么 |

**引脚、极性、任务数量、刷新周期一律以源码为准**，不以注释和 README 为准。已核对的冲突见仓库根 `AGENTS.md` §3 与 §8。

## 边界铁律

- **原工程主线（已声明具备）**：双电机闭环、IMU 航向控制、按键、OLED、固定矩形路径。
- **后续建议（未实现）**：硬件急停、IMU 超时停车、命令超时刹车、CRC、看门狗、DMA 环形缓冲、Linux/ROS2 接入。

任何时候不得把「后续建议」讲成「原工程已有功能」。这是本 skill 的第一条红线。

## 十二阶段闸门

严格串行。用户要求跳步时，**先说明跳过的后果，再按闸门走**。

| # | 阶段 | 出口判据 |
|---|---|---|
| 0 | 版本与采购边界 | 编码器 A/B 两相；IMU 出 `5A AA`+`5A BB`、115200；OLED 是 I2C `0x3C`；驱动 A4950；知道仓库无 LICENSE |
| 1 | 工具链 | `armcc --version` = `5.06`；EIDE `Configure Toolchain` 显 ✔；DFP 2.3.0 可选到 `STM32F103RC`。**编译通过才算过** |
| 2 | 首次构建 | `EIDE/build/RCT6_2motocar_PIDbymatlab/` 出 `.axf` / `.hex` |
| 3 | 电源（**不接电机**） | 12 V / 5 V / 3.3 V 量过、无短路、共地 |
| 4 | 烧录（**不接电机**） | OLED `System Ready`、蜂鸣三声、可暂停单步 |
| 5 | USART1 | 115200-8-N-1 + 行尾 `\r\n`，`K1`/`K2`/`K3` 有响应 |
| 6 | IMU 帧 | 帧头 `0x5A`、115200、转动时 Yaw/Wz 变化 |
| 7 | 架空·逐电机 | 两轮朝车头、编码器正转为正、`K3` 能停、共地 |
| 8 | 架空·速度闭环 | 目标速度从 **10** 起步，观察启动/超调/振荡/堵转 |
| 9 | 矩形路径 | 先架空看状态机，再落地低速、留足转向空间 |
| 10 | MATLAB 辨识调参 | 可选，非前置条件 |
| 11 | 工程化改造 | 属后续，不冒充原工程能力 |

**「能烧录」≠「闭环通过」。** 每次回答前先判断用户处在哪个闸门，只回答当前闸门及其前置。

## 逐阶段执行卡

每张卡按「目标 / 动作 / 出口」给，缺的信息标「需补充」。细则与原始措辞见 `references/bringup-tutorial.md`。

### 阶段 0：边界与采购

- 记录仓库 `https://github.com/wokaka209/stm32f103-freertos-robot-car`，手册核对提交 `a33df5363db73ece9d1ba13bfd834484612e7157`
- 采购前四项硬确认：编码器 A/B 两相、IMU 帧头、OLED 是 I2C 非 SPI、驱动是 A4950（用 TB6612 必须先改代码）
- **不要买**：普通 MPU6050 接 USART3（代码不读 I2C 寄存器）、JY901S 期待直接可用、只有单相测速的电机
- 需补充：车头机械定义、电机减速比/编码器线数/轮径、A4950 端子定义、电源容量

### 阶段 1：工具链

- Git → VS Code → 扩展 `cl.eide` → **AC5** → DFP 2.3.0 → 下载器驱动 → 串口监视器
- AC5：已装 MDK 的机器**直接用 MDK 自带的 `ARM\ARMCC`**；另装 u7 到 `C:\Program Files (x86)` 会报 `C9555E`（无许可）
- DFP：固定 2.3.0。`.pack` 本身是 zip，可直接解到 `EIDE/.pack/Keil/STM32F1xx_DFP.2.3.0/`，与 `eide.yml` 的 `packDir` 对齐
- 串口助手：只用 `ms-vscode.vscode-serial-monitor`（有 `lineEnding`）；`eclipse-cdt.serial-monitor` 没有行尾设置
- 出口：逐项打勾，`armcc --version` 必须是 `5.06 update N`

### 阶段 2：构建

- 入口只有 `EIDE/RCT6_2motocar_PIDbymatlab.code-workspace`；`Ctrl+Shift+B` 或 `EIDE: Build`
- 报 `cannot open source input file "Gyro.h"` / `Periph.h` → `eide.yml` 的 `incList` 缺 `EIDE/IMU`、`EIDE/MotorDrive`（`428fa6a` 已补）
- 弹「找不到 arm-none-eabi-gcc」是误报，真错误在 `unify_builder.log` 的 `Error: #5`
- 基线：ROM 25.1 KB（9.8%）、RAM 25.2 KB（**52.5%**）—— RAM 是瓶颈

### 阶段 3：电源（不接电机）

- 不接电机；万用表量 12 V / 5 V / 3.3 V；确认无短路；确认共地；不从 ST-Link 给电机供电
- 需补充：各轨容差、测量点、上电顺序

### 阶段 4：烧录（不接电机）

- `EIDE: Upload to Device` 或 task `flash`（上传器 OpenOCD + `interface: stlink`，openocd 路径见 `AGENTS.md` §11）
- 观察四项：`System Ready`、蜂鸣三声、能暂停、能单步。任一缺失不进下一阶段

### 阶段 5：USART1

- PB6/PB7（已 Remap），115200-8-N-1，USB-TTL 必须 3.3 V 电平
- 发 `K1\r\n` / `K2\r\n` / `K3\r\n`；**行尾必须是 `\r\n`**，否则「看起来没反应」
- 未接电机时车轮不转是正常的，用 OLED 模式变化或 200 ms 的 `yaw` 输出确认命令已收到

### 阶段 6：IMU

- IMU TX → PB11（USART3 RX），RX 可暂时不接，必须共地
- 帧：`0x5A TYPE L H SUM`；`0xAA`=Wz（`raw/32768*2000` °/s）、`0xBB`=Yaw（`raw/32768*180` °）
- 协议不匹配 → **先改 `EIDE/IMU/Gyro.c`，不要调 PID**
- 当前固件**没有** IMU 超时判断（属后续改造）

### 阶段 7：架空·逐电机

- 轮子离地；先只接一个电机 + 一个 A4950；发 `K1` 看方向；手动转轮看计数器符号；测 `K3`
- PWM：正转 `PA8/PA10=0`、`PA9/PA11=PWM`；反转相反；停止 `=3599`（主动刹车）
- 编码器：`delta_b = -(current_b - last_b)`（B 侧取反，镜像安装）
- `Mmethods/m_methods.c` 的极性注释与 `MotorDrive.c` 不一致 → 以实测轮向为准
- 需补充：TIM2/TIM3 与电机 A/B 的明确对应；「立即停」的阈值

### 阶段 8：架空·速度闭环

- 目标速度从 **10** 起步，不要直接 30；观察启动平顺性、超调、振荡、堵转
- 速度单位是「编码器脉冲 / 10 ms」，不是 RPM
- 需补充：把 `K1` 固定 30 改成 10 的代码入口；超调/稳态误差的量化阈值；普通固件下怎么记录阶跃 CSV

### 阶段 9：矩形路径

- 先架空：`K4` 启动/取消，观察 `IDLE→SIDE→BRAKE→TURN→AFTER_TURN`
- 再落地：低速、第一次转向留足空间、查 Yaw 正负
- 方向反了：`RECT_CLOCKWISE_ANGLE` 从 `90.0f` 改 `-90.0f`
- 参数：每边 20000 脉冲、巡航 30、减速点 1500/500、刹停误差 40、等待 200 ms、转向 90°、误差 ≤3°、角速度 ≤5°/s、稳定 200 ms
- 需补充：总边数、一圈判定、「低速」具体值、Yaw 零点与摆放基准

### 阶段 10–11：可选与后续

- MATLAB（`identify_motor.m` / `tune_motor_pid.m` / `identify_yaw.m` / `tune_yaw_pid.m` / `pso_pid.m` / `PIDmotor.slx`）只做离线整定，**不让固件依赖它**
- 工程化改造清单见仓库 `AGENTS.md` §8「已知缺陷」与 §9「硬规则」

## 安全红线（用户明确要求也先拦一道）

必须阻止并说明理由：

- 电机未架空就测方向 / 跑闭环 / 跑路径
- 未确认三路电压、无短路、共地就接电机
- 把 `K3` 说成或验收成「硬件急停」—— 它只是 `Key_ScanTask` 里的软件停止
- IMU 协议不匹配却继续调 PID
- 转向反了却不查 `RECT_CLOCKWISE_ANGLE`
- 工具链缺失就改去跑 CMake / Makefile
- 只 clone 就声称「严格复现 `a33df53`」

急停改造方向（**未实现**）：A4950 EN 脚串硬件开关；`K3` 改 EXTI 高优先级中断；命令 **>200 ms 无更新即刹车**。

## 回答格式

给「这一步怎么做」时固定六段，缺一项就说明为什么缺：

1. **目标** —— 这一步完成什么
2. **前置条件** —— 上一步已通过什么验收
3. **具体任务** —— 按序动作
4. **关键配置** —— 参数、文件、命令、观察项
5. **预期结果 + 验收方式** —— 正常看到什么，怎样判定可进下一步
6. **需补充** —— 当前无法严谨补齐的点

单位必须带：`speed`（脉冲/10 ms）、脉冲、`°`、`°/s`、CCR。

## 排障速查

| 现象 | 先查 | 处理 |
|---|---|---|
| 构建报找不到 `Gyro.h` / `Periph.h` | `eide.yml` 的 `incList` | 补 `EIDE/IMU`、`EIDE/MotorDrive` |
| 弹「找不到 arm-none-eabi-gcc」 | 是误报 | 看 `unify_builder.log` 的 `Error: #5` |
| 编译报 license 类错误 | AC5 许可 | 用 MDK 自带 ARMCC，别用独立版 u7 |
| 选不到 `STM32F103RC` | `EIDE/.pack/` 是否为空 | 重装 DFP 2.3.0，不换版本顶替 |
| 串口发命令没反应 | 行尾 | 改 `\r\n`；再查 COM 口（CH340 当前为 COM6，**端口号会变，以设备管理器为准**；别选蓝牙 COM4/5/7/8） |
| 车轮不转 | 是否接了电机 | 未接电机时属正常，用 OLED/`yaw` 输出验证 |
| 转向方向相反 | `RECT_CLOCKWISE_ANGLE` | `90.0f` ↔ `-90.0f` |
| IMU 数据不动 | 接线与协议 | TX→PB11、共地、115200、帧头 `0x5A`；不符先改 `Gyro.c` |

## 与其他文件的分工

| 文件 | 职责 |
|---|---|
| 仓库根 `AGENTS.md` | 代码事实与硬规则（布局、构建、引脚、协议、参数、缺陷）。**代码冲突以它为准** |
| 本 skill | 上板顺序、闸门、安全拦截、排障。**流程冲突以它为准** |
| `references/bringup-tutorial.md` | 原始长篇教程全文，含每步的完整措辞、外部依据链接、本机环境记录 |

仓库默认位置：`C:\Users\孙禧园\Desktop\stm32f103-freertos-robot-car`。

## References

- `references/bringup-tutorial.md` —— 新手保姆级落地教程全文（约 2200 行）。需要原始措辞、〔外部〕来源链接、完整「需补充」清单时读它；只想定位某节可 grep 关键词，如 `grep -n "阶段七"`、`grep -n "需补充"`、`grep -n "C9555E"`。
