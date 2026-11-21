# CLAUDE.md

本目录的工程约束已统一收敛到**仓库根目录**的 `AGENTS.md`（上一级 `../AGENTS.md`）。

请阅读 `../AGENTS.md`：源码布局、EIDE 构建流程、硬件引脚映射（已修正 OLED SCL/SDA 与 USART3 用途）、FreeRTOS 三任务结构、串口协议、控制参数、已知缺陷与硬规则。

本文件不再单独维护内容 —— 原内容中关于「两个任务」「OLED 每 10 秒刷新」「OLED SCL=PB5」「USART3 为辅助串口」的描述与源码不符。
