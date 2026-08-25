<!-- English README -->
**[中文](README_cn.md)** | English

# stm32f103-freertos-robot-car

![STM32](https://img.shields.io/badge/MCU-STM32F103RC-03234B?style=flat-square&logo=stmicroelectronics&logoColor=fff)
![FreeRTOS](https://img.shields.io/badge/FreeRTOS-10.3.1-45B8AC?style=flat-square&logo=freertos&logoColor=fff)
![C](https://img.shields.io/badge/C-C99-A8B9CC?style=flat-square&logo=c&logoColor=fff)
![MATLAB](https://img.shields.io/badge/MATLAB-PID_tuning-E16737?style=flat-square&logo=mathworks&logoColor=fff)
![Arm Compiler](https://img.shields.io/badge/Arm_Compiler-5.06-0091BD?style=flat-square&logo=arm&logoColor=fff)

An STM32F103RC two-wheel robot car with encoder speed control, IMU heading control, FreeRTOS tasks, and MATLAB-based system identification and PID tuning.

## Table of Contents

- [What is this](#what-is-this)
- [Control architecture](#control-architecture)
- [Hardware map](#hardware-map)
- [Button controls](#button-controls)
- [Getting started](#getting-started)
- [Project layout](#project-layout)
- [MATLAB workflow](#matlab-workflow)
- [Contributing](#contributing)

## What is this

- **Closed-loop wheel speed** — both motors use encoder feedback sampled every 10 ms. The speed controller combines feedforward, position-form PI, target ramping, and anti-windup.
- **Heading hold and turning** — IMU yaw and angular velocity feed cascaded heading and yaw-rate loops.
- **Automatic route** — K4 runs a clockwise four-side path. Each side is 20,000 average encoder pulses, with staged deceleration and a stabilized 90° turn at every corner.
- **Useful feedback** — the SSD1306 OLED shows live control data; the buzzer and LEDs provide startup and button feedback.
- **Identification tools** — PRBS data collection and MATLAB scripts are included for motor and yaw model fitting and PID tuning.

## Control architecture

```text
Encoder A/B --> 10 ms speed PI + feedforward --> A4950 PWM (20 kHz)
IMU yaw     --> 50 ms heading loop -----------+
IMU wz      --> 10 ms yaw-rate loop ----------+--> left/right speed correction
```

Straight driving uses the average wheel distance and heading correction. Near a route corner, the target speed drops from 30 to 20 and then 10 pulses/10 ms. The car brakes, turns, and only starts the next side after yaw error is within 3° and angular speed is below 5°/s for 200 ms.

## Hardware map

| Peripheral | Pins | Purpose |
|---|---|---|
| TIM1 CH1–4 | PA8–PA11 | Four 20 kHz PWM outputs for two A4950 H-bridges |
| TIM2 CH1–2 | PA0–PA1 | First motor encoder input |
| TIM3 CH1–2 | PA6–PA7 | Second motor encoder input |
| USART1 | PB6/PB7 | 115200-8-N-1 diagnostic and identification output |
| USART3 | PB10/PB11 | Auxiliary serial port |
| SSD1306 OLED | PB5/PB4 | Software I2C, SCL/SDA |
| Buttons | PC13, PC14, PC0–PC3 | K1–K6, active low with pull-ups |
| LEDs | PB12–PB15, PC6 | Status and button feedback |
| Buzzer | PC15 | Active-low startup and key feedback |

## Button controls

| Button | Pin | Action |
|---|---|---|
| K1 | PC13 | Drive both wheels forward at 30 pulses/10 ms |
| K2 | PC14 | Drive both wheels backward at 30 pulses/10 ms |
| K3 | PC0 | Emergency stop; always has the highest priority |
| K4 | PC1 | Start or cancel the clockwise automatic route |
| K5 | PC2 | Disabled |
| K6 | PC3 | Disabled |

K1 or K2 cancels automatic mode before starting manual motion. Pressing K4 again also cancels the route. Test the first turn with the wheels clear and keep K3 ready; if your IMU mounting reverses the yaw sign, change `RECT_CLOCKWISE_ANGLE` from `90.0f` to `-90.0f` in `freertos.c`.

## Getting started

### Prerequisites

- VS Code with the [EIDE](https://marketplace.visualstudio.com/items?itemName=cl.eide) extension
- Arm Compiler 5.06 (AC5)
- J-Link or an OpenOCD-compatible probe
- MATLAB for identification and controller-tuning scripts (optional)

### Build and flash

```bash
git clone https://github.com/wokaka209/stm32f103-freertos-robot-car.git
cd stm32f103-freertos-robot-car/EIDE
```

Open `RCT6_2motocar_PIDbymatlab.code-workspace` in VS Code. Select the AC5 toolchain in EIDE and install the STM32F1 device pack if prompted.

| Action | VS Code / EIDE command |
|---|---|
| Build | `Ctrl+Shift+B`, or `EIDE: Build` |
| Rebuild | Run task `rebuild` |
| Flash | `EIDE: Upload to Device`, or task `flash` |
| Build + flash | Run task `build and flash` |
| Debug | Press `F5` with cortex-debug and J-Link configured |

This is an EIDE project, not a CMake or Make project. The checked configuration targets AC5 with C99, `-O3`, and one ELF section per function.

## Project layout

```text
.
├── EIDE/                              # Workspace, control modules, OLED, keys, MATLAB files
│   ├── Ctrl/                          # Speed, position, heading, and yaw-rate control
│   ├── MotorDrive/                    # A4950 PWM and board peripherals
│   ├── IMU/                           # Gyroscope data handling
│   ├── Key/                           # Debounced button state machine
│   ├── OLED/                          # SSD1306 software-I2C driver
│   └── *.m / *.mat / *.slx            # Identification and PID-tuning assets
├── RCT6_2motocar_PIDbymatlab/         # STM32CubeMX firmware, HAL, FreeRTOS, startup files
├── Mmethods/                          # PRBS/M-sequence identification experiment
└── motocontrl/                        # Earlier low-level motor/encoder helpers
```

## MATLAB workflow

- `EIDE/identify_motor.m` — fit a motor model from captured encoder data.
- `EIDE/tune_motor_pid.m` — tune the wheel-speed controller.
- `EIDE/identify_yaw.m` and `EIDE/tune_yaw_pid.m` — identify and tune rotational dynamics.
- `EIDE/pso_pid.m` — experiment with particle-swarm PID tuning.
- `EIDE/PIDmotor.slx` — inspect the motor control loop in Simulink.

Serial captures used by these scripts live in `EIDE/csv/`. Check sample time, PWM range, motor polarity, and encoder sign before reusing an identified model on different hardware.

## Contributing

Read [`EIDE/AGENTS.md`](EIDE/AGENTS.md) before changing the firmware. Keep custom STM32CubeMX code inside `USER CODE BEGIN/END` blocks, format C with the repository `.clang-format`, and verify changes with an EIDE/AC5 build before opening a pull request.
