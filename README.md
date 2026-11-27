![STM32](https://img.shields.io/badge/MCU-STM32-03234C?style=for-the-badge&logo=stmicroelectronics&logoColor=white)
![C](https://img.shields.io/badge/Language-C-00599C?style=for-the-badge&logo=c&logoColor=white)
![Arm](https://img.shields.io/badge/Architecture-ARM-0091BD?style=for-the-badge&logo=arm&logoColor=white)
![SolidWorks](https://img.shields.io/badge/CAD-SolidWorks-DB5C5D?style=for-the-badge&logo=dassaultsystemes&logoColor=white)

# Neurix Maze-Solving Robot

A compact, stable, and mid-speed autonomous maze-solving robot powered by the **STM32F411CEU6** microcontroller. Designed for reliable wall tracking, responsive control, and repeatable runs on standard maze courses.

## Overview

**Neurix** balances compact mechanical packaging with deterministic real-time control on the STM32F411CEU6 (ARM Cortex-M4, up to 100 MHz with FPU). It focuses on clean sensor feedback, stable closed-loop motion, and predictable mid-speed navigation rather than unstable high-speed extremes.

<img width="1600" height="1600" alt="Gallery" src="https://github.com/user-attachments/assets/28e3f503-20f6-4443-8ae7-66649d5940dc" />

## Key Features

- **Core MCU:** STM32F411CEU6 ("Black Pill") for high-frequency control loops, timers, PWM, and peripheral interfacing.
- **Compact & Stable Chassis:** Low-center-of-gravity layout engineered for consistent traction, smooth turns, and minimal oscillation.
- **Mid-Speed Tuned Performance:** Prioritizes repeatable path execution, reliable braking/cornering, and steady sensor readings.
- **Autonomous Maze Navigation:** Scans walls/paths, maps the maze, and computes routes to reach the goal efficiently.

## Components

| Component | Usage in Robot |
| :--- | :--- |
| **STM32 Black Pill (STM32F411CEU6)** | Main controller running sensor acquisition, PID motion control, and maze navigation logic |
| **TB6612FNG** | Dual DC motor driver controlling drive motor direction and PWM speed |
| **Sharp GP2Y0A51SK0F** | Short-range IR distance sensor for detecting nearby walls and obstacles |
| **N20 300 RPM Motors with Encoders** | Drive motors with quadrature encoders for propulsion and wheel odometry/speed feedback |
| **MPU-6050** | 6-axis IMU (accelerometer + gyroscope) for orientation tracking, turn estimation, and motion stability |
| **OLED Display** | Onboard status display for mode selection, sensor/navigation debugging, and run telemetry |
| **MD0585 Buck Converters** | Step-down DC-DC regulators supplying regulated power rails to logic, sensors, and subsystems |

## Repository Structure

```text
Neurix-Maze-Solving-Robot/
├── Firmware/      # STM32F411CEU6 source code, drivers, and control logic
├── Hardware/      # PCB schematics, layout files, BOM, and 3D chassis CAD
└── README.md
