# Micromouse Software Architecture

## Overview

The Micromouse software is designed with a layered architecture, abstracting hardware details from the high-level maze-solving logic. The system prioritizes deterministic control loops, modularity, and memory efficiency suitable for an STM32F411 microcontroller running the Arduino Core.

## Layers

1. **Hardware Layer (`hardware/`)**
   - Directly interacts with MCU registers and pins.
   - Provides abstracted interfaces for Motors, Encoders, PWM, Timers, Buttons, Buzzer, and LEDs.
   - Hides STM32-specific register manipulations.

2. **Sensor Layer (`sensors/`)**
   - Interfaces with I2C devices (MPU6050, VL53L0X).
   - Manages sensor polling and filtering.
   - Provides high-level abstractions like "distance to wall" and "fused heading".

3. **Localization Layer (`localization/`)**
   - Fuses encoder odometry with gyro heading and wall-distance corrections.
   - Maintains the robot's absolute Pose (x, y, theta) in the maze.

4. **Control Layer (`control/`)**
   - Implements closed-loop PID controllers.
   - Cascading architecture: Trajectory -> Heading/Wall Following -> Wheel Speed -> PWM.

5. **Motion Layer (`motion/`)**
   - Generates velocity profiles (trapezoidal/s-curve).
   - Translates high-level moves (e.g., "move one cell", "turn 90 left") into smooth trajectories.

6. **Maze Layer (`maze/`)**
   - Pure algorithmic logic.
   - Flood-fill, Dijkstra, path smoothing.
   - 100% hardware-agnostic (written in C).

7. **Robot / Application Layer (`robot/`)**
   - High-level State Machine (Boot, Idle, Search, Fast Run).
   - Executes missions and coordinates between moving and thinking.

8. **Display & Utils (`display/`, `utils/`)**
   - User interface (OLED menu) and common utilities (math, logging).

## Core Concepts

- **1kHz Control Loop**: All critical control (PID, sensor fusion, odometry) runs inside a hardware timer interrupt exactly 1000 times per second.
- **Main Loop**: Handles non-time-critical tasks like pathfinding, menu updates, OLED drawing, and low-priority sensor reading (I2C distance sensors).
