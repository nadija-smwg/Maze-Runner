# Micromouse Software Architecture

## Overview
The Micromouse software is designed with a layered architecture, abstracting hardware details from the high-level maze-solving logic. The system prioritizes deterministic control loops, modularity, and memory efficiency suitable for an STM32F401CCU6 (Black Pill) running Arduino Core.

**Current Status** (as of Phase 5.5): Cell drive + 90° turns + wall following operational.

## Layers

1. **Hardware Layer (`hardware/`)**
   - Direct STM32 register access for Encoders (TIM2=Right 32-bit, TIM3=Left 16-bit — **hardware wiring was swapped** from original design), PWM (TIM1 20kHz), and Timer ISR (TIM4 1kHz).
   - Motor: TB6612FNG via GPIO + PWM. Dead-zone compensation: Left=300, Right=250 PWM.
   - Encoder Mode 3 (4× quadrature), IC digital filter applied (code 3).

2. **Sensor Layer (`sensors/`)**
   - **VL53L0X × 5**: I2C address reassignment via XSHUT sequencing (one at a time). Continuous ranging at 33ms timing budget. Per-sensor 5-stage filter: validity → offset calibration → Median-3 → jump rejection → EMA (α=0.30).
   - Calibration offsets: FRONT=-28mm, LEFT=-14mm, RIGHT=-9mm. **FL/FR offsets: NOT YET SET (Bug B1)**.
   - **MPU6050**: 125Hz sample rate, ±500°/s range, DLPF ~44Hz. Calibration: 1000-sample average for bias. EMA: accel α=0.90, gyro α=0.80. Stationary drift correction at 0.002/update rate.
   - **`sensor_fusion.cpp`**: DEPRECATED — `fusion_update()` is never called in active test modes. Dead code. Will be removed in Phase 6 cleanup.

3. **Localization Layer (`localization/`)**
   - **Active heading source**: `heading_estimator.cpp` — complementary filter at 1kHz: 99.9% gyro + 0.1% encoder. Gyro deadband: 1.2 °/s. Empirical scale: GYRO_MULTIPLIER_LEFT = GYRO_MULTIPLIER_RIGHT = 0.60.
   - **Odometry**: Differential drive integration (d_center, d_theta), world-frame X/Y/θ. Reset to (0,0,0) at each cell start for distance tracking.

4. **Control Layer (`control/`)**
   - **Cascading architecture** (high-level to low):
     ```
     Straight motion profile (trapezoidal)
           ↓ target v (mm/s)
     Heading error × KP_HEADING=2.0        ──→ +w (rad/s)
     Wall centering error × KP_WALL=0.010  ──→ +w (rad/s) [PD with KD_WALL=0.005]
           ↓ total (v, w)
     Unicycle model: v_L = v - w×L/2,  v_R = v + w×L/2
           ↓ target wheel speeds (mm/s)
     Per-wheel PI+KFF: KFF=2.6, KP=5.0, KI=1.0
           ↓ PWM  (clamped to ±4199)
     Dead-zone compensation → TB6612FNG motors
     ```
   - **Turn controller**: In-place P-controller: w = -KP_TURN×err, KP_TURN=5.0, MAX=6.0 rad/s, MIN=1.5 rad/s. Deadband = 0.035 rad (≈2°).
   - **Wall follower**: PD on lateral centering error (mm), KP_WALL=0.010, KD_WALL=0.005, max correction = 0.5 rad/s.

5. **Motion Layer (`motion/`)**
   - Trapezoidal profile: accel=1500 mm/s², cruise=300 mm/s, decel=1500 mm/s² (search run).
   - Cell size = 180mm. Completion = odometry X ≥ 180mm AND speed ≤ end_speed+1mm/s.
   - S-curve and rolling turns: code exists, disabled (ENABLE_S_CURVE=0, ENABLE_DIAGONALS=0).

6. **Maze Layer (`maze/`)**
   - Pure C code — flood fill (`flood_fill.c`) + weighted Dijkstra (`dijkstra_weighted.c`) + path smoother (`path_smoother.c`).
   - **Phase 6 TODO**: Wire to `motion_execute_command()` and wall detection from distance_manager.

7. **Robot / Application Layer (`robot/`)**
   - High-level FSM (Boot, Idle, Search, Fast Run) — stub implementation.

8. **Display & Utils (`display/`, `utils/`)**
   - SSD1306 OLED 128×64, serial debug at 115200 baud.

## Core Timing

| Context | Rate | What runs |
|---------|------|-----------|
| 1kHz ISR | every 1ms | encoder_update_velocity, heading_estimator_update, straight_motion_update, velocity_controller_update, speed_controller, motor PWM |
| Main loop | ~200Hz | mpu6050_update_filter (I2C — cannot be in ISR), button_update, led_update |
| Main loop | ~100Hz | distance_manager_update (all 5 ToF sensors via I2C) |
| Main loop | ~20Hz | OLED update, Serial print, state machine logic |

**Key rule**: I2C reads (MPU6050, VL53L0X) must NEVER be called from the 1kHz ISR (I2C + ISR = deadlock). The ISR reads only cached filtered values.
