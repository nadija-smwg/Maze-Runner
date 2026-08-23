# 🧠 Master Micromouse Implementation Plan (Updated)

> **Objective:** Port raw hardware logic from `Testing Codes/` and pathfinding algorithms from `src/` into the modular C++ architecture of `Full Code/Micromouse/`. When you say **"complete this phase"**, I will write all the production code for every file listed in that phase.
>
> All robot dimensions use: `ENCODER_CPR = 1820`, `WHEELBASE_MM = 75.0`, `MM_PER_COUNT = 0.05869`

---

## Current Status Summary

| Phase | Name | Status | Files Complete | Files Remaining |
|-------|------|--------|----------------|-----------------|
| **1** | Hardware Abstraction & Basic I/O | ✅ **COMPLETE** | 8/8 (.h + .cpp) | 0 |
| **2** | Actuation & Low-Level Kinematics | ✅ **COMPLETE** | 10/10 (.h + .cpp) | 0 |
| **3** | Sensing & I2C Devices | 🔴 **STUB** | 0/10 implementations | 10 |
| **4** | Sensor Fusion & Localization | 🔴 **STUB** | 0/10 implementations | 10 |
| **5** | High-Level Motion & Wall Following | 🔴 **STUB** | 0/16 implementations | 16 |
| **6** | Maze Solving & State Machine | 🔴 **STUB** | 0/12 implementations | 12 |

---

## Phase 1: Hardware Abstraction & Basic I/O ✅ COMPLETE

All files have full, working implementations. Phase 1 test mode is functional in `Micromouse.ino`.
*Note: All stale TODOs have been removed from the header files and replaced with actual implementation details.*

| File | Status | Details |
|------|--------|---------|
| [gpio.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/hardware/gpio.h) | ✅ Done | Motor pin abstraction API |
| [gpio.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/hardware/gpio.cpp) | ✅ Done | `pinMode()` + `digitalWrite()` for AIN/BIN/STBY |
| [led.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/hardware/led.h) | ✅ Done | Status + debug LED API |
| [led.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/hardware/led.cpp) | ✅ Done | Non-blocking blink via `millis()` |
| [button.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/hardware/button.h) | ✅ Done | Debounced button API (edge detection) |
| [button.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/hardware/button.cpp) | ✅ Done | 20ms debounce, `button_just_pressed()` |
| [battery.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/hardware/battery.h) | ✅ Done | ADC battery monitoring API |
| [battery.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/hardware/battery.cpp) | ✅ Done | 12-bit ADC, EMA filter, voltage divider math |
| [timer.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/hardware/timer.h) | ✅ Done | 1kHz timer API with callback |
| [timer.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/hardware/timer.cpp) | ✅ Done | TIM4 HardwareTimer, 1kHz ISR |

---

## Phase 2: Actuation & Low-Level Kinematics ✅ COMPLETE

All files have full, working implementations. Phase 2 test mode is functional in `Micromouse.ino`.
*Note: All stale TODOs have been removed from the header files and replaced with actual implementation details.*

| File | Status | Details |
|------|--------|---------|
| [pwm.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/hardware/pwm.h) | ✅ Done | TIM1 PWM API |
| [pwm.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/hardware/pwm.cpp) | ✅ Done | Register-level TIM1, PA8/PA9 AF1, 20kHz, CCR1/CCR2 |
| [motor.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/hardware/motor.h) | ✅ Done | TB6612FNG motor control API |
| [motor.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/hardware/motor.cpp) | ✅ Done | Direction control + signed speed → PWM + direction |
| [encoder.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/hardware/encoder.h) | ✅ Done | Quadrature encoder API |
| [encoder.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/hardware/encoder.cpp) | ✅ Done | TIM2(32-bit)/TIM3(16-bit) Encoder Mode 3, 4x decoding, input filters |
| [pid.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/pid.h) | ✅ Done | PID class with anti-windup |
| [pid.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/pid.cpp) | ✅ Done | P+I+D, derivative-on-measurement, output clamping |
| [math_utils.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/utils/math_utils.h) | ✅ Done | Constrain + map helpers |
| [math_utils.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/utils/math_utils.cpp) | ✅ Done | Implementation |

---

## Phase 3: Sensing & I2C Devices 🔴 STUB — Ready for Implementation

> **Source Code:** Port from [1.MPU6050.ino](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Testing%20Codes/1.MPU6050/1.MPU6050.ino) and [Codefor5TOF.ino](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Testing%20Codes/VL53L0X/Codefor5TOF.ino)

| File | Status | What To Do |
|------|--------|------------|
| [oled_driver.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/display/oled_driver.cpp) | 🔴 Stub | Create `Adafruit_SSD1306` instance (128×64, I2C 0x3C). Implement `oled_init()`, `oled_clear()`, `oled_update()`, `oled_print()`, `oled_draw_maze()` |
| [oled_driver.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/display/oled_driver.h) | 🟡 Header OK | No changes needed |
| [mpu6050.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/sensors/mpu6050.cpp) | 🔴 Stub | Port from `1.MPU6050.ino`: Wake-up (0x6B=0x00), DLPF (0x1A=0x03), Gyro ±500°/s (0x1B=0x08), Accel ±2g (0x1C=0x00). Calibrate 1000 samples. Read 14 bytes from 0x3B. Scale: gyro÷65.5, accel÷16384 |
| [mpu6050.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/sensors/mpu6050.h) | 🟡 Header OK | No changes needed |
| [vl53l0x.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/sensors/vl53l0x.cpp) | 🔴 Stub | Port XSHUT multiplexing from `Codefor5TOF.ino`: All LOW→delay→sequence HIGH one-by-one→`begin(addr)`. Addresses: 0x30–0x34. Configure timing budget for speed |
| [vl53l0x.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/sensors/vl53l0x.h) | 🟡 Header OK | No changes needed |
| [distance_manager.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/sensors/distance_manager.cpp) | 🔴 Stub | Populate sensor array, call `vl53l0x_init_all()`. Wall thresholds: front<120mm, sides<100mm. Centering error logic. EMA filter on readings |
| [distance_manager.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/sensors/distance_manager.h) | 🟡 Header OK | May need threshold constants added |
| [sensor_manager.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/sensors/sensor_manager.cpp) | 🔴 Stub | Wire up all init calls, update loop for ToF polling |
| [sensor_manager.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/sensors/sensor_manager.h) | 🟡 Header OK | No changes needed |
| [calibration.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/sensors/calibration.cpp) | 🔴 Stub | Call `mpu6050_calibrate_gyro(1000)`, encoder calibration routine |
| [calibration.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/sensors/calibration.h) | 🟡 Header OK | No changes needed |

### Phase 3 Testing Verification
- OLED displays "Calibrating IMU..." for 3 seconds, then live telemetry
- Wave hand in front of each ToF sensor → correct reading drops
- Rotate robot by hand → Gyro Z responds and returns to ~0

---

## Phase 4: Sensor Fusion & Localization 🔴 STUB — Ready for Implementation

> **Key Constants (already in `robot_config.h`):** `ENCODER_CPR=1820`, `WHEEL_BASE_MM=75.0`, `MM_PER_COUNT=0.0587`

| File | Status | What To Do |
|------|--------|------------|
| [odometry.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/localization/odometry.cpp) | 🔴 Stub | Uncomment TODO math: `d_center = (L+R)/2`, `d_theta = (R-L)/WHEEL_BASE_MM`, update X/Y/theta using cos/sin |
| [odometry.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/localization/odometry.h) | 🟡 Header OK | No changes needed |
| [pose.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/localization/pose.cpp) | 🔴 Stub | Implement angle normalization, pose distance/heading helpers |
| [pose.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/localization/pose.h) | 🟡 Header OK | Check Pose struct definition |
| [heading_estimator.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/localization/heading_estimator.cpp) | 🔴 Stub | Integrate gyro Z-axis over dt for heading estimate |
| [heading_estimator.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/localization/heading_estimator.h) | 🟡 Header OK | No changes needed |
| [position_estimator.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/localization/position_estimator.cpp) | 🔴 Stub | High-level pose manager combining odometry + heading |
| [position_estimator.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/localization/position_estimator.h) | 🟡 Header OK | No changes needed |
| [coordinate_transform.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/localization/coordinate_transform.cpp) | 🔴 Stub | Local↔global frame transforms, mm↔cell conversions |
| [coordinate_transform.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/localization/coordinate_transform.h) | 🟡 Header OK | No changes needed |
| [sensor_fusion.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/sensors/sensor_fusion.cpp) | 🔴 Stub | Complementary filter: `fused = 0.98*(fused + gyro_dθ) + 0.02*(odom_θ)`. Track fused heading + velocity |
| [sensor_fusion.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/sensors/sensor_fusion.h) | 🟡 Header OK | No changes needed |

### Phase 4 Testing Verification
- Push robot 100mm straight → `global_X ≈ 100.0`, `global_Y ≈ 0`
- Spin robot 90° → `fused_heading ≈ 1.57 rad`
- If off, tune `MM_PER_COUNT` and `WHEEL_BASE_MM`

---

## Phase 5: High-Level Motion & Wall Following 🔴 STUB — Ready for Implementation

> **Source Code:** Port from [motion_profile.c](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/src/motion_profile.c), [path_smoother.c](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/src/path_smoother.c), and [Motors_Motion_Control_With_PID.ino](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Testing%20Codes/Motors_Motion_Control_With_PID/Motors_Motion_Control_With_PID.ino)

| File | Status | What To Do |
|------|--------|------------|
| [motion_controller.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/motion_controller.cpp) | 🔴 Stub | Master 1kHz loop: read encoders → odometry → fusion → trajectory → heading → wall_follower → speed → PWM |
| [motion_controller.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/motion_controller.h) | 🟡 Header OK | No changes needed |
| [speed_controller.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/speed_controller.cpp) | 🔴 Stub | Left/Right PID (Kp=1.8, Ki=0.8, Kd=0.02 from testing), feedforward, PWM output. Port from `Motors_Motion_Control_With_PID.ino` |
| [speed_controller.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/speed_controller.h) | 🟡 Header OK | No changes needed |
| [heading_controller.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/heading_controller.cpp) | 🔴 Stub | PID on heading error (wrapped to ±180°), output angular velocity correction |
| [heading_controller.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/heading_controller.h) | 🟡 Header OK | No changes needed |
| [velocity_controller.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/velocity_controller.cpp) | 🔴 Stub | Convert (v, ω) → individual wheel speeds: `left = v - ω*W/2`, `right = v + ω*W/2` |
| [velocity_controller.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/velocity_controller.h) | 🟡 Header OK | No changes needed |
| [wall_follower.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/wall_follower.cpp) | 🔴 Stub | PD controller on lateral error from side ToF sensors. Both walls: `error = L-R`. One wall: `error = dist - target` |
| [wall_follower.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/wall_follower.h) | 🟡 Header OK | No changes needed |
| [trajectory_controller.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/trajectory_controller.cpp) | 🔴 Stub | Manage active motion command (straight/turn), delegate to motion modules |
| [cell_controller.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/cell_controller.cpp) | 🔴 Stub | Cell-level positioning: stop at cell center, align to cell grid |
| [turn_controller.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/turn_controller.cpp) | 🔴 Stub | In-place turns (90°/180°) using heading PID |
| [straight_motion.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/motion/straight_motion.cpp) | 🔴 Stub | S-curve/trapezoidal velocity profile for straight segments. Port from `motion_profile.c` |
| [rolling_turn.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/motion/rolling_turn.cpp) | 🔴 Stub | Arc-based rolling turns with constant radius. Port from `motion_profile.c` arc functions |
| [s_curve.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/motion/s_curve.cpp) | 🔴 Stub | Jerk-limited acceleration ramp generation |
| [arc_motion.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/motion/arc_motion.cpp) | 🔴 Stub | Arc trajectory math (radius, angular velocity, arc length) |
| [look_ahead.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/motion/look_ahead.cpp) | 🔴 Stub | Multi-command look-ahead for smooth deceleration planning |

### Phase 5 Testing Verification
- `DRIVE_STRAIGHT(500mm)` → robot ramps up, cruises, ramps down, stops at exactly 500mm
- Place robot off-center in corridor → actively steers back to center

---

## Phase 6: Maze Solving & State Machine 🔴 STUB — Ready for Implementation

> **Source Code:** Port from [solver.c](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/src/solver.c), [flood_fill.c](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/src/flood_fill.c), [dijkstra_weighted.c](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/src/dijkstra_weighted.c), [path_smoother.c](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/src/path_smoother.c)
>
> These are pure C files already in `Full Code/Micromouse/src/maze/`. They need `extern "C"` wrapping and integration with the firmware.

| File | Status | What To Do |
|------|--------|------------|
| [maze.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/maze/maze.h) | ✅ Done | Already ported — maze types, wall encoding, MotionCommand struct |
| [flood_fill.c](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/maze/flood_fill.c) | ✅ Done | Already ported — multi-source BFS algorithm |
| [flood_fill.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/maze/flood_fill.h) | ✅ Done | API header |
| [dijkstra_weighted.c](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/maze/dijkstra_weighted.c) | ✅ Done | Already ported — 1024-state 3D Dijkstra |
| [dijkstra_weighted.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/maze/dijkstra_weighted.h) | ✅ Done | API header |
| [solver.c](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/maze/solver.c) | ✅ Done | Already ported — search + fast path solver |
| [solver.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/maze/solver.h) | ✅ Done | API header |
| [path_smoother.c](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/maze/path_smoother.c) | ✅ Done | Already ported — direction→command compression |
| [path_smoother.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/maze/path_smoother.h) | ✅ Done | API header |
| [maze_explorer.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/maze/maze_explorer.cpp) | 🔴 Stub | Decision logic: when to return, when exploration is complete, unvisited cell tracking |
| [robot_state_machine.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/robot/robot_state_machine.cpp) | 🔴 Stub | Full FSM: IDLE→CALIBRATING→SEARCH_RUN→RETURN_START→FAST_RUN. Wire `solver_record_walls()`, `solver_search_step()`, `solver_compute_fast_path()` |
| [robot_state_machine.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/robot/robot_state_machine.h) | 🟡 Header OK | States already defined |
| [mission_manager.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/robot/mission_manager.cpp) | 🔴 Stub | Multi-run strategy, search→optimize→speed run sequencing |
| [mission_manager.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/robot/mission_manager.h) | 🟡 Header OK | No changes needed |
| [command_executor.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/robot/command_executor.cpp) | 🔴 Stub | Execute MotionCommand queue: feed commands to motion_controller, wait for completion |
| [command_executor.h](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/robot/command_executor.h) | 🟡 Header OK | No changes needed |
| [competition_state.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/robot/competition_state.cpp) | 🔴 Stub | Competition timer, run counter, error recovery |
| [search_mode.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/robot/search_mode.cpp) | 🔴 Stub | Search run loop: stop→read walls→flood fill→move |
| [fast_run_mode.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/robot/fast_run_mode.cpp) | 🔴 Stub | Speed run: execute pre-computed command sequence at max speed |
| [Micromouse.ino](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/Micromouse.ino) | 🟡 Partial | Update `setup()` to wire all Phase 3–6 inits. Update `loop()` for production flow |

### Remaining Support Files

| File | Status | What To Do |
|------|--------|------------|
| [display/debug_screen.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/display/debug_screen.cpp) | 🔴 Stub | Debug telemetry display (battery, heading, distances) |
| [display/menu.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/display/menu.cpp) | 🔴 Stub | OLED menu for mode selection |
| [display/status_screen.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/display/status_screen.cpp) | 🔴 Stub | Main status display |
| [utils/filters.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/utils/filters.cpp) | 🔴 Stub | EMA/complementary filter utilities |
| [utils/serial_debug.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/utils/serial_debug.cpp) | 🔴 Stub | Serial command parser for runtime debugging |
| [utils/timing.cpp](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/utils/timing.cpp) | 🔴 Stub | Timing profiler for loop execution measurement |

### Phase 6 Testing Verification
- Place robot in maze → Press BTN_START → drives cell-by-cell, mapping walls
- Reaches center → Return to start
- Press BTN_MODE → Sprint to center using optimized path, rolling turns

---

## Execution Workflow

When you say **"complete Phase X"**, I will:

1. **Read all relevant source files** (Testing Codes + `src/` algorithms)
2. **Write complete production-ready implementations** for every `.cpp` file in that phase
3. **Update headers** if signatures need to change
4. **Add Phase X test mode** to `Micromouse.ino` so you can flash and verify
5. **Document any hardware-specific notes** (pin conflicts, timing constraints)

> [!IMPORTANT]
> **Next step: Tell me which phase to complete.** Phase 3 (Sensing & I2C) is the natural next step since Phases 1 & 2 are done.

---

## Open Questions

> [!WARNING]
> **Hardware measurement TODO:** `WHEEL_BASE_MM = 75.0f` and `SENSOR_FRONT_OFFSET_MM = 30.0f` in `robot_config.h` are marked with TODO. Have you physically measured and verified these values? Incorrect values will cause the robot to crash.

> [!WARNING]
> **Voltage divider ratio:** `BATTERY_DIVIDER_RATIO = 2.0f` is marked TODO in `robot_config.h`. Please confirm your actual resistor values for accurate battery monitoring.

> [!NOTE]
> **Right encoder direction:** The PID testing code ([Motors_Motion_Control_With_PID.ino:L166](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Testing%20Codes/Motors_Motion_Control_With_PID/Motors_Motion_Control_With_PID.ino#L166)) negates the right encoder delta. This suggests the right motor is physically wired in reverse. This correction needs to be applied consistently in the speed controller during Phase 5.
