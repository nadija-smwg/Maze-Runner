# 🏁 Micromouse Project — Final Comprehensive Report

> **Date:** 2026-08-24  
> **Project:** Maze-Runner Micromouse  
> **Platform:** STM32F401CCU6 (Black Pill) + TB6612FNG + N20 Motors + MPU6050 + 5×VL53L0X + SSD1306 OLED

---

## 📋 Executive Summary

The Micromouse project has successfully completed hardware bring-up (Phases 1-3) and partially implemented sensor fusion (Phase 4) and motion control (Phase 5). **All 5 ToF sensors, both motors with encoders, the IMU, and the OLED display are fully operational.** The linear distance measurement (X in mm) from encoders is accurate and calibrated.

However, the **heading/angle estimation is broken** in the Full_Code due to 5 interconnected bugs in the sensor fusion pipeline. These have been identified and documented with specific fixes. The motion controller (Phase 5) remains a skeleton of TODO stubs — no motorized autonomous movement has been implemented yet.

A revised **bottom-up, test-driven strategy** has been proposed to complete the remaining work in 5 isolated, testable stages.

---

## 📊 Phase-by-Phase Status

### Phase 1: Hardware Abstraction & Basic I/O ✅ COMPLETE

| Component | File | Status | Notes |
|-----------|------|--------|-------|
| GPIO | [gpio.cpp](./Full_Code/Micromouse/src/hardware/gpio.cpp) | ✅ Working | Motor direction pins, STBY |
| Buttons | [button.cpp](./Full_Code/Micromouse/src/hardware/button.cpp) | ✅ Working | BTN_START (PB5), BTN_MODE (PB4) with debounce |
| LEDs | [led.cpp](./Full_Code/Micromouse/src/hardware/led.cpp) | ✅ Working | Status (PA5), Debug (PC13) with non-blocking blink |
| Battery | [battery.cpp](./Full_Code/Micromouse/src/hardware/battery.cpp) | ✅ Working | ADC on PB0, voltage divider ratio 2.0 |
| Timer | [timer.cpp](./Full_Code/Micromouse/src/hardware/timer.cpp) | ✅ Working | 1kHz hardware timer interrupt |
| OLED | [oled_driver.cpp](./Full_Code/Micromouse/src/display/oled_driver.cpp) | ✅ Working | SSD1306 128×64 on I2C (0x3C) |

**Testing Evidence:** Phase 1 test mode successfully displays battery voltage, timer ticks, and responds to button presses with LED toggles.

---

### Phase 2: Actuation & Low-Level Kinematics ✅ COMPLETE

| Component | File | Status | Notes |
|-----------|------|--------|-------|
| PWM | [pwm.cpp](./Full_Code/Micromouse/src/hardware/pwm.cpp) | ✅ Working | TIM1 @ 20kHz, ARR=4199 |
| Motors | [motor.cpp](./Full_Code/Micromouse/src/hardware/motor.cpp) | ✅ Working | Forward, reverse, turn L/R, stop |
| Encoders | [encoder.cpp](./Full_Code/Micromouse/src/hardware/encoder.cpp) | ✅ Working | TIM2 (32-bit) + TIM3 (16-bit), 4× quadrature |
| PID Class | [pid.cpp](./Full_Code/Micromouse/src/control/pid.cpp) | ✅ Working | Generic PID with anti-windup |

**Key Calibration:**
- `GEAR_RATIO = 18.85f` — tuned so 100cm physical = 100cm on OLED
- `WHEEL_DIAMETER_MM = 43.0f`
- `ENCODER_CPR = 527.8` (7 PPR × 18.85 gear × 4 quadrature)
- `MM_PER_COUNT = 0.2559` (135.09mm circumference / 527.8 CPR)

**Encoder Wiring Note:** Physical left/right encoders are swapped vs. hardware timers:
- Physical LEFT encoder → TIM3 (16-bit) with negation
- Physical RIGHT encoder → TIM2 (32-bit) without negation

**Testing Evidence:** Phase 2 test mode cycles through motor states (stop→fwd→rev→turnL→turnR→stop) and displays encoder counts and speed in mm/s.

**Testing Code Reference:** [Motors_Motion_Control_With_PID.ino](./Testing%20Codes/Motors_Motion_Control_With_PID/Motors_Motion_Control_With_PID.ino) — PID speed control proven working with gains `kp=1.8, ki=0.8, kd=0.02`.

---

### Phase 3: Sensing & I2C Devices ✅ COMPLETE

| Component | File | Status | Notes |
|-----------|------|--------|-------|
| MPU6050 | [mpu6050.cpp](./Full_Code/Micromouse/src/sensors/mpu6050.cpp) | ✅ Working | ±500°/s, DLPF=44Hz, 1000 sample calibration |
| VL53L0X (×5) | [vl53l0x.cpp](./Full_Code/Micromouse/src/sensors/vl53l0x.cpp) | ✅ Working | XSHUT multiplexing, addrs 0x30-0x34 |
| Distance Mgr | [distance_manager.cpp](./Full_Code/Micromouse/src/sensors/distance_manager.cpp) | ✅ Working | EMA filter (α=0.7), wall detection thresholds |
| Sensor Mgr | [sensor_manager.cpp](./Full_Code/Micromouse/src/sensors/sensor_manager.cpp) | ✅ Working | Orchestrates init of all I2C sensors |
| Calibration | [calibration.cpp](./Full_Code/Micromouse/src/sensors/calibration.cpp) | ✅ Working | Calls mpu6050_calibrate_gyro(1000) |

**MPU6050 Implementation Details:**
- Low-pass filter: EMA with α=0.85 on gyro Z axis (`gz_filtered = 0.85 * gz_filtered + 0.15 * raw_gz`)
- This matches the filtering from the working test code [1.MPU6050.ino](./Testing%20Codes/1.MPU6050/1.MPU6050.ino)
- Calibration averages 1000 samples with 2ms spacing (after discarding first 50 samples)

**ToF Sensor Layout:**
```
        FL(PB1)    F(PA4)    FR(PC14)
          ╲         │         ╱
           ╲        │        ╱
    L(PA15)──── [ROBOT] ────R(PB3)
```

**Testing Evidence:** Phase 3 test mode displays all 5 ToF distances, gyro Z rate, gyro bias, and raw gyro values. All sensors respond correctly.

---

### Phase 4: Sensor Fusion & Localization ⚠️ PARTIAL

| Component | File | Status | Notes |
|-----------|------|--------|-------|
| Odometry | [odometry.cpp](./Full_Code/Micromouse/src/localization/odometry.cpp) | ✅ Working | X/Y/θ from encoder deltas |
| Heading Estimator | [heading_estimator.cpp](./Full_Code/Micromouse/src/localization/heading_estimator.cpp) | ❌ **Broken** | alpha=1.0 (gyro only), encoder dtheta=0 |
| Sensor Fusion | [sensor_fusion.cpp](./Full_Code/Micromouse/src/sensors/sensor_fusion.cpp) | ❌ **Broken** | Passes 0.0 for encoder dtheta |
| Position Estimator | [position_estimator.cpp](./Full_Code/Micromouse/src/localization/position_estimator.cpp) | ⚠️ Partial | Wall corrections disabled for testing |
| Pose | [pose.cpp](./Full_Code/Micromouse/src/localization/pose.cpp) | ✅ Working | Angle normalization [-180,180] and [-π,π] |

#### 🔴 ROOT CAUSE: Why Angle Is Wrong

**5 interconnected bugs** prevent correct heading estimation:

| # | Bug | Location | Impact |
|---|-----|----------|--------|
| 1 | `alpha = 1.0f` | heading_estimator.cpp:35 | 100% gyro, 0% encoder correction |
| 2 | `encoder_dtheta = 0.0f` | sensor_fusion.cpp:33 | Encoder rotation never reaches heading estimator |
| 3 | Unthrottled fusion loop | Micromouse.ino:540-548 | dt≈0.00001s destroys 0.85 EMA filter |
| 4 | Wrong WHEEL_BASE_MM | robot_config.h:76 | Calibrated against gyro-only heading (circular) |
| 5 | GEAR_RATIO mismatch | robot_config.h:26 | May affect turn calculations (distance is fine) |

**Why X-distance works but angle doesn't:**
- X/Y position uses encoder counts × MM_PER_COUNT directly — no fusion involved
- Heading uses the complementary filter which has all 5 bugs above

**Why Phase 5 gyro hand-test works:**
- Phase 5 throttles fusion to 100Hz (10ms intervals) — fixes bug #3
- Phase 5 disabled ToF sensor reads — prevents I2C blocking from corrupting dt
- Hand-rotation only tests gyro, not encoder-gyro fusion

---

### Phase 5: High-Level Motion & Control ⚠️ SKELETON

| Component | File | Status | Notes |
|-----------|------|--------|-------|
| Motion Controller | [motion_controller.cpp](./Full_Code/Micromouse/src/control/motion_controller.cpp) | ❌ TODO stubs | Master 1kHz loop — empty |
| Speed Controller | [speed_controller.cpp](./Full_Code/Micromouse/src/control/speed_controller.cpp) | ❌ TODO stubs | Per-wheel PID — empty |
| Heading Controller | [heading_controller.cpp](./Full_Code/Micromouse/src/control/heading_controller.cpp) | ❌ TODO stubs | Heading PID — empty |
| Velocity Controller | [velocity_controller.cpp](./Full_Code/Micromouse/src/control/velocity_controller.cpp) | ❌ TODO stubs | Diff drive mixer — empty |
| Cell Controller | [cell_controller.cpp](./Full_Code/Micromouse/src/control/cell_controller.cpp) | ❌ TODO stubs | Distance tracking — empty |
| Turn Controller | [turn_controller.cpp](./Full_Code/Micromouse/src/control/turn_controller.cpp) | ❌ TODO stubs | In-place turn — empty |
| Wall Follower | [wall_follower.cpp](./Full_Code/Micromouse/src/control/wall_follower.cpp) | ❌ TODO stubs | Centering PD — empty |
| Trajectory Controller | [trajectory_controller.cpp](./Full_Code/Micromouse/src/control/trajectory_controller.cpp) | ❌ TODO stubs | S-curve profiler — empty |

**Current Phase 5 Test:** Only a gyro hand-test that displays heading as you turn the robot by hand. No motors are activated.

---

### Phase 6: Maze Solving & State Machine ❌ NOT STARTED

| Component | File | Status |
|-----------|------|--------|
| Maze | maze/maze.cpp | ❌ Not started |
| Flood Fill | maze/flood_fill.cpp | ❌ Not started |
| Dijkstra | maze/dijkstra_weighted.c | ❌ Not started |
| Path Smoother | maze/path_smoother.c | ❌ Not started |
| State Machine | robot/robot_state_machine.cpp | ❌ Not started |
| Mission Manager | robot/mission_manager.cpp | ❌ Not started |

---

## 📁 Complete File Audit

### Full_Code/Micromouse/src/config/
| File | Lines | Status | Purpose |
|------|-------|--------|---------|
| [config.h](./Full_Code/Micromouse/src/config/config.h) | 147 | ✅ Complete | Maze geometry, costs, motion profile params |
| [pin_config.h](./Full_Code/Micromouse/src/config/pin_config.h) | 161 | ✅ Complete | All GPIO pin definitions |
| [robot_config.h](./Full_Code/Micromouse/src/config/robot_config.h) | 150 | ⚠️ WHEEL_BASE needs fix | Physical constants |

### Full_Code/Micromouse/src/hardware/
| File | Lines | Status | Purpose |
|------|-------|--------|---------|
| [motor.cpp](./Full_Code/Micromouse/src/hardware/motor.cpp) | 108 | ✅ Complete | TB6612FNG driver |
| [motor.h](./Full_Code/Micromouse/src/hardware/motor.h) | 156 | ✅ Complete | Motor API |
| [encoder.cpp](./Full_Code/Micromouse/src/hardware/encoder.cpp) | 187 | ✅ Complete | Hardware encoder mode |
| [encoder.h](./Full_Code/Micromouse/src/hardware/encoder.h) | 138 | ✅ Complete | Encoder API |
| [pwm.cpp](./Full_Code/Micromouse/src/hardware/pwm.cpp) | ~60 | ✅ Complete | TIM1 PWM setup |
| [gpio.cpp](./Full_Code/Micromouse/src/hardware/gpio.cpp) | ~35 | ✅ Complete | Direction pins |
| [battery.cpp](./Full_Code/Micromouse/src/hardware/battery.cpp) | ~75 | ✅ Complete | ADC voltage monitoring |
| [button.cpp](./Full_Code/Micromouse/src/hardware/button.cpp) | ~60 | ✅ Complete | Debounced buttons |
| [led.cpp](./Full_Code/Micromouse/src/hardware/led.cpp) | ~60 | ✅ Complete | Non-blocking blink |
| [timer.cpp](./Full_Code/Micromouse/src/hardware/timer.cpp) | ~75 | ✅ Complete | 1kHz ISR |

### Full_Code/Micromouse/src/sensors/
| File | Lines | Status | Purpose |
|------|-------|--------|---------|
| [mpu6050.cpp](./Full_Code/Micromouse/src/sensors/mpu6050.cpp) | 179 | ✅ Complete | IMU driver with calibration |
| [vl53l0x.cpp](./Full_Code/Micromouse/src/sensors/vl53l0x.cpp) | ~65 | ✅ Complete | ToF sensor driver |
| [distance_manager.cpp](./Full_Code/Micromouse/src/sensors/distance_manager.cpp) | 113 | ✅ Complete | Wall detection + centering error |
| [sensor_manager.cpp](./Full_Code/Micromouse/src/sensors/sensor_manager.cpp) | ~40 | ✅ Complete | Init orchestrator |
| [sensor_fusion.cpp](./Full_Code/Micromouse/src/sensors/sensor_fusion.cpp) | 60 | ❌ **Broken** | dtheta=0, no proper fusion |
| [calibration.cpp](./Full_Code/Micromouse/src/sensors/calibration.cpp) | ~25 | ✅ Complete | Gyro calibration wrapper |

### Full_Code/Micromouse/src/localization/
| File | Lines | Status | Purpose |
|------|-------|--------|---------|
| [odometry.cpp](./Full_Code/Micromouse/src/localization/odometry.cpp) | 55 | ✅ Complete | Encoder dead reckoning |
| [heading_estimator.cpp](./Full_Code/Micromouse/src/localization/heading_estimator.cpp) | 46 | ❌ **Broken** | alpha=1.0, deadband blocks |
| [position_estimator.cpp](./Full_Code/Micromouse/src/localization/position_estimator.cpp) | 63 | ⚠️ Partial | Wall corrections disabled |
| [pose.cpp](./Full_Code/Micromouse/src/localization/pose.cpp) | 25 | ✅ Complete | Angle normalization |
| [coordinate_transform.cpp](./Full_Code/Micromouse/src/localization/coordinate_transform.cpp) | ~15 | ✅ Complete | mm ↔ cell conversion |

### Full_Code/Micromouse/src/control/
| File | Lines | Status | Purpose |
|------|-------|--------|---------|
| [pid.cpp](./Full_Code/Micromouse/src/control/pid.cpp) | 52 | ✅ Complete | Generic PID controller |
| [motion_controller.cpp](./Full_Code/Micromouse/src/control/motion_controller.cpp) | 53 | ❌ TODO | Master control loop |
| [speed_controller.cpp](./Full_Code/Micromouse/src/control/speed_controller.cpp) | 34 | ❌ TODO | Per-wheel speed PID |
| [heading_controller.cpp](./Full_Code/Micromouse/src/control/heading_controller.cpp) | 32 | ❌ TODO | Heading PID |
| [velocity_controller.cpp](./Full_Code/Micromouse/src/control/velocity_controller.cpp) | 25 | ❌ TODO | Diff drive mixer |
| [cell_controller.cpp](./Full_Code/Micromouse/src/control/cell_controller.cpp) | 37 | ❌ TODO | Distance tracking |
| [turn_controller.cpp](./Full_Code/Micromouse/src/control/turn_controller.cpp) | 37 | ❌ TODO | In-place turns |
| [wall_follower.cpp](./Full_Code/Micromouse/src/control/wall_follower.cpp) | 32 | ❌ TODO | Wall centering PD |
| [trajectory_controller.cpp](./Full_Code/Micromouse/src/control/trajectory_controller.cpp) | ~15 | ❌ TODO | S-curve profiler |

### Full_Code/Micromouse/src/motion/
| File | Lines | Status | Purpose |
|------|-------|--------|---------|
| [motion_profile.c](./Full_Code/Micromouse/src/motion/motion_profile.c) | ~320 | ⚠️ Untested | S-curve/trapezoidal profiles |
| [straight_motion.cpp](./Full_Code/Micromouse/src/motion/straight_motion.cpp) | ~30 | ❌ TODO | Straight line controller |
| [arc_motion.cpp](./Full_Code/Micromouse/src/motion/arc_motion.cpp) | ~15 | ❌ TODO | Rolling turn controller |
| [rolling_turn.cpp](./Full_Code/Micromouse/src/motion/rolling_turn.cpp) | ~25 | ❌ TODO | Smooth turn |
| [s_curve.cpp](./Full_Code/Micromouse/src/motion/s_curve.cpp) | ~10 | ❌ TODO | S-curve wrapper |
| [look_ahead.cpp](./Full_Code/Micromouse/src/motion/look_ahead.cpp) | ~20 | ❌ TODO | Look-ahead planner |

### Testing Codes/ (Reference Implementations)
| File | Lines | Status | Purpose |
|------|-------|--------|---------|
| [1.MPU6050.ino](./Testing%20Codes/1.MPU6050/1.MPU6050.ino) | 287 | ✅ Working | Gyro calibration + yaw tracking. Uses α=0.85 EMA, α=0.98 comp. filter. **Reference for heading.** |
| [Motors_Motion_Control.ino](./Testing%20Codes/Motors_Motion_Control/Motors_Motion_Control.ino) | 217 | ✅ Working | Raw encoder deltas + speed. Uses MM_PER_COUNT=0.05869. |
| [Motors_Motion_Control_With_PID.ino](./Testing%20Codes/Motors_Motion_Control_With_PID/Motors_Motion_Control_With_PID.ino) | 270 | ✅ Working | **PID speed control: kp=1.8, ki=0.8, kd=0.02.** Accumulative PWM pattern. |
| 4.Motors_Combine_With_Motors.ino | ~280 | ✅ Working | Combined motor + encoder test |
| VL53L0X/Codefor5TOF.ino | ~200 | ✅ Working | 5-sensor XSHUT multiplexing |

---

## 🔧 Testing Code vs Full_Code Analysis

### Key Differences Discovered

| Aspect | Testing Code | Full_Code | Issue? |
|--------|-------------|-----------|--------|
| MM_PER_COUNT | 0.05869 (hardcoded) | 0.2559 (derived from GEAR_RATIO=18.85) | ⚠️ Different values! |
| GEAR_RATIO | Implicit: CPR=600 | 18.85 (tuned) | Different CPR assumptions |
| Gyro filter α | 0.85 | 0.85 | ✅ Same |
| Comp. filter α | 0.98 | 1.0 (broken) | ❌ Must fix to 0.98 |
| PID pattern | Accumulative (pwm += output) | Standard (output = P+I+D) | ⚠️ Adaptation needed |
| Fusion rate | 100Hz (10ms delay) | Unthrottled in P4, 100Hz in P5 | ❌ P4 broken |
| Encoder direction | Manual negation per motor | Negation in encoder_get_count | ✅ Same logical result |

### MM_PER_COUNT Discrepancy Explained
- **Testing Code:** `MM_PER_COUNT = (π × 34.0) / 600 = 0.1780` ... wait, but code says `0.05869`
  - `0.05869 = (π × 34.0) / (7 × 65 × 4)` where `7×65×4 = 1820`
  - So testing code was using the OLD gear ratio of 65:1
- **Full_Code:** `MM_PER_COUNT = (π × 43.0) / (7 × 18.85 × 4) = 135.09 / 527.8 = 0.2559`
  - Uses recalibrated gear ratio 18.85 and wheel diameter 43mm
- **Conclusion:** The Full_Code values are the calibrated ones (100cm = 100cm). The testing code used different motor specs. **Full_Code values are correct for current hardware.**

---

## 🆕 Revised Implementation Strategy

### Previous Strategy (Failed)
Tried to implement Phases 4-5 simultaneously: sensor fusion + motion profiles + PID + wall following. Multiple interacting systems made it impossible to isolate bugs. The heading was wrong but we couldn't tell if it was the gyro, the filter, the encoders, the wheelbase, or the loop rate.

### New Strategy: Bottom-Up Test-Driven

```
Stage E: Wall Following & Maze Solving
    │ (depends on)
Stage D: Cell-By-Cell Navigation  
    │ (depends on)
Stage C: ToF Sensor Turn Decisions
    │ (depends on)
Stage B: 90° In-Place Turn (IMU + Encoder)
    │ (depends on)
Stage A: Drive Straight 180mm (Encoder + IMU heading)
    │ (depends on)
Phase 4 Fixes: Fix heading estimator + fusion rate + wheelbase
    │ (depends on)
Phases 1-3: ✅ ALREADY WORKING
```

**Each stage is a self-contained test. You flash it, press a button, and the robot does ONE thing. If it fails, you know exactly which capability is broken.**

See the detailed implementation plans:
- [implementation_plan.md](./implementation_details/implementation_plan.md) — Master plan with all stages
- [phase4_sensor_fusion.md](./implementation_details/phase4_sensor_fusion.md) — Phase 4 fixes
- [phase5_high_level_motion.md](./implementation_details/phase5_high_level_motion.md) — Stage A/B/C details

---

## 📐 Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    Micromouse.ino (Main Loop)                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐        │
│  │ Buttons  │  │  OLED    │  │   ToF    │  │  State   │        │
│  │  20Hz    │  │  10Hz    │  │  20Hz    │  │ Machine  │        │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘        │
└─────────────────────────┬───────────────────────────────────────┘
                          │ (main loop, ~50Hz)
══════════════════════════╪═══════════════════════════════════════
                          │ (1kHz timer ISR boundary)
┌─────────────────────────┴───────────────────────────────────────┐
│                 motion_controller_update() @ 1kHz                │
│  ┌───────────────┐    ┌──────────────────┐                      │
│  │ Sensor Fusion │    │ Cell/Turn Ctrl   │                      │
│  │  IMU + Enc    │───▶│ Target v, θ      │                      │
│  └───────────────┘    └────────┬─────────┘                      │
│                                │                                 │
│  ┌───────────────┐    ┌────────▼─────────┐                      │
│  │   Heading     │◀───│ Velocity Mixer   │                      │
│  │  Controller   │    │ v→(vL, vR)       │                      │
│  └───────────────┘    └────────┬─────────┘                      │
│                                │                                 │
│                       ┌────────▼─────────┐                      │
│                       │ Speed Controller │                      │
│                       │ PID L + PID R    │                      │
│                       └────────┬─────────┘                      │
│                                │                                 │
│                       ┌────────▼─────────┐                      │
│                       │   Motor Driver   │                      │
│                       │ PWM L + PWM R    │                      │
│                       └──────────────────┘                      │
└─────────────────────────────────────────────────────────────────┘
```

---

## ⚡ Priority Action Items

| # | Action | Difficulty | Impact |
|---|--------|-----------|--------|
| 1 | Fix heading_estimator alpha → 0.98 | Easy (1 line) | Critical |
| 2 | Pass real encoder dtheta in sensor_fusion.cpp | Easy (5 lines) | Critical |
| 3 | Throttle Phase 4 fusion to 100Hz | Easy (5 lines) | Critical |
| 4 | Re-measure actual WHEEL_BASE_MM | Physical measurement | Critical |
| 5 | Implement speed_controller (port PID from test code) | Medium (~50 lines) | High |
| 6 | Implement heading_controller | Medium (~30 lines) | High |
| 7 | Implement velocity_controller (mixer) | Easy (~10 lines) | High |
| 8 | Implement cell_controller (distance tracking) | Medium (~60 lines) | High |
| 9 | Implement motion_controller (wire up loop) | Medium (~40 lines) | High |
| 10 | Implement turn_controller | Medium (~50 lines) | High |
| 11 | Create Stage A test in Micromouse.ino | Medium (~80 lines) | High |
| 12 | Create Stage B test in Micromouse.ino | Medium (~60 lines) | High |
| 13 | Create Stage C test in Micromouse.ino | Medium (~80 lines) | Medium |
| 14 | Implement wall_follower | Medium (~40 lines) | Medium |
| 15 | Implement maze solving (Phase 6) | Hard (~300 lines) | Low (for now) |

---

## 📝 Recommendations

1. **Fix items 1-4 first** — these are the Phase 4 heading bugs. Each is a 1-5 line change. After fixing them, the hand-rotation test should show correct angles from BOTH gyro and encoders.

2. **Then implement Stage A (items 5-9, 11)** — this is the first motorized test. Keep speeds very low (150-200 mm/s) for safety. The robot should drive 180mm in a straight line and stop.

3. **Then Stage B (items 10, 12)** — 90° turns. This validates the entire sensor fusion + motor control pipeline.

4. **Stage C-E can wait** — they depend on A+B working correctly.

5. **Physical measurement of WHEEL_BASE_MM is critical** — no amount of software tuning will fix a wrong wheelbase constant. Use calipers if available.
