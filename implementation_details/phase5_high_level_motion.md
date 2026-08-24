# Phase 5: High-Level Motion & Control (REVISED)

> **Goal:** Make the robot actually MOVE autonomously using motors controlled by PID loops, guided by encoder distance and IMU heading. This phase is broken into three testable sub-stages.
>
> **Status:** ⚠️ PARTIALLY COMPLETE — Gyro hand-test works, no motorized movement implemented yet.

---

## Current State of Phase 5 Code

### What Exists (Working)
- `Micromouse.ino` Phase 5 block: Hand-rotation gyro test (no motors, just display heading)
- `control/pid.cpp`: Generic PID class with anti-windup — **fully implemented**
- `hardware/motor.cpp`: Motor control with direction and PWM — **fully implemented**
- `hardware/encoder.cpp`: Hardware encoder mode with delta tracking — **fully implemented**
- `sensors/sensor_fusion.cpp`: Fusion pipeline structure — **partially implemented**

### What Exists (Skeleton/TODO)
- `control/motion_controller.cpp`: All functions are empty TODO stubs
- `control/speed_controller.cpp`: Empty TODO stubs  
- `control/heading_controller.cpp`: Empty TODO stubs
- `control/velocity_controller.cpp`: Empty TODO stubs
- `control/cell_controller.cpp`: Empty TODO stubs
- `control/turn_controller.cpp`: Empty TODO stubs
- `control/wall_follower.cpp`: Empty TODO stubs

### What Works in Testing Codes (Reference)
- `Motors_Motion_Control_With_PID.ino`: PID speed control with `kp=1.8, ki=0.8, kd=0.02` — **PROVEN WORKING**
- `Motors_Motion_Control.ino`: Raw encoder delta and speed calculation — **PROVEN WORKING**

---

## Stage A: Drive Straight One Cell (180mm)

### Architecture Overview
```
                    ┌──────────────────────────────────┐
                    │        CELL CONTROLLER            │
                    │  Target: 180mm, Current: Xmm     │
                    │  State: ACCEL → CRUISE → DECEL   │
                    └──────────┬───────────────────────┘
                               │ target_linear_velocity (mm/s)
                               ▼
                    ┌──────────────────────────────────┐
                    │      HEADING CONTROLLER            │
                    │  Target: 0°, Current: θ°         │
                    │  PID → angular_correction (ω)    │
                    └──────────┬───────────────────────┘
                               │ (target_v, correction_ω)
                               ▼
                    ┌──────────────────────────────────┐
                    │     VELOCITY CONTROLLER            │
                    │  v_left  = v - (ω × W/2)         │
                    │  v_right = v + (ω × W/2)         │
                    └──────────┬───────────────────────┘
                               │ (target_left_speed, target_right_speed)
                               ▼
                    ┌──────────────────────────────────┐
                    │       SPEED CONTROLLER             │
                    │  Left PID:  target_L vs actual_L  │
                    │  Right PID: target_R vs actual_R  │
                    │  Output: PWM left, PWM right     │
                    └──────────┬───────────────────────┘
                               │ (pwm_left, pwm_right)
                               ▼
                    ┌──────────────────────────────────┐
                    │          MOTOR DRIVER              │
                    │  motor_set_both(pwm_L, pwm_R)    │
                    └──────────────────────────────────┘
```

### Implementation Details

#### 1. Cell Controller (`control/cell_controller.cpp`)
```
State Machine:
  IDLE → ACCELERATING → CRUISING → DECELERATING → COMPLETE

Parameters:
  - target_distance_mm = 180.0f  (1 cell)
  - max_speed_mm_s = 200.0f      (conservative for first test)
  - acceleration_mm_s2 = 500.0f  (gentle ramp)
  - deceleration_mm_s2 = 500.0f  (gentle stop)

Distance Tracking:
  - Use average of left and right encoder distances
  - distance_traveled = (encoder_left_mm + encoder_right_mm) / 2.0
  - remaining = target_distance - distance_traveled

Speed Profile (Simple Trapezoidal):
  - ACCELERATING: target_v increases by (accel × dt) each tick
  - CRUISING: target_v = max_speed
  - DECELERATING: When remaining < (current_speed² / (2 × decel)):
                  target_v decreases by (decel × dt) each tick
  - COMPLETE: target_v = 0, motors braked
```

#### 2. Heading Controller (`control/heading_controller.cpp`)
```
Purpose: Keep robot driving straight by correcting heading drift

Input:
  - target_heading: 0° (for straight driving)
  - current_heading: from fusion_get_heading()

PID Gains (starting values):
  - kp = 5.0    (degrees error → deg/s correction)
  - ki = 0.0    (no integral for heading — it causes oscillation)
  - kd = 0.5    (dampen angular oscillations)
  - Output limits: [-180, +180] deg/s angular velocity

Output:
  - angular_velocity_correction (ω) in deg/s
  - Convert to rad/s before passing to velocity controller
```

#### 3. Velocity Controller (`control/velocity_controller.cpp`)
```
Purpose: Differential drive mixer

Equations:
  v_left  = linear_v - (angular_ω_rad × WHEEL_BASE_MM / 2.0)
  v_right = linear_v + (angular_ω_rad × WHEEL_BASE_MM / 2.0)

These are TARGET wheel speeds in mm/s, passed to the speed controller.
```

#### 4. Speed Controller (`control/speed_controller.cpp`)
```
Purpose: PID control of individual wheel speeds

Based on: Motors_Motion_Control_With_PID.ino (PROVEN WORKING)

PID Gains (from testing code):
  - kp = 1.8
  - ki = 0.8
  - kd = 0.02
  - Output limits: [-PWM_MAX, +PWM_MAX] = [-4199, +4199]

The testing code uses an ACCUMULATIVE PID pattern:
  pwm += pid_output
  pwm = constrain(pwm, 0, PWM_MAX)

This must be adapted for the Full Code PID class which already handles
integral accumulation internally.

Speed Measurement:
  - Every 1ms tick: read encoder deltas
  - actual_left_speed = left_delta_counts × MM_PER_COUNT / dt
  - actual_right_speed = right_delta_counts × MM_PER_COUNT / dt
```

#### 5. Motion Controller — 1kHz ISR (`control/motion_controller.cpp`)
```
Called by: timer interrupt at exactly 1kHz (every 1ms)

void motion_controller_update(void) {
    float dt = 0.001f; // Fixed 1ms timestep

    // 1. Update sensors and localization
    fusion_update(dt);

    // 2. Get current state
    Pose pose = position_estimator_get_pose();
    float current_heading = heading_estimator_get();

    // 3. If idle, hold position
    if (motion_state == MOTION_IDLE) {
        motor_stop();
        return;
    }

    // 4. Get target velocity from active controller
    float target_v = 0.0f;
    if (motion_state == MOTION_STRAIGHT) {
        target_v = cell_controller_get_target_speed();
        if (cell_is_complete()) {
            motion_state = MOTION_IDLE;
            motor_stop();
            return;
        }
    }

    // 5. Heading correction
    float target_heading = stored_target_heading; // 0° for straight
    float omega = heading_controller_update(target_heading, current_heading, dt);

    // 6. Differential drive mixer
    float omega_rad = omega * (PI / 180.0f);
    float v_left  = target_v - (omega_rad * WHEEL_BASE_MM / 2.0f);
    float v_right = target_v + (omega_rad * WHEEL_BASE_MM / 2.0f);

    // 7. Speed PID
    speed_controller_update(v_left, v_right, actual_left_speed, actual_right_speed, dt);
}
```

### Phase 5A Test Sketch (in Micromouse.ino)
```
Setup:
  - Init all hardware (motors, encoders, IMU, OLED)
  - Calibrate gyro
  - Init fusion
  - Init all controllers
  - Wait for button press

Loop:
  - BTN_START pressed → start cell_move(1) — drive 180mm
  - BTN_MODE pressed → reset everything, stop motors
  - Display: distance, heading, speed on OLED
  - Serial: log at 100Hz for debugging
```

---

## Stage B: Turn Exactly 90° In-Place

### Architecture Overview
```
                    ┌──────────────────────────────────┐
                    │       TURN CONTROLLER              │
                    │  Target: ±90°, Current: θ°       │
                    │  State: TURNING → SETTLING → DONE │
                    └──────────┬───────────────────────┘
                               │ angular_velocity (ω) in deg/s
                               ▼
                    ┌──────────────────────────────────┐
                    │       SPEED CONTROLLER             │
                    │  For in-place turn:               │
                    │    left_speed = -(ω × W/2)       │
                    │    right_speed = +(ω × W/2)      │
                    └──────────┬───────────────────────┘
                               │ (pwm_left, pwm_right)
                               ▼
                    ┌──────────────────────────────────┐
                    │          MOTOR DRIVER              │
                    └──────────────────────────────────┘
```

### Implementation Details

#### Turn Controller (`control/turn_controller.cpp`)
```
State Machine:
  IDLE → TURNING → SETTLING → COMPLETE

Parameters:
  - turn_speed_dps = 200.0f       (degrees per second, conservative)
  - tolerance_deg = 3.0f          (±3° is acceptable)
  - settle_time_ms = 200          (wait 200ms after reaching target)

Algorithm:
  1. Record start_heading = fusion_get_heading()
  2. Calculate target_heading = start_heading + turn_angle
     (normalize to [-180, 180])
  3. TURNING state:
     - error = target_heading - current_heading (wrapped to [-180,180])
     - If |error| > 10°: drive at full turn_speed_dps
     - If |error| < 10°: use PID for precision approach
     - Switch to SETTLING when |error| < tolerance_deg
  4. SETTLING state:
     - Count settle timer
     - If heading drifts out of tolerance, go back to TURNING
     - If stable for settle_time_ms → COMPLETE

Heading Source: IMU gyro (primary)
  - During in-place turns, wheels may slip on smooth surfaces
  - Gyro measures true angular rotation regardless of wheel contact
  - Use fused heading (0.98 gyro + 0.02 encoder) for best results

Cross-Check (logging only):
  - encoder_turn_angle = (right_mm - (-left_mm)) / WHEEL_BASE_MM × (180/π)
  - Log both IMU angle and encoder angle for comparison
  - Large discrepancy indicates wheel slip or wheelbase error
```

#### Turn PID Gains (starting values)
```
kp = 8.0    (heading error in degrees → motor speed)
ki = 0.0    (no integral for turning)
kd = 1.0    (dampen oscillation at target)
Output limits: [-turn_speed_dps, +turn_speed_dps]
```

### Phase 5B Test Sketch
```
BTN_START sequence:
  1. Turn LEFT 90° → pause 1s → display angle
  2. Turn RIGHT 90° → pause 1s → display angle (should return to ~0°)
  3. Turn LEFT 90° → Turn LEFT 90° → Turn LEFT 90° → Turn LEFT 90°
     (four 90° turns = should return to ~0°)

Display: Target angle, Current fused heading, Encoder-derived angle
Serial: Heading data at 100Hz for graphing/analysis
```

---

## Stage C: ToF Integration for Turn Decisions

### When to Turn (Threshold Logic)
```
Wall Detection Thresholds (for 180mm cell):
  - FRONT_WALL_STOP_MM = 50     (stop when front wall this close)
  - FRONT_WALL_SLOW_MM = 100    (start decelerating)
  - SIDE_WALL_PRESENT_MM = 150  (wall detected if closer than this)
  - SIDE_WALL_ABSENT_MM = 200   (opening detected if farther than this)

Decision Tree (simple left-wall-follower):
  1. Drive straight
  2. If front_distance < FRONT_WALL_SLOW_MM: start decelerating
  3. If front_distance < FRONT_WALL_STOP_MM: full stop
  4. Read side sensors:
     a. If left is open: turn left 90°
     b. Else if front is open: continue straight (shouldn't happen, we stopped)
     c. Else if right is open: turn right 90°
     d. Else: turn 180° (dead end)
  5. Drive straight (repeat from 1)
```

### Timing Architecture
```
Timer ISR (1kHz):
  - Motor PID control
  - Encoder reading
  - Gyro reading + fusion

Main Loop (~20Hz):
  - Button polling
  - ToF sensor reading (blocking I2C, ~30ms per sensor)
  - OLED update
  - State machine decisions
```

---

## 🛠 Testing & Verification for Phase 5 (Revised)

### Stage A Tests
| Test | Expected Result |
|------|-----------------|
| Drive 180mm forward | Stops within ±5mm of 180mm |
| Heading during straight drive | Drift < 2° over 180mm |
| Drive 360mm (2 cells) | Stops within ±10mm, drift < 3° |
| Drive 900mm (5 cells) | Stops within ±20mm, drift < 5° |

### Stage B Tests
| Test | Expected Result |
|------|-----------------|
| Turn left 90° | Within ±3° of target |
| Turn right 90° | Within ±3° of target |
| Turn left 90° then right 90° | Return within ±3° of start |
| Four left 90° turns | Return within ±5° of start |
| Turn 180° | Within ±5° of target |

### Stage C Tests
| Test | Expected Result |
|------|-----------------|
| Drive toward wall, stop | Stops 30-60mm from wall |
| L-shaped corridor | Drives, detects wall, turns, drives |
| U-shaped corridor | Drives, turns 180°, drives back |
| Simple 3-cell maze | Navigates all cells correctly |
