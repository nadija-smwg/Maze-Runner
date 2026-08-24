# 🧠 Master Micromouse Implementation Plan (Revised)

> **Objective:** Complete the remaining Phase 4 (Sensor Fusion) and Phase 5 (Motion Control) using a bottom-up, test-driven strategy. Each sub-step produces a flashable, testable sketch that validates ONE behavior before moving on.
>
> **Key Insight from Testing:** X-distance from encoders is accurate but **angle/heading is incorrect** in the Full_Code. The root cause has been identified and is documented below.

---

## Current Status Summary

| Phase | Status | Notes |
|-------|--------|-------|
| Phase 1: Hardware I/O | ✅ Complete | GPIO, buttons, LEDs, battery, OLED, timer all working |
| Phase 2: Actuation & Encoders | ✅ Complete | Motors, PWM, encoder hardware mode all working |
| Phase 3: Sensing & I2C | ✅ Complete | 5× VL53L0X ToF sensors + MPU6050 IMU all working |
| Phase 4: Sensor Fusion | ⚠️ Partial | X-distance OK, **angle is INCORRECT** |
| Phase 5: Motion Control | ⚠️ Partial | Gyro hand-test mode works, but **no motorized movement** yet |
| Phase 6: Maze Solving | ❌ Not Started | Depends on Phases 4+5 |

---

## 🔴 Root Cause Analysis: Why Angle Is Wrong

### Problem
In Phase 4 testing, when the robot is pushed/rotated by hand, the X-distance tracks correctly but the **heading angle is inaccurate**. It shows values like 123° for an actual 90° turn.

### Root Causes Identified

#### 1. Heading Estimator `alpha = 1.0f` (100% Gyro, 0% Encoder)
In `heading_estimator.cpp` Line 35:
```cpp
float alpha = 1.0f; // tau / (tau + dt);
```
This was intentionally set to 100% gyro for Phase 5 testing, meaning the heading is **purely from gyro integration** with zero encoder correction. Gyro drift and noise accumulate unchecked.

#### 2. Encoder `dtheta` is hardcoded to `0.0f`
In `sensor_fusion.cpp` Line 33:
```cpp
heading_estimator_update(imu.gyro_z_dps, 0.0f, dt);
```
The encoder delta-theta is passed as `0.0f` — the heading estimator never receives encoder rotation data. Even if alpha were 0.98, the encoder correction term would be zero.

#### 3. `WHEEL_BASE_MM = 127.1f` was calibrated against a wrong reference
In `robot_config.h` Line 76:
```cpp
#define WHEEL_BASE_MM  127.1f  /**< Calibrated from 123 deg display for 90 deg actual */
```
This was tuned to make the encoder-derived angle match 90° when the display showed 123°. But since the display was showing the **gyro-only heading** (alpha=1.0), not the encoder heading, the calibration was done against the wrong reference. The actual physical wheelbase needs re-measurement.

#### 4. Phase 4 loop runs fusion at uncontrolled rate
In `Micromouse.ino` Phase 4 block (Lines 540-548):
```cpp
static uint32_t last_fusion_tick = millis();
uint32_t now = millis();
float dt = (now - last_fusion_tick) / 1000.0f;
if (dt > 0.0f) {
    fusion_update(dt);
    last_fusion_tick = now;
}
```
No throttle! This runs fusion at ~100kHz, causing `dt` to be ~0.00001s. The 0.85 low-pass filter on the gyro was tuned for ~100Hz (10ms intervals). At 100kHz, each sample only contributes 0.15 × 0.00001 ≈ nothing — the filter becomes a brick wall that swallows the gyro data.

> [!IMPORTANT]
> Phase 5 already fixed this by throttling fusion to exactly 10ms intervals (100Hz), which is why the hand-rotation gyro test in Phase 5 works correctly. **The fix must be applied to Phase 4 as well, and then forward into all future motion tests.**

#### 5. `GEAR_RATIO = 18.85f` — Empirically tuned, needs validation
In `robot_config.h` Line 26:
```cpp
#define GEAR_RATIO  18.85f  /**< Calibrated for 100cm actual = 100cm OLED */
```
This was tuned until the OLED showed 100cm when the robot traveled 100cm physically. This is good for distance, but the true gear ratio might be different. The MM_PER_COUNT derived from it could be slightly off for turns.

---

## 🆕 Revised Strategy: Bottom-Up Test-Driven Approach

Instead of trying to implement everything at once, we will build and validate **one capability at a time**, in this exact order:

### Stage A: Drive Straight One Cell (180mm) — Encoder + IMU
### Stage B: Turn Exactly 90° In-Place — Encoder + IMU  
### Stage C: Integrate ToF Sensors for Turn Decisions
### Stage D: Combine Into Cell-By-Cell Navigation
### Stage E: Wall Following & Maze Solving

Each stage produces a **self-contained test sketch** in the Full_Code's Phase test system.

---

## Proposed Changes

### Stage A: Drive Straight One Cell (180mm)

> **Goal:** Robot drives forward exactly 180mm (one maze cell) and stops. Uses encoder counts as the primary distance measurement. Uses IMU heading correction to drive **perfectly straight** (compensate for wheel speed differences).

#### [MODIFY] `sensors/sensor_fusion.cpp`
- Fix `fusion_update()` to pass actual encoder `dtheta` to heading estimator
- Currently passes `0.0f`, must pass the real encoder-derived rotation delta

#### [MODIFY] `localization/heading_estimator.cpp`
- Change `alpha` from `1.0f` (100% gyro) to proper complementary filter value `0.98f`
- This fuses 98% gyro (short-term accurate) with 2% encoder (long-term stable)

#### [MODIFY] `control/speed_controller.cpp`
- Implement the PID speed controller using the proven pattern from `Testing Codes/Motors_Motion_Control_With_PID.ino`
- Left PID and Right PID objects control individual wheel speeds
- Port the working gains: `kp=1.8, ki=0.8, kd=0.02`

#### [MODIFY] `control/heading_controller.cpp`
- Implement heading PID: compares target heading (0° for straight) with fused heading
- Output is a differential correction applied to left/right wheel speeds

#### [MODIFY] `control/velocity_controller.cpp`
- Implement differential drive mixer: converts (linear_v, angular_ω) → (v_left, v_right)
- `v_left = v - (ω × WHEEL_BASE_MM / 2)`
- `v_right = v + (ω × WHEEL_BASE_MM / 2)`

#### [MODIFY] `control/cell_controller.cpp`
- Implement `cell_start_move(1)`: calculates target distance = `1 × CELL_SIZE_MM = 180mm`
- Monitors encoder distance, starts deceleration when within stopping distance
- Uses simple trapezoidal profile: accelerate → cruise → decelerate → stop

#### [MODIFY] `control/motion_controller.cpp`
- Wire up the 1kHz control loop:
  1. `fusion_update(dt)` — update pose
  2. Read target from cell_controller or turn_controller
  3. `heading_controller_update()` → angular correction
  4. `velocity_controller_update(target_v, correction_ω)` → wheel speeds
  5. `speed_controller_update()` → PWM output

#### [MODIFY] `Micromouse.ino` — Phase 5 Test Block
- Replace the current "gyro hand test" with a **motorized straight-line test**
- Press BTN_START → robot drives forward exactly 180mm and stops
- OLED shows: distance traveled, heading error, target vs actual speed
- Serial logs all data for analysis

#### Verification
- Place robot at a known position with ruler alongside
- Press BTN_START
- Robot drives forward and stops
- **PASS:** Robot stops within ±5mm of 180mm AND heading drift < 2°
- If robot curves: tune heading PID gains
- If distance is off: verify GEAR_RATIO / MM_PER_COUNT calibration

---

### Stage B: Turn Exactly 90° In-Place

> **Goal:** Robot rotates exactly 90° on the spot using in-place turning (left wheel backward, right wheel forward). IMU provides the primary angle measurement. Encoder differential provides secondary confirmation.

#### [MODIFY] `control/turn_controller.cpp`
- Implement `turn_start_inplace(TURN_LEFT_90)`:
  1. Record starting heading from fusion
  2. Set target heading = start + 90° (or -90° for right)
  3. Command motors: left backward, right forward at moderate PWM
  4. PID loop on heading error: `error = target_heading - current_heading`
  5. When |error| < `TURN_TOLERANCE_DEG` (2°) AND angular rate < 5°/s → turn complete

#### Strategy: IMU-Primary with Encoder Cross-Check
- **Primary sensor: IMU gyro** — measures true rotation regardless of wheel slip
- **Secondary check: Encoders** — differential `(right_mm - left_mm) / WHEEL_BASE_MM` should also show ~90°
  - If IMU says 90° but encoders say 70° → wheel slip occurred (expected, not a problem)
  - If IMU says 90° but encoders say 110° → possible WHEEL_BASE_MM calibration issue
- **Decision: Trust IMU for turns, trust encoders for straight distance**

#### [MODIFY] `Micromouse.ino` — Phase 5 Test Block
- BTN_START → robot turns 90° left, pauses 1s, turns 90° right, pauses 1s
- OLED shows: target angle, current fused heading, encoder-derived angle
- Serial logs heading data at 100Hz for graphing

#### Verification
- Place robot on flat surface with right-angle reference
- Press BTN_START
- Robot turns 90° left
- **PASS:** Final heading within ±3° of 90° AND robot stops cleanly (no oscillation)
- If overshoot: reduce turn speed or increase D gain
- If undershoot: check gyro calibration or increase P gain

---

### Stage C: Integrate ToF Sensors for Turn Decisions

> **Goal:** Robot reads ToF sensor values while moving and decides when to stop / turn based on wall distances. This is the bridge between "blind movement" and "maze navigation."

#### [MODIFY] `Micromouse.ino` — Phase 6 Test Block
- Combine Stage A + Stage B into a simple reactive behavior:
  1. Drive straight using Stage A logic
  2. While driving, poll front ToF sensor
  3. When `front_distance < WALL_THRESHOLD_FRONT_MM` (e.g., 80mm): stop
  4. Check left and right ToF sensors:
     - If left is open (> threshold) → turn left 90°
     - If right is open (> threshold) → turn right 90°
     - If both blocked → turn 180°
  5. After turn, drive straight again (repeat)

#### [MODIFY] `sensors/distance_manager.cpp`
- Tune `WALL_THRESHOLD_FRONT_MM` and `WALL_THRESHOLD_SIDE_MM` for actual maze cell (180mm)
- Ensure `distance_manager_update()` is called at appropriate rate (~20Hz) without blocking motor control

#### Key Timing Constraint
> [!WARNING]
> The VL53L0X sensors take ~30-90ms per read over I2C. Reading all 5 sensors sequentially blocks the main loop for 150-450ms. During this time, the 1kHz motor control ISR must still run independently.
>
> **Solution:** Only read sensors in the main `loop()` at 20Hz. The 1kHz timer ISR handles motor PID + fusion independently.

#### Verification
- Place robot in a simple corridor (two parallel walls)
- Press BTN_START
- Robot drives forward, detects wall, turns, drives back
- **PASS:** Robot successfully navigates a simple L-shaped corridor

---

### Stage D: Combine Into Cell-By-Cell Navigation

> **Goal:** Robot drives cell-by-cell through the maze, reading walls at each cell center and deciding the next direction.

#### Implementation
1. Drive into cell center (90mm past entrance = center of 180mm cell)
2. Stop momentarily
3. Read all 5 ToF sensors → determine walls (front, left, right)
4. Use simple left-wall-following or flood-fill to decide next direction
5. Turn to face that direction
6. Drive forward one cell
7. Repeat

#### Files Modified
- `robot/robot_state_machine.cpp` — Implement the state machine
- `maze/maze.cpp` — Store wall data in the 16×16 array
- `maze/flood_fill.cpp` — Calculate shortest path to center

---

### Stage E: Wall Following & Speed Optimization

> **Goal:** While driving straight, use side ToF sensors to keep the robot centered in the corridor. Then implement fast-run optimization.

#### Files Modified
- `control/wall_follower.cpp` — PD controller on lateral error
- `motion/motion_profile.c` — S-curve velocity profiles for fast run
- `robot/robot_state_machine.cpp` — Add fast-run state

---

## Robot Physical Constants (Current Calibrated Values)

| Parameter | Value | Source | Confidence |
|-----------|-------|--------|------------|
| `WHEEL_DIAMETER_MM` | 43.0 | Measured | ✅ High |
| `GEAR_RATIO` | 18.85 | Tuned: 100cm = 100cm OLED | ✅ High (for distance) |
| `ENCODER_PPR` | 7 | Datasheet | ✅ High |
| `ENCODER_CPR` | 527.8 (7×18.85×4) | Derived | ✅ High |
| `WHEEL_CIRCUMFERENCE_MM` | 135.09 (π×43) | Derived | ✅ High |
| `MM_PER_COUNT` | 0.2559 (135.09/527.8) | Derived | ✅ High |
| `WHEEL_BASE_MM` | 127.1 | ⚠️ Calibrated against wrong ref | ❌ **Needs re-calibration** |
| `CELL_SIZE_MM` | 180 | Standard micromouse | ✅ High |

> [!IMPORTANT]
> `WHEEL_BASE_MM` MUST be re-calibrated after fixing the heading estimator. The procedure:
> 1. Fix alpha to 0.98 and pass real encoder dtheta
> 2. Spin robot exactly 360° by hand (use physical marks)
> 3. Read encoder-derived angle. If it shows 380°, then `WHEEL_BASE_MM` is too small. Adjust until encoder reports 360° for a physical 360° spin.
> 4. Then verify gyro reads 360° as well (it should, independently).

---

## Verification Plan

### Automated Tests
- Not applicable (embedded firmware, no unit test framework on target)

### Manual Verification
| Test | Stage | Expected Result |
|------|-------|-----------------|
| Drive 180mm straight | A | ±5mm accuracy, <2° drift |
| Drive 360mm (2 cells) | A | ±10mm accuracy, <3° drift |
| Turn 90° left | B | ±3° accuracy, no oscillation |
| Turn 90° right | B | ±3° accuracy, no oscillation |
| Turn 180° | B | ±5° accuracy |
| 4× 90° turns = 360° | B | Return to within ±5° of start |
| Detect front wall and stop | C | Stops 30-50mm from wall |
| Navigate L-corridor | C | Drives, turns, drives back |
| Cell-by-cell exploration | D | Maps walls correctly |
| Wall-following centering | E | Stays centered ±10mm |

---

## User Review Required

> [!IMPORTANT]
> This revised plan takes a fundamentally different approach: **one capability at a time, validated before moving on.** The old plan tried to implement everything simultaneously, leading to bugs that were impossible to isolate.
>
> **Key questions for you:**
> 1. For straight-line driving (Stage A), should we use **encoders as primary** (count distance) with **IMU as heading correction only**? Or should both contribute to distance?
> 2. What is the **actual physical wheelbase** of your robot in mm? (Measure center-to-center of the two wheel contact patches)
> 3. Do you have a ruler or tape to verify 180mm travel distance during testing?
