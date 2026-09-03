# Micromouse — Full Implementation Plan (Phase 4 → Competition)

> [!IMPORTANT]
> **Phases 1–3 are 100% complete and hardware-verified.**
> Sensors are calibrated. Motor dead-zones are measured. All constants are locked.
> This plan is the complete roadmap from here to a maze-solving robot.

---

## Software Architecture (Final Target)

```
         ┌──────────────────┐   ┌──────────────────┐   ┌──────────────────┐
         │   MPU6050 IMU    │   │ 5× VL53L0X ToF   │   │ Quadrature Enc.  │
         └────────┬─────────┘   └────────┬──────────┘   └────────┬─────────┘
                  │                      │                        │
                  └──────────────────────┼────────────────────────┘
                                         ↓
                              ┌──────────────────────┐
                              │   Sensor Fusion /    │
                              │   State Estimator    │
                              │  x, y, θ, v, ω,      │
                              │  walls, distances    │
                              └───────────┬──────────┘
                                          ↓
                              ┌──────────────────────┐
                              │    Maze Mapping      │
                              │  16×16 wall/visited  │
                              └───────────┬──────────┘
                                          ↓
                              ┌──────────────────────┐
                              │     Flood Fill       │
                              └───────────┬──────────┘
                                          ↓
                              ┌──────────────────────┐
                              │   Motion Planner     │
                              │  (path → commands)   │
                              └───────────┬──────────┘
                                          ↓
                     ┌────────────────────────────────────┐
                     │       Movement Controller          │
                     │  Velocity PID | Heading PID        │
                     │  Wall PD      | Profile generator  │
                     └────────────────────┬───────────────┘
                                          ↓
                              ┌──────────────────────┐
                              │  Motor PWM (TIM1)    │
                              │  TB6612FNG           │
                              └──────────────────────┘
```

---

## Hardware Constants (Locked — Do Not Change)

| Constant | Value | How Measured |
|----------|-------|-------------|
| `LEFT_ENCODER_CPR` | 581 | Phase 2 State 7 hand-count |
| `RIGHT_ENCODER_CPR` | 581 | Phase 2 State 8 hand-count |
| `WHEEL_DIAMETER_MM` | 43.0 mm | Calipers |
| `WHEEL_BASE_MM` | 80.0 mm | Calipers (center-to-center) |
| `LEFT_MOTOR_DEAD_PWM` | 300 | Phase 2 State 5 sweep |
| `RIGHT_MOTOR_DEAD_PWM` | 250 | Phase 2 State 6 sweep |
| `LEFT_MM_PER_COUNT` | 0.2325 mm | π × 43 / 581 |
| `KFF` (feed-forward) | 8.4 | PWM 1500 → 179 mm/s measured |

---

## Pre-Flight Checklist

Before starting any new code, confirm:

- [ ] Measure `BATTERY_DIVIDER_RATIO`: multimeter reading (mV) ÷ Serial `Bat:` value
- [ ] Confirm gyro sign: spinning robot **clockwise** → does `gz_dps` return **positive** or **negative**?
- [ ] Measure `SENSOR_FRONT_OFFSET_MM`: wheel axle center → front ToF sensor face (mm)
- [ ] (Optional) 10-revolution CPR recheck for ±0.5% accuracy

---

## Phase 4 — Closed-Loop Velocity + Pose Estimation

> **Prerequisite:** Phase 3 complete ✅
> **Exit criterion:** Robot drives 500mm straight within ±1 cm, with pose (X, Y, θ) printed live.

---

### Step 4.1 — PI Velocity Controller Tuning

**Status of code:** `speed_controller.cpp` and `velocity_controller.cpp` are fully implemented. This is tuning only.

**File to edit:** [`speed_controller.cpp`](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/speed_controller.cpp)

**Tuning sequence (do NOT skip steps):**

| Step | Setting | Goal |
|------|---------|------|
| 1 | `KP=0, KI=0`. Adjust `KFF` from 8.4 | Open-loop speed ≈ 300 mm/s |
| 2 | Fix KFF. Increase `KP` by +0.5 | Reaches target in <300ms, no oscillation |
| 3 | Increase `KI` by +0.1 | Steady-state error < ±5 mm/s |
| 4 | Reduce `KI` if you see "windup wobble" | Stable, flat response |

**Test 4.1 — Step response (PHASE_4_TEST_MODE):**
```
[PI] L:0.0   R:0.0     ← start
[PI] L:185   R:180     ← ramping
[PI] L:298   R:302     ← near target
[PI] L:300   R:300     ← settled ✅
```
**Pass:** Settles within ±5 mm/s of 300 mm/s in <500ms, no sustained oscillation.

**Test 4.2 — Straight line:**
- Set `target = 300 mm/s forward`, `ω = 0`
- Run robot on flat floor for 500mm
- `DistL` and `DistR` must be within ±5 mm of each other
- Robot must stay within ±1 cm of a straight line

---

### Step 4.2 — Odometry Kinematics

**File:** [`odometry.cpp`](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/localization/odometry.cpp)
**Status:** Stub with math commented out as TODO.

**Implement (6 lines):**
```cpp
void odometry_update(float left_delta_mm, float right_delta_mm) {
    float d_center = (left_delta_mm + right_delta_mm) / 2.0f;
    float d_theta  = (right_delta_mm - left_delta_mm) / WHEEL_BASE_MM;

    _current_pose.theta_rad = pose_normalize_angle_rad(
                                  _current_pose.theta_rad + d_theta);
    _current_pose.x_mm += d_center * cosf(_current_pose.theta_rad);
    _current_pose.y_mm += d_center * sinf(_current_pose.theta_rad);
}
```

**Who calls it (add to 1kHz ISR):**
```cpp
int32_t l_ticks = encoder_get_delta(ENCODER_LEFT);
int32_t r_ticks = encoder_get_delta(ENCODER_RIGHT);
odometry_update(l_ticks * LEFT_MM_PER_COUNT, r_ticks * RIGHT_MM_PER_COUNT);
```

**Test 4.3 — Push linear:**
- Push robot exactly 100mm with a ruler. Do NOT use motors.
- Serial must show: `X ≈ 100.0, Y ≈ 0.0, θ ≈ 0.0 rad`
- If X shows 80: `WHEEL_DIAMETER_MM` needs to increase proportionally
- If X shows 120: decrease `WHEEL_DIAMETER_MM`

**Test 4.4 — Push rotate:**
- Manually spin robot exactly 90° in place
- Serial must show: `θ ≈ 1.5708 rad (90°)`
- If off by >3°: `new_WHEEL_BASE = 80.0 × (actual_deg / serial_deg)`

---

### Step 4.3 — Heading Estimator (Complementary Filter)

**File:** [`heading_estimator.cpp`](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/localization/heading_estimator.cpp)
**Status:** Completely empty stub.

**Implement:**
```cpp
static float _fused_heading_rad = 0.0f;

void heading_estimator_init(void) { _fused_heading_rad = 0.0f; }

void heading_estimator_update(float gyro_z_dps,
                               float encoder_dtheta_rad, float dt) {
    // Gyro: fast response, drifts over time
    float gyro_dtheta = (gyro_z_dps * 0.017453f) * dt;

    // Complementary filter:
    // 98% gyro  → fast, immune to wheel slip
    // 2%  encoder → long-term, drift-free
    _fused_heading_rad = 0.98f * (_fused_heading_rad + gyro_dtheta)
                       + 0.02f * encoder_dtheta_rad;

    // Normalize to [-π, π]
    while (_fused_heading_rad >  3.14159f) _fused_heading_rad -= 6.28318f;
    while (_fused_heading_rad < -3.14159f) _fused_heading_rad += 6.28318f;
}

float heading_estimator_get(void) { return _fused_heading_rad; }
```

> [!IMPORTANT]
> **Gyro sign check:** Spin robot clockwise (right turn). If `gz_dps` is NEGATIVE, add a `-` sign in the `gyro_dtheta` line above. If POSITIVE, leave as-is.

**Test 4.5 — Stationary drift:**
- Robot sits still for 60 seconds
- `fused_heading` must stay within ±0.01 rad (≈0.6°) — no drift

**Test 4.6 — Spin response:**
- Spin robot physically 90° right
- `fused_heading` must show ≈+1.57 rad
- After stopping, heading must NOT continue drifting

---

### Step 4.4 — Wire 1kHz Control Loop

**File:** [`Micromouse.ino`](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/Micromouse.ino)

Add `#define PHASE_4_TEST_MODE 1`. The timer callback brings all subsystems together:

```cpp
void phase4_timer_callback(void) {
    // 1. Encoder deltas (must be read FIRST, before velocity update consumes them)
    int32_t l_ticks = encoder_get_delta(ENCODER_LEFT);
    int32_t r_ticks = encoder_get_delta(ENCODER_RIGHT);
    float l_mm = l_ticks * LEFT_MM_PER_COUNT;
    float r_mm = r_ticks * RIGHT_MM_PER_COUNT;

    // 2. Velocity LPF (smoothed mm/s for PI controller)
    encoder_update_velocity(CONTROL_LOOP_DT_S);

    // 3. Odometry pose (X, Y, θ from encoders)
    odometry_update(l_mm, r_mm);

    // 4. Heading fusion (gyro + encoder)
    float gz = mpu6050_get_filtered_gz_dps();
    heading_estimator_update(gz, odometry_get_pose().theta_rad,
                             CONTROL_LOOP_DT_S);

    // 5. Velocity + heading PI
    velocity_controller_update(_target_v_mm_s, _target_w_rad_s);
}
```

**Phase 4 is complete when:**
- ✅ Test 4.1: PI step response passes
- ✅ Test 4.2: Straight line 500mm within ±1 cm
- ✅ Test 4.3: Push 100mm → X shows 100 ±2
- ✅ Test 4.4: Spin 90° → θ shows 1.57 ±0.05
- ✅ Test 4.5/4.6: Heading estimator stable, no drift

---

## Phase 5 — Motion Primitives & Wall Following

> **Prerequisite:** Phase 4 complete ✅
> **Exit criterion:** Robot drives one maze cell (180mm) reliably, turns 90°, and self-centres in a corridor.

---

### Step 5.1 — Straight-Line Heading Control

Before using motion profiles, add **heading correction** to straight driving. This uses the fused heading to cancel any drift:

```cpp
// In 1kHz ISR during straight motion:
float heading_error = 0.0f - heading_estimator_get();  // target = 0 (straight)
float w_correction  = 2.0f * heading_error;            // Kp_heading = 2.0 rad/s per rad error
velocity_controller_update(target_speed, w_correction);
```

**Test 5.1 — Heading-corrected straight:**
- Drive 1000mm
- Robot must stay within ±5mm of centre line
- Heading drift < 1° over 1000mm

---

### Step 5.2 — Trapezoidal Motion Profile

**File:** [`straight_motion.cpp`](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/motion/straight_motion.cpp)

`motion_profile.c` already provides `profile_compute_linear()` and `profile_get_speed()`. Wire the stub:

```cpp
void straight_motion_start(float distance_mm, float start_speed,
                            float end_speed,  float max_speed) {
    profile_compute_linear(&_linear_profile, distance_mm,
                            max_speed, start_speed, end_speed,
                            SEARCH_ACCEL_MM_S2, SEARCH_DECEL_MM_S2);
    _current_target_speed = start_speed;
}

void straight_motion_update(float distance_traveled_mm) {
    _current_target_speed = profile_get_speed(&_linear_profile,
                                               distance_traveled_mm);
}

bool straight_motion_is_complete(void) { return _linear_profile.done; }
```

> [!NOTE]
> This creates a smooth velocity profile: 0 → ramp up → cruise → ramp down → 0.
> Never command an instant jump from 0 to full speed — it will slip wheels and break odometry.

---

### Step 5.3 — Cell Drive Primitive

**File:** [`motion_controller.cpp`](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/motion_controller.cpp)

```cpp
static bool _cell_moving = false;

void motion_drive_cell(void) {
    odometry_set_pose({0.0f, 0.0f, 0.0f});   // local reset
    straight_motion_start(CELL_SIZE_MM,       // 180mm
                          0.0f,              // start at rest
                          0.0f,              // stop at end
                          SEARCH_MAX_SPEED_MM_S);
    _cell_moving = true;
}

// Call every 1ms in ISR:
void motion_controller_update(void) {
    if (!_cell_moving) return;

    float dist = fabsf(odometry_get_pose().x_mm);
    straight_motion_update(dist);

    float v = straight_motion_get_target_speed();
    float heading_err = 0.0f - heading_estimator_get();
    float w = 2.0f * heading_err;                 // heading correction

    // Wall following correction (injected additively)
    float wall_w = wall_follower_update(
        distance_get_filtered(SENSOR_LEFT),
        distance_get_filtered(SENSOR_RIGHT),
        CONTROL_LOOP_DT_S);

    velocity_controller_update(v, w + wall_w);

    if (straight_motion_is_complete()) {
        velocity_controller_update(0.0f, 0.0f);
        _cell_moving = false;
    }
}
```

**Test 5.2 — Single cell:**
- Command `motion_drive_cell()`
- Robot must travel 180 ±5mm and stop cleanly

**Test 5.3 — 4-cell straight:**
- 4× `motion_drive_cell()` in sequence
- Total 720 ±10mm, heading drift < 3°

---

### Step 5.4 — 90° Turn Primitive (Gyro-Guided)

```cpp
static bool _turning = false;
static float _turn_target_rad = 0.0f;

void motion_turn_left_90(void) {
    _turn_target_rad = heading_estimator_get() - 1.5708f;  // -90°
    _turning = true;
}

void motion_turn_right_90(void) {
    _turn_target_rad = heading_estimator_get() + 1.5708f;  // +90°
    _turning = true;
}

// In ISR, every 1ms:
if (_turning) {
    float heading_err = heading_estimator_get() - _turn_target_rad;
    // Normalize error to [-π, π]
    while (heading_err >  3.14159f) heading_err -= 6.28318f;
    while (heading_err < -3.14159f) heading_err += 6.28318f;

    if (fabsf(heading_err) > 0.035f) {  // >2° remaining
        float w = -5.0f * heading_err;  // Kp_turn = 5 rad/s per rad
        w = clampf(w, -MAX_TURN_RAD_S, MAX_TURN_RAD_S);
        velocity_controller_update(0.0f, w);
    } else {
        velocity_controller_update(0.0f, 0.0f);
        _turning = false;
    }
}
```

> [!IMPORTANT]
> Do NOT use `delay()` for turns. Always use closed-loop heading control.

**Test 5.4 — Single 90° turn:**
- Command `motion_turn_left_90()`
- Heading changes 90° ±2°

**Test 5.5 — 360° closure:**
- 4× `motion_turn_left_90()`
- Robot returns to original heading ±3°
- This is the most important test for `WHEEL_BASE_MM` accuracy

---

### Step 5.5 — Wall Following PD Controller

**File:** [`wall_follower.cpp`](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/wall_follower.cpp)

```cpp
static float _kp = 0.8f;
static float _kd = 50.0f;
static float _last_error = 0.0f;
static bool  _enabled = false;
static const float TARGET_WALL_MM = 60.0f;  // ~centre of 180mm cell

// Hysteresis thresholds — prevents wall flicker
static const float WALL_ON_MM  = 80.0f;
static const float WALL_OFF_MM = 90.0f;

float wall_follower_update(float left_mm, float right_mm, float dt) {
    if (!_enabled) return 0.0f;

    // Both walls visible → centre between them
    // One wall visible → maintain fixed distance from that wall
    // No wall visible  → no correction (rely on heading only)
    float error;
    bool left_ok  = (left_mm  > 5.0f && left_mm  < WALL_OFF_MM);
    bool right_ok = (right_mm > 5.0f && right_mm < WALL_OFF_MM);

    if (left_ok && right_ok) {
        error = left_mm - right_mm;
    } else if (left_ok) {
        error = left_mm - TARGET_WALL_MM;
    } else if (right_ok) {
        error = TARGET_WALL_MM - right_mm;
    } else {
        _last_error = 0.0f;
        return 0.0f;
    }

    float d_err = (error - _last_error) / dt;
    _last_error = error;
    return _kp * error + _kd * d_err;  // returns rad/s angular correction
}

void wall_follower_enable(bool enable) { _enabled = enable; }
void wall_follower_reset(void) { _last_error = 0.0f; }
```

**Tuning wall follower:**
1. Start: `kp=0.5, kd=0`
2. Place in corridor, push sideways — should return to centre in ~300ms
3. If oscillating: increase `kd` to 20, 50, 100 until smooth

**Test 5.6 — Corridor centering:**
- Drive 5 cells (900mm) in a physical corridor
- Robot stays centred ±10mm
- Push sideways mid-run → robot recovers

**Test 5.7 — L-shape corner:**
- cell + 90° turn + cell
- No wall collision, smooth recovery in corridor

---

## Phase 6 — Maze Mapping & Cell Navigation

> **Prerequisite:** Phase 5 complete ✅

### Step 6.1 — Robot State Struct

Create a single unified state that all higher-level code uses:

```cpp
struct RobotState {
    // Pose
    float x_mm, y_mm, theta_rad;
    // Velocity
    float linear_mm_s, angular_rad_s;
    // Maze position
    int cell_x, cell_y;
    Direction heading;       // NORTH, EAST, SOUTH, WEST
    // Wall detection (hysteresis applied)
    bool front_wall, left_wall, right_wall;
    // Raw filtered distances (mm)
    float front_mm, left_mm, right_mm;
};
```

### Step 6.2 — Absolute Direction Conversion

When the robot reads sensor walls, convert relative → absolute:

```cpp
// If heading == EAST:
//   front  → EAST wall
//   left   → NORTH wall
//   right  → SOUTH wall
Direction relative_to_absolute(Direction robot_heading,
                                RelativeDir sensor_side);
```

### Step 6.3 — Maze Data Structure

Each cell stores walls as bits and distance (for flood fill):

```cpp
struct Cell {
    uint8_t  walls;      // bit 0=N, 1=E, 2=S, 3=W
    uint16_t distance;   // flood fill distance value
    bool     visited;
};

Cell maze[MAZE_SIZE][MAZE_SIZE];  // 16×16 = 256 cells
```

### Step 6.4 — Flood Fill

**Status:** Already implemented in `src/maze/`. Wire it to the robot state.

Flow each iteration:
1. Enter cell
2. Read sensors → update `maze[x][y].walls`
3. Run flood fill from goal
4. Choose neighbor with lowest distance value
5. Convert direction → `motion_turn_XX` + `motion_drive_cell()`

---

## Phase 7 — Speed Optimization

> **Prerequisite:** Robot solves maze reliably ✅

### Step 7.1 — Increase Cruise Speed

Increase `SEARCH_MAX_SPEED_MM_S` in 50 mm/s increments:
- 300 → 400 → 500 → 600 → ... → until slippage
- Check Serial: `DistL ≈ DistR` within ±10mm still

### Step 7.2 — Smooth Turn-Into-Straight

Instead of stop → turn → start:
```
Decelerate → enter turn arc → accelerate
```
Use `arc_motion.cpp` (stub exists). Reduces cell traversal time by ~30%.

### Step 7.3 — Multi-Cell Straight Optimization

If the motion planner sees 3 consecutive straight cells, merge them:
```
cell + cell + cell  →  STRAIGHT 540mm
```
This eliminates two start/stop sequences.

---

## Complete Progressive Test Sequence

```
Test 1  Motor spin (each direction)             ✅ Done (Phase 2)
Test 2  Encoder counts (1 rev, 10 rev)          ✅ Done (Phase 2)
Test 3  Dead-zone measurement                   ✅ Done (Phase 2)
Test 4  Velocity PID step response              ← Phase 4 Test 4.1
Test 5  Straight 500mm encoder accuracy         ← Phase 4 Test 4.2
Test 6  Push test — odometry X/Y/θ             ← Phase 4 Tests 4.3/4.4
Test 7  Heading estimator drift check           ← Phase 4 Tests 4.5/4.6
Test 8  Heading-corrected straight (1000mm)     ← Phase 5 Test 5.1
Test 9  Single cell drive (180mm profile)       ← Phase 5 Test 5.2
Test 10 4-cell straight accuracy               ← Phase 5 Test 5.3
Test 11 90° turn accuracy                      ← Phase 5 Test 5.4
Test 12 360° closure                           ← Phase 5 Test 5.5
Test 13 Corridor wall following                ← Phase 5 Test 5.6
Test 14 L-shape corner (cell+turn+cell)        ← Phase 5 Test 5.7
Test 15 Wall detection + maze update           ← Phase 6
Test 16 Flood fill path selection              ← Phase 6
Test 17 Complete maze solve (slow)             ← Phase 6
Test 18 Speed optimization                     ← Phase 7
```

---

## Files To Implement (Ordered)

| Priority | File | Current State | What To Do |
|----------|------|--------------|------------|
| 1 | `Micromouse.ino` | Partial | Add PHASE_4_TEST_MODE, wire 1kHz callback |
| 2 | `odometry.cpp` | Stub | Implement 6-line kinematics |
| 3 | `heading_estimator.cpp` | Stub | Implement complementary filter |
| 4 | `speed_controller.cpp` | ✅ Complete | Tune KFF/KP/KI only |
| 5 | `velocity_controller.cpp` | ✅ Complete | No changes needed |
| 6 | `straight_motion.cpp` | Stub | Wire to motion_profile.c |
| 7 | `motion_controller.cpp` | Stub | Implement drive_cell + turn_90 |
| 8 | `wall_follower.cpp` | Stub | Implement PD controller |
| 9 | `src/maze/` files | Existing | Wire to robot state |

---

## Open Questions (Answers Needed Before Coding)

> [!IMPORTANT]
> **Q1 — Gyro sign:** Spin robot clockwise (right turn, viewed from above).
> Does `mpu6050_get_filtered_gz_dps()` return **positive** or **negative**?
> This controls the sign inside `heading_estimator_update()`.

> [!IMPORTANT]
> **Q2 — Front sensor offset:** Measure from **wheel axle center** to **front ToF sensor face** in mm.
> This is used to calculate when to stop before a wall in `motion_drive_cell()`.

> [!NOTE]
> **Q3 — Battery divider:** Multimeter mV ÷ Serial `Bat:` mV = `BATTERY_DIVIDER_RATIO`.
> Update `robot_config.h` line 323 when you have a reading.
