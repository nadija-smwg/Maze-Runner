# 🔧 Phase 5 Complete Report: Why Wheels Shutter, All Files Explained, Calibration Guide

---

## Table of Contents

1. [Your Shuttering Problem — Root Cause Diagnosis](#1-the-shuttering-problem)
2. [Why There Are Multiple PID Values](#2-why-multiple-pid-values)
3. [Every Phase 5 File — What It Does & Why](#3-every-phase-5-file-explained)
4. [Every Future Phase File — What It Does & Why](#4-future-phase-files)
5. [Step-by-Step Calibration for Smooth Motion](#5-calibration-guide)

---

## 1. The Shuttering Problem

### What You See
When you press START, the wheels **jerk violently** — rapid bursts of ON/OFF power. When you change Kp, the speed changes (slow vs fast) but the shuttering never goes away.

### Root Cause: The PID Control Loop Is Fighting Itself

Here is exactly what happens every millisecond (1kHz) inside your robot:

```
┌─────────────────────────────────────────────────────────────┐
│  Every 1ms (1000 times per second), this runs:              │
│                                                             │
│  1. Read encoder delta (how many counts in last 1ms?)       │
│  2. Convert to speed: delta × MM_PER_COUNT × 1000           │
│  3. PID compares target speed (150 mm/s) with measured      │
│  4. PID output + Feedforward → PWM to motors                │
└─────────────────────────────────────────────────────────────┘
```

**The problem is in Step 1 and Step 2.** At 1kHz (every 1ms), the encoder produces **extremely few counts per sample.**

#### The Math That Causes Shuttering

Your encoder specs:
- `ENCODER_PPR = 7` (7 pulses per motor revolution)
- `GEAR_RATIO = 18.85`
- `ENCODER_QUADRATURE = 4×`
- **`ENCODER_CPR = 7 × 18.85 × 4 = 527.8 counts per wheel revolution`**
- `WHEEL_CIRCUMFERENCE = π × 43 = 135.09 mm`
- **`MM_PER_COUNT = 135.09 / 527.8 = 0.2559 mm/count`**

At 150 mm/s target speed:
```
Counts per second = 150 / 0.2559 = 586 counts/s
Counts per 1ms    = 586 / 1000  = 0.586 counts/ms
```

> [!CAUTION]
> **You get LESS THAN 1 encoder count per control loop tick!**
>
> The encoder reads either **0 counts** or **1 count** each millisecond. There is no in-between. This is integer quantization — the encoder cannot produce 0.586 counts.

So the speed measurement oscillates like this:

| Tick | Encoder Delta | Calculated Speed | Actual Speed | Error |
|------|:---:|:---:|:---:|:---:|
| 1 | 0 counts | **0 mm/s** | 150 mm/s | huge |
| 2 | 1 count | **255.9 mm/s** | 150 mm/s | huge |
| 3 | 0 counts | **0 mm/s** | 150 mm/s | huge |
| 4 | 1 count | **255.9 mm/s** | 150 mm/s | huge |
| 5 | 1 count | **255.9 mm/s** | 150 mm/s | huge |
| 6 | 0 counts | **0 mm/s** | 150 mm/s | huge |

The measured speed **bounces between 0 and 256 mm/s**, even though the actual wheel speed is a steady 150 mm/s!

### What the PID Sees

```
Tick 1: measured=0 → error=+150 → PID says "FULL POWER!"     → big PWM spike
Tick 2: measured=256 → error=-106 → PID says "BRAKE HARD!"   → PWM drops
Tick 3: measured=0 → error=+150 → PID says "FULL POWER!"     → big PWM spike
... (repeat forever)
```

**This is your shuttering.** The PID is reacting to measurement noise, not to actual speed changes.

### Why Changing Kp Doesn't Fix It

| Kp Value | What Happens |
|:---:|---|
| Low (0.2) | PID reacts weakly → slower overall → **still shutters** (just smaller oscillations) |
| Medium (1.0) | PID reacts moderately → moderate speed → **still shutters** |
| High (2.0) | PID reacts strongly → faster overall → **shutters MORE violently** |

Kp just scales the magnitude of the reaction. **The noise in the measurement is the root problem.** No Kp value can fix it because the input data itself is garbage at 1kHz with this encoder resolution.

### The Solution: Low-Pass Filter on Speed Measurement

Your code already has a filter in [velocity_controller.cpp](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/velocity_controller.cpp):

```cpp
static LowPassFilter left_speed_filter(0.05f);   // ← alpha = 0.05
static LowPassFilter right_speed_filter(0.05f);
```

**But `alpha = 0.05` is far too aggressive** (too smoothing). This means:
- New value contributes only 5% → the filter takes ~20 ticks to respond
- The PID sees a sluggish, delayed measurement → it overcompensates → **more oscillation**

And conversely, if you remove the filter entirely, the PID sees the raw 0/256 noise.

> [!IMPORTANT]
> **The fix:** Change alpha to `0.15` – `0.25`. This provides enough smoothing to remove the quantization noise, while still being responsive enough for the PID to work.
>
> ```cpp
> static LowPassFilter left_speed_filter(0.20f);   // ← CHANGE
> static LowPassFilter right_speed_filter(0.20f);
> ```

### Additional Fix: The Feedforward Is Doing Most of the Work Wrong

In [speed_controller.cpp](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/speed_controller.cpp#L20):

```cpp
#define FEEDFORWARD_KV 5.0f   // Assumes max 800mm/s at PWM 4200

float left_ff = target_left_speed_mm_s * FEEDFORWARD_KV;
// For target=150: left_ff = 150 × 5.0 = 750 PWM
```

Then the PID **adds more** on top of that 750. When the measured speed bounces to 0, the PID adds another huge chunk. Total PWM spikes to 2000+, then crashes back down.

> [!TIP]
> **The feedforward value `FEEDFORWARD_KV = 5.0` must be calibrated to YOUR robot.** Run the motors at several known PWM values, measure actual speed, and compute `KV = PWM / speed`. See [Section 5](#5-calibration-guide) for the exact procedure.

### Quick Summary of Fixes Needed

| Fix | What to Change | Why |
|-----|---------------|-----|
| ① Filter alpha | `0.05f` → `0.20f` | Smooth out encoder quantization noise |
| ② Speed PID Kp | `2.0f` → `1.0f` | Less aggressive reaction to noise |
| ③ Speed PID Ki | `1.0f` → `0.5f` | Slower integral buildup, less windup |
| ④ Speed PID Kd | `0.0f` → `0.01f` | Dampen oscillation |
| ⑤ Feedforward KV | Calibrate | Must match YOUR motor's actual response |

---

## 2. Why Multiple PID Values

Your project has **4 separate PID controllers**, each controlling a different thing. They are NOT duplicates — they form a **cascade (layered) control system:**

```
┌──────────────────────────────────────────────────────────────────┐
│                     CONTROL HIERARCHY                            │
│                                                                  │
│  Layer 4: Cell Controller (How far to travel)                    │
│       ↓ outputs: target_velocity (mm/s)                          │
│  Layer 3: Heading Controller (Which direction to face)           │
│       ↓ outputs: angular_correction (rad/s)                      │
│  Layer 2: Velocity Controller (Mix linear + angular → wheels)    │
│       ↓ outputs: target_left_speed, target_right_speed (mm/s)    │
│  Layer 1: Speed Controller (Make each wheel hit its target)      │
│       ↓ outputs: PWM to motor driver                             │
│                                                                  │
│  ╔═══════════════════════════════╗                                │
│  ║        PHYSICAL MOTORS        ║                                │
│  ╚═══════════════════════════════╝                                │
└──────────────────────────────────────────────────────────────────┘
```

### PID #1: Speed Controller PID (Left + Right = 2 PID objects)

**File:** [speed_controller.cpp](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/speed_controller.cpp)

| Parameter | Value | Purpose |
|-----------|:---:|---------|
| `SPEED_KP` | 2.0 | React to speed error |
| `SPEED_KI` | 1.0 | Eliminate steady-state error over time |
| `SPEED_KD` | 0.0 | Dampen oscillations (currently disabled) |
| Output range | -4199 to +4199 | PWM duty cycle |

**What it controls:** Each wheel's actual speed (mm/s) → matches a target speed  
**Input:** `target_speed` from velocity controller, `measured_speed` from encoders  
**Output:** PWM value sent to motor driver  
**Analogy:** Like a cruise control for each wheel individually

### PID #2: Heading Controller PID

**File:** [heading_controller.cpp](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/heading_controller.cpp)

| Parameter | Value | Purpose |
|-----------|:---:|---------|
| `HEADING_KP` | 1.0 | React to heading error |
| `HEADING_KI` | 0.0 | No integral (heading shouldn't accumulate) |
| `HEADING_KD` | 0.0 | No damping yet |
| Output range | -5.0 to +5.0 | Angular velocity in rad/s |

**What it controls:** Robot's facing direction (degrees)  
**Input:** `target_heading` (e.g., 0° for straight), `current_heading` from IMU+encoders  
**Output:** Angular velocity correction (ω in rad/s) — "turn left/right this fast"  
**Analogy:** Like a steering wheel controller — keeps the robot pointed in the right direction

### PID #3: Wall Follower PID (Future — Not Active Yet)

**File:** [wall_follower.cpp](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/wall_follower.cpp)

**What it will control:** Lateral distance from side walls  
**Input:** Side ToF sensor readings (distance to left/right wall)  
**Output:** Additional angular correction to keep robot centered  
**Analogy:** Like lane-keep assist in a car

### PID #4: Turn Controller PID (Future — Not Active Yet)

**File:** [turn_controller.cpp](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/turn_controller.cpp)

**What it will control:** In-place rotation (90°, 180° turns)  
**Input:** Target angle, current fused heading  
**Output:** Motor commands for spinning in place  

### PID in Testing Code (For Reference)

**File:** [Motors_Motion_Control_With_PID.ino](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Testing Codes/Motors_Motion_Control_With_PID/Motors_Motion_Control_With_PID.ino)

| Parameter | Value | Purpose |
|-----------|:---:|---------|
| `kp` | 1.8 | |
| `ki` | 0.8 | |
| `kd` | 0.02 | |

> [!NOTE]
> This test sketch runs PID at **10ms intervals (100Hz)**, not 1ms (1kHz). At 100Hz, you get ~5.86 encoder counts per sample instead of 0.586. **That's 10× more resolution!** This is why the test sketch works smoother than the Full_Code.
>
> **Critical difference:** The test sketch also uses `leftPWM += calculatePID(...)` (incremental accumulation), while the Full_Code uses `feedforward + PID output` (absolute). These are fundamentally different control strategies.

### Why Each PID Has Different Values

| PID | Controls | Speed of Response | Typical Kp Range |
|-----|---------|:---:|:---:|
| Speed | Wheel RPM → PWM | Very fast (ms) | 0.5–3.0 |
| Heading | Direction → ω correction | Medium (10ms) | 0.5–2.0 |
| Wall Follow | Lateral position → ω | Slow (50ms) | 0.1–1.0 |
| Turn | Rotation angle → motor | Medium (10ms) | 0.5–2.0 |

Each PID operates on **different physical quantities** with different units, different scales, and different response times. You can't use the same values for all of them.

---

## 3. Every Phase 5 File Explained

### The Control Stack (What's Active in Phase 5)

#### 3.1 [`pid.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/pid.cpp) + [`pid.h`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/pid.h)

**Purpose:** Generic reusable PID controller class — the "engine" that all controllers share.

**How it works:**
```
error = setpoint - measurement
P = Kp × error                              ← react to current error
I = I + (Ki × error × dt)                   ← accumulate past errors
D = Kd × (measurement_change) / dt          ← predict future error

output = P + I - D    (clamped to [out_min, out_max])
```

**Key design decisions:**
- **Derivative on measurement** (not on error): Prevents "derivative kick" when target changes suddenly. If target jumps from 0 to 150, the error spikes, but measurement changes smoothly.
- **Anti-windup on integral**: The integral is clamped to `[out_min, out_max]` so it doesn't accumulate to infinity when the motor is saturated.

---

#### 3.2 [`speed_controller.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/speed_controller.cpp) + [`speed_controller.h`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/speed_controller.h)

**Purpose:** Makes each wheel spin at the exact requested speed (mm/s).

**Data flow:**
```
target_left_speed ──┐
                    ├─→ LEFT PID ──→ left_pwm  ──┐
measured_left_speed─┘                             │
                                                  ├─→ motor_set_both(left, right)
target_right_speed ──┐                            │
                     ├─→ RIGHT PID ──→ right_pwm ─┘
measured_right_speed─┘
```

**Why feedforward + PID:**
- **Feedforward** (`target × KV`): Gives the motor approximately the right PWM immediately, without waiting for PID to ramp up. Like setting the gas pedal to roughly the right position.
- **PID**: Fine-tunes the feedforward to correct for battery voltage changes, friction, load differences, etc.

**Current problem:** Feedforward KV=5.0 was estimated, not measured. If it's wrong, the PID has to work too hard, causing oscillation.

---

#### 3.3 [`heading_controller.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/heading_controller.cpp) + [`heading_controller.h`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/heading_controller.h)

**Purpose:** Keeps the robot pointed in the target direction. Outputs an angular velocity correction.

**How it works:**
```
target_heading = 0°  (for driving straight)
current_heading = fusion_get_heading()   (from gyro + encoders)

error = target - current = 0° - 2.3° = -2.3°
    ↓ (wrap to ±180°)
PID output = ω = -2.3° × Kp = -2.3 rad/s correction
    ↓
This ω goes to velocity_controller which:
  left_speed  = 150 - (ω × wheelbase/2)   ← slows left wheel
  right_speed = 150 + (ω × wheelbase/2)   ← speeds up right wheel
    ↓
Robot turns back toward 0°
```

**The angle wrapping** (lines 28-31) is critical: without it, going from 359° to 1° would look like a 358° error instead of a 2° error.

---

#### 3.4 [`velocity_controller.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/velocity_controller.cpp) + [`velocity_controller.h`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/velocity_controller.h)

**Purpose:** Converts the "unicycle model" commands `(forward_speed, turn_rate)` into individual wheel speeds `(left_speed, right_speed)`.

**The differential drive equations:**
```
w_term = ω × (WHEEL_BASE_MM / 2)

left_wheel_speed  = linear_velocity - w_term
right_wheel_speed = linear_velocity + w_term
```

**Example: Driving straight at 150 mm/s, turning slightly right (ω = 0.5 rad/s):**
```
w_term = 0.5 × (127.1 / 2) = 31.8 mm/s

left  = 150 - 31.8 = 118.2 mm/s  (slower)
right = 150 + 31.8 = 181.8 mm/s  (faster)
```
→ Right wheel faster → robot turns left to correct.

**This file also reads encoder speeds and applies the low-pass filter** (the one with `alpha = 0.05` that needs fixing).

---

### The Sensor/Fusion Stack (What Feeds Data Into Control)

#### 3.5 [`sensor_fusion.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/sensors/sensor_fusion.cpp)

**Purpose:** Combines encoder data and IMU data into one unified "where am I?" estimate.

**What it does every tick:**
1. `odometry_update()` → update encoder-based position and rotation
2. `mpu6050_read_scaled()` → get gyro rotation rate (°/s)
3. `heading_estimator_update()` → fuse gyro + encoder into heading
4. `position_estimator_update()` → optionally correct position with wall sensors

---

#### 3.6 [`heading_estimator.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/localization/heading_estimator.cpp)

**Purpose:** Complementary filter — fuses gyro and encoder heading.

**The complementary filter formula:**
```
heading = α × (heading + gyro_delta) + (1-α) × encoder_heading
```
Where `α = 0.98`:
- **98% gyro:** Accurate short-term (not affected by wheel slip), but drifts over time
- **2% encoder:** Accurate long-term (no drift), but noisy during turns (wheel slip)

**The deadband** (line 25): If the robot isn't moving (encoder says 0 rotation AND gyro < 1°/s), ignore the gyro to prevent drift from noise while stationary.

---

#### 3.7 [`odometry.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/localization/odometry.cpp)

**Purpose:** Pure encoder-based position tracking (dead reckoning).

**The math:**
```
left_mm  = left_encoder_delta × MM_PER_COUNT
right_mm = right_encoder_delta × MM_PER_COUNT

d_center = (left_mm + right_mm) / 2        ← forward distance
d_theta  = (right_mm - left_mm) / WHEEL_BASE_MM   ← rotation

x += d_center × cos(theta)
y += d_center × sin(theta)
theta += d_theta
```

---

#### 3.8 [`position_estimator.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/localization/position_estimator.cpp)

**Purpose:** Combines odometry position with fused heading, and optionally corrects position using side wall sensors.

**Currently:** The wall correction code is **disabled** (commented out) because it would cause problems on a flat floor with no maze walls. It will be re-enabled in Phase 6 (Stage C) when testing inside an actual maze.

---

### The Main Sketch

#### 3.9 [`Micromouse.ino`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/Micromouse.ino) — Phase 5 Block (Lines 604–688)

**What the Phase 5 test does:**
```
┌─────────────────────────────────────┐
│         Phase 5 Test Loop           │
│                                     │
│  Sensor Update (10ms = 100Hz)       │
│  ├── Read ToF sensors              │
│  └── Read IMU                      │
│                                     │
│  Fusion Update (10ms = 100Hz)       │
│  ├── Update odometry               │
│  └── Fuse heading                  │
│                                     │
│  Motion Control (1ms = 1kHz)        │
│  ├── Heading PID → ω               │
│  └── Velocity Controller → PWM     │
│                                     │
│  OLED Display (100ms = 10Hz)        │
│  └── Show state, Kp, heading       │
│                                     │
│  Buttons:                           │
│  ├── START: Toggle drive/stop       │
│  └── MODE: Cycle Kp value          │
└─────────────────────────────────────┘
```

**The Kp tuning feature** (lines 639-648): MODE button increments `live_kp` by 0.2 each press, wrapping around at 3.0. This lets you tune the heading PID on the fly without re-flashing. But **this only tunes the heading Kp** — the speed PID gains are hardcoded.

---

### Config Files

#### 3.10 [`robot_config.h`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/config/robot_config.h)

All physical constants that define YOUR specific robot:

| Constant | Value | What It Means |
|----------|:---:|---|
| `GEAR_RATIO` | 18.85 | Motor shaft turns 18.85× for one wheel turn |
| `ENCODER_PPR` | 7 | 7 raw pulses per motor shaft revolution |
| `ENCODER_QUADRATURE` | 4 | Hardware quadrature decoding multiplier |
| `ENCODER_CPR` | 527.8 | Total counts per wheel revolution |
| `WHEEL_DIAMETER_MM` | 43.0 | Wheel outer diameter |
| `WHEEL_CIRCUMFERENCE_MM` | 135.09 | Distance per wheel revolution |
| `MM_PER_COUNT` | 0.2559 | Distance per encoder count |
| `WHEEL_BASE_MM` | 127.1 | Distance between wheel centers (**needs recalibration**) |
| `PWM_MAX` | 4199 | Maximum PWM duty cycle |
| `CONTROL_LOOP_FREQ_HZ` | 1000 | Control loop runs at 1kHz |
| `CONTROL_LOOP_DT_S` | 0.001 | 1ms per control tick |

---

## 4. Future Phase Files

### Phase 5 Stage B Files (Turn Control)

#### 4.1 [`turn_controller.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/turn_controller.cpp)

**Current status:** Skeleton with TODO comments  
**What it will do:** Execute 90° and 180° in-place turns

**How it will work:**
```
turn_start_inplace(TURN_LEFT_90):
  1. Record start heading (e.g., 0°)
  2. Set target heading (90°)
  3. Command: left motor backward, right motor forward
  4. PID loop: error = target_heading - current_heading
  5. When |error| < 2° AND angular_rate < 5°/s → done
```

#### 4.2 [`trajectory_controller.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/trajectory_controller.cpp)

**Current status:** Skeleton  
**What it will do:** Coordinate between straight motion and turns — decides what the robot should be doing right now and passes targets down to heading/velocity controllers.

---

### Phase 5 Stage A Files (Cell-by-Cell Motion)

#### 4.3 [`cell_controller.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/cell_controller.cpp)

**Current status:** Skeleton  
**What it will do:** Drive exactly one maze cell (180mm)

```
cell_start_move(1):
  distance = 1 × 180mm = 180mm
  Start motion profile: accelerate → cruise → decelerate → stop
  Monitor encoder distance until complete
```

#### 4.4 [`straight_motion.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/motion/straight_motion.cpp)

**Current status:** Skeleton  
**What it will do:** Execute a straight-line motion using a velocity profile

---

### Phase 6 Files (Maze Intelligence)

#### 4.5 [`motion_controller.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/motion_controller.cpp)

**Current status:** Skeleton — all functions have TODO  
**What it will do:** The **master orchestrator**. Called by the 1kHz timer interrupt. Runs the complete control chain:

```
motion_controller_update():
  1. fusion_update(dt)           ← "Where am I now?"
  2. trajectory_controller()     ← "What should I be doing?"
  3. heading_controller()        ← "Am I pointed right?"
  4. wall_follower()             ← "Am I centered?"
  5. velocity_controller()       ← "Set wheel speeds"
  6. speed_controller()          ← "Generate PWM"
```

> [!NOTE]
> Currently in Phase 5, the main sketch (`Micromouse.ino`) does this chain **manually** in the loop. Once `motion_controller.cpp` is implemented, it will be called from a timer interrupt instead, making it truly real-time.

#### 4.6 [`wall_follower.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/wall_follower.cpp)

**Current status:** Skeleton  
**What it will do:** While driving straight in a corridor, use side ToF sensors to keep the robot centered between walls.

```
lateral_error = (right_wall_distance - left_wall_distance) / 2
PD correction → adjusts angular velocity ω
```

#### 4.7 [`robot_state_machine.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/robot/robot_state_machine.cpp)

**Current status:** Basic skeleton with state switch  
**What it will do:** Top-level decision maker:

```
STATE_BOOT → STATE_IDLE → STATE_EXPLORING → STATE_RETURNING → STATE_FAST_RUN
     │
     ↓
  Wait for button → Start exploring maze
  At each cell: read walls, update maze map, decide next direction
  When goal found: return to start
  On second button press: fast run (optimized path)
```

#### 4.8 Maze Algorithm Files

| File | Purpose |
|------|---------|
| [`flood_fill.c`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/maze/flood_fill.c) | Classic flood-fill algorithm — assigns distance values to every cell, finds path to center |
| [`dijkstra_weighted.c`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/maze/dijkstra_weighted.c) | Weighted shortest path — accounts for turn cost, used for fast-run optimization |
| [`solver.c`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/maze/solver.c) | High-level maze solver — decides explore vs return, manages solver state |
| [`maze_explorer.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/maze/maze_explorer.cpp) | Exploration logic — decides which unvisited cells to explore next |
| [`maze.h`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/maze/maze.h) | 16×16 maze data structure — stores wall info for each cell |
| [`path_smoother.c`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/maze/path_smoother.c) | Converts cell-by-cell path into smooth curves for fast-run |

#### 4.9 Advanced Motion Files (Fast-Run Phase)

| File | Purpose |
|------|---------|
| [`motion_profile.c`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/motion/motion_profile.c) | Trapezoidal & S-curve velocity profiles — controls acceleration/deceleration |
| [`arc_motion.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/motion/arc_motion.cpp) | Circular arc turns — robot turns while moving forward (faster than stopping) |
| [`rolling_turn.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/motion/rolling_turn.cpp) | Rolling turn coordination — blends straight segments with arc segments |
| [`s_curve.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/motion/s_curve.cpp) | S-curve math — jerk-limited acceleration for ultra-smooth motion |
| [`look_ahead.cpp`](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/motion/look_ahead.cpp) | Plans ahead — ensures robot decelerates in time before a turn |

---

## 5. Calibration Guide

### Step-by-Step: Get Phase 5 Running Smooth

> [!IMPORTANT]
> Follow these steps **in exact order**. Do NOT skip steps. Each step depends on the previous one being correct.

---

### Step 1: Calibrate Feedforward (KV)

**What you need:** Serial Monitor, the robot on the floor

**Procedure:**
1. Temporarily bypass PID. Set motors to known PWM values and measure actual speed:

```cpp
// In Phase 5 loop, temporarily replace the drive code with:
if (p5_state == 1) {
    motor_set_both(500, 500);  // Fixed PWM, no PID
}
```

2. Read encoder speed from Serial output
3. Repeat for PWM = 500, 1000, 1500, 2000, 2500

4. Fill this table:

| PWM | Measured Speed (mm/s) | KV = PWM / Speed |
|:---:|:---:|:---:|
| 500 | ??? | ??? |
| 1000 | ??? | ??? |
| 1500 | ??? | ??? |
| 2000 | ??? | ??? |
| 2500 | ??? | ??? |

5. **Average the KV values** → that's your `FEEDFORWARD_KV`

6. Update in [speed_controller.cpp](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/speed_controller.cpp):
```cpp
#define FEEDFORWARD_KV  [your_average]
```

---

### Step 2: Fix the Low-Pass Filter

In [velocity_controller.cpp](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/velocity_controller.cpp), change:

```diff
-static LowPassFilter left_speed_filter(0.05f);
-static LowPassFilter right_speed_filter(0.05f);
+static LowPassFilter left_speed_filter(0.20f);
+static LowPassFilter right_speed_filter(0.20f);
```

**Why 0.20?** This means each new reading contributes 20% to the average. At 1kHz, the filter has a time constant of ~5ms, which is fast enough for control but smooth enough to remove the 0/1-count quantization noise.

---

### Step 3: Tune Speed PID (Start Conservative)

In [speed_controller.cpp](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/control/speed_controller.cpp), start with very conservative values:

```cpp
#define SPEED_KP 0.5f    // Start low
#define SPEED_KI 0.3f    // Start low
#define SPEED_KD 0.01f   // Small damping
```

**Tuning procedure:**
1. Set target speed to 150 mm/s
2. Press START → watch Serial plotter
3. **If oscillating** (speed bounces above and below target): reduce Kp
4. **If too slow to reach target** (takes > 1 second): increase Kp slightly
5. **If reaches target but sits 10 mm/s below**: increase Ki slightly
6. **If overshoots and oscillates before settling**: increase Kd slightly

```
Good PID response:           Bad PID response (shuttering):
                              
Speed ▲                       Speed ▲
      │    ┌──────────              │   /\/\/\/\/\/\/\/\
      │   /                         │  /
      │  /                          │ /
      │ /                           │/
──────┴──────────▶ Time      ──────┴──────────────────▶ Time
      smooth ramp up               violent oscillation
```

---

### Step 4: Add Serial Logging for Tuning

Add this to the Phase 5 OLED print section (inside the `millis() >= 100` block) to see PID behavior in Serial Plotter:

```cpp
Serial.print("Target:");
Serial.print(target_v);
Serial.print(",Heading_Error:");
Serial.print(target_h - fusion_get_heading());
Serial.print(",State:");
Serial.println(p5_state);
```

Open Arduino Serial Plotter (not Monitor) to see the graphs in real-time.

---

### Step 5: Calibrate Wheel Base

After the speed PID is smooth, calibrate `WHEEL_BASE_MM` for accurate turning:

1. Mark the robot's position on the floor with tape
2. Physically rotate the robot exactly 360° by hand (use a protractor or compass marks)
3. Read the heading on the OLED display
4. **If OLED shows more than 360°** (e.g., 380°): Your `WHEEL_BASE_MM` is too small → increase it
5. **If OLED shows less than 360°** (e.g., 340°): Your `WHEEL_BASE_MM` is too large → decrease it

**Formula:**
```
corrected_WHEEL_BASE = current_WHEEL_BASE × (displayed_angle / 360.0)
```

Example: If current WHEEL_BASE = 127.1 and display shows 380°:
```
corrected = 127.1 × (380 / 360) = 134.1 mm
```

Update in [robot_config.h](file:///c:/Users/KM Computers/OneDrive/Desktop/Projects/Maze-Runner/Full_Code/Micromouse/src/config/robot_config.h):
```cpp
#define WHEEL_BASE_MM  [your_corrected_value]
```

---

### Step 6: Tune Heading PID

With speed PID working smoothly and wheelbase calibrated:

1. Set heading Kp = 0.5, Ki = 0.0, Kd = 0.0
2. Drive straight → does it drift?
3. **If drifts a lot**: increase Kp to 1.0
4. **If oscillates (weaves left-right)**: decrease Kp, add Kd = 0.05
5. **If drifts slowly one way**: add small Ki = 0.01

---

### Summary: Recommended Starting Values

```cpp
// speed_controller.cpp
#define SPEED_KP 0.8f
#define SPEED_KI 0.4f
#define SPEED_KD 0.01f
#define FEEDFORWARD_KV [calibrated value from Step 1]

// heading_controller.cpp
#define HEADING_KP 0.8f
#define HEADING_KI 0.0f
#define HEADING_KD 0.05f

// velocity_controller.cpp
LowPassFilter left_speed_filter(0.20f);
LowPassFilter right_speed_filter(0.20f);
```

---

## Complete Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                        MICROMOUSE CONTROL FLOW                       │
│                                                                      │
│  ┌──────────────┐                                                    │
│  │   BUTTONS    │──── START: toggle drive ── MODE: tune Kp           │
│  └──────────────┘                                                    │
│         │                                                            │
│  ┌──────▼──────────────────────────────────────────────────────┐     │
│  │  Main Loop (Micromouse.ino)                                  │     │
│  │  ┌──────────────────────────────────────────────────────┐   │     │
│  │  │ Sensor Update (100 Hz)                                │   │     │
│  │  │  sensor_manager_update() → reads ToF + IMU           │   │     │
│  │  └──────────────────────────────────────────────────────┘   │     │
│  │  ┌──────────────────────────────────────────────────────┐   │     │
│  │  │ Fusion Update (100 Hz)                                │   │     │
│  │  │  odometry_update() → encoder position/heading        │   │     │
│  │  │  heading_estimator_update() → fuse gyro + encoder    │   │     │
│  │  │  position_estimator_update() → best X, Y, heading    │   │     │
│  │  └──────────────────────────────────────────────────────┘   │     │
│  │  ┌──────────────────────────────────────────────────────┐   │     │
│  │  │ Motion Control (1000 Hz / 1kHz)                       │   │     │
│  │  │                                                       │   │     │
│  │  │  heading_controller_update()                          │   │     │
│  │  │       target_heading vs current_heading               │   │     │
│  │  │       → angular_velocity ω                            │   │     │
│  │  │              │                                        │   │     │
│  │  │  velocity_controller_update(linear_v, ω)              │   │     │
│  │  │       ├── differential drive mixing                   │   │     │
│  │  │       ├── read encoder deltas                         │   │     │
│  │  │       ├── low-pass filter speeds                      │   │     │
│  │  │       └── speed_controller_update()                   │   │     │
│  │  │              ├── left PID  → left PWM                 │   │     │
│  │  │              ├── right PID → right PWM                │   │     │
│  │  │              ├── add feedforward                      │   │     │
│  │  │              └── motor_set_both(left, right)          │   │     │
│  │  └──────────────────────────────────────────────────────┘   │     │
│  │  ┌──────────────────────────────────────────────────────┐   │     │
│  │  │ OLED Display (10 Hz)                                  │   │     │
│  │  │  Show state, Kp, heading                             │   │     │
│  │  └──────────────────────────────────────────────────────┘   │     │
│  └─────────────────────────────────────────────────────────────┘     │
│                                                                      │
│  ┌──────────────────────────────────────────────────────────────┐    │
│  │  FUTURE (Phase 6+)                                           │    │
│  │  cell_controller → trajectory_controller → motion_controller │    │
│  │  robot_state_machine → maze solver → flood_fill/dijkstra     │    │
│  │  wall_follower → rolling_turn → arc_motion → s_curve         │    │
│  └──────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────┘
```

---

> [!TIP]
> **The #1 thing to do right now:** Fix the low-pass filter alpha from 0.05 to 0.20 in `velocity_controller.cpp`. This single change will have the biggest impact on reducing shuttering, before you even touch PID gains.
