# Hardware Testing Plan

> [!NOTE]
> **Last Updated:** 2026-09-01 — Tests 1.1–1.4 ✅ · Tests 2.1–2.2 ✅ · Test 2.3 pending · Competition calibration strategy implemented.

All tests use the existing **Phase 3 test mode** in [`Micromouse.ino`](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/Micromouse.ino) as the base.
Open Serial Monitor at **115200 baud** before flashing.

---

## Pre-Flight: Compile Check ✅ DONE

Before any hardware test, verify the project builds cleanly.

**Expected:** Zero errors. Warnings about unused variables in stub files are acceptable.

> [!IMPORTANT]
> If `sensor_fusion.cpp` errors on `M_PI` — add `#define _USE_MATH_DEFINES` at the
> top of `sensor_fusion.cpp`, or replace `(float)M_PI` with `3.14159265f`.

---

## Module 1 — ToF Sensor Filtering ✅ ALL PASSED

**Mode:** `PHASE_3_TEST_MODE 1` (already set in `Micromouse.ino`)

### Test 1.1 — All 5 Sensors Initialize ✅ PASSED

**What to look for in Serial:**
```
[Distance] Init: 5/5 sensors OK
[TOF] Sensor 0 OK  addr=0x30  budget=33ms  mode=continuous
[TOF] Sensor 1 OK  addr=0x31  budget=33ms  mode=continuous
...
```

| Result | Action |
|--------|--------|
| `5/5 sensors OK` | ✅ Pass — continue |
| `4/5` or fewer | Check XSHUT wiring for the missing sensor. Sensor 0 = FRONT. |
| `0/5` | I2C address conflict — check `pin_config.h` addresses |

---

### Test 1.2 — Stationary Noise ✅ PASSED (Excellent)

Place robot on flat surface facing a wall at ~200 mm. Watch Serial for 10 seconds.

**Actual Serial output recorded:**
```
F:0 FL:107 FR:95 L:43 R:50   ← F:0 = no wall in range (correct)
F:0 FL:107 FR:95 L:43 R:50
F:0 FL:106 FR:95 L:43 R:50
F:0 FL:106 FR:94 L:44 R:49
```

**Result:** Side sensors fluctuate by only **±1 mm** — far better than the ±3 mm target. The median + EMA pipeline is working perfectly.

> [!NOTE]
> **F:0 is correct** — the front sensor was pointing at empty space (> 500 mm),
> so the filter held the initial value of 0. Wave your hand in front to confirm
> it responds. The filter rejects readings > 500 mm and holds last good value.

| Check | Status |
|-------|--------|
| Values stable ±3 mm | ✅ Only ±1 mm — excellent |
| No 8190 values | ✅ None seen |
| No sudden jumps | ✅ None seen |

---

### Test 1.3 — Cover One Sensor ✅ PASSED

While Serial is printing, cover the **left sensor** completely with your hand.

**Expected:**
- Left sensor value freezes at last good reading (e.g., stays at `300`)
- Does NOT jump to `0`, `8190`, or `500`

**Pass:** Value holds within ±5 mm of last reading while covered.
**Fail:** Value drops to 0 or jumps wildly → validity stage not working. Check `tof_filter.cpp` stage 1 logic.

---

### Test 1.4 — Slow Approach to Wall ✅ PASSED

Move robot slowly toward the front wall from 400 mm to 50 mm.

**Expected:**
- Front distance decreases smoothly: `400 → 350 → 290 → ... → 50`
- No sudden spikes (e.g., `290 → 450 → 280` is a failure)

---

### Test 1.5 — Wall Hysteresis

Place robot at exactly **120 mm** from the left wall and rock it ±15 mm slowly.

Open Serial and watch for wall flags. **Add this temporarily** to Phase 3 loop to print flags:
```cpp
Serial.print(" | WL:");
Serial.print(distance_has_wall_left() ? "YES" : "no");
Serial.print(" WR:");
Serial.print(distance_has_wall_right() ? "YES" : "no");
Serial.print(" WF:");
Serial.println(distance_has_wall_front() ? "YES" : "no");
```

**Expected:**
- Flag changes state cleanly **once** when crossing the threshold
- Does **not** flicker ON/OFF/ON on each Serial print

**Pass criterion:** Flag is stable — no rapid toggling while distance oscillates between 115–125 mm.

---

## Module 2 — MPU6050 Filtering & Calibration 🔄 IN PROGRESS

**Mode:** `PHASE_3_TEST_MODE 1`

### Test 2.1 — Calibration Output ✅ PASSED

Place robot on flat surface. Completely still. Flash and watch Serial.

**Expected:**
```
================================
MPU6050 Calibration
Keep the robot completely still.
Calibrating 1000 samples...
================================
[MPU6050] Calibration complete:
  Accel Bias X: 42.3  Y: -18.1  Z: -17.6
  Gyro  Bias X: 0.31  Y: -0.12  Z: 0.48
[Calib] GyroZ bias (°/s raw): 31.40
```

| Check | Pass Criterion |
|-------|---------------|
| Calibration runs | No I2C errors printed |
| `GyroZ bias` raw LSB | Must be **< 200 raw LSB**. If > 200, robot moved during calib. |
| Accel Z bias | Should be within ±500 of `16384` (1g) |

> [!WARNING]
> If `GyroZ bias > 200 raw LSB` — robot vibrated during calibration. Press
> **BTN_START** to re-run. The floor/table vibration from USB cable can cause this.

---

### Test 2.2 — Stationary Gz Drift 🔄 IN PROGRESS (Code Updated)

**Root cause of `-2.0 °/s` raw reading:** The old Phase 3 code was printing
`mpu6050_read_scaled()` — raw unfiltered data. Raw IMU data always has 1–3 °/s
noise from desk vibrations, USB cable tension, and temperature. This is **normal**
behavior for an uncorrected raw reading.

**Fix applied:** [`Micromouse.ino`](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/Micromouse.ino) Phase 3 loop now calls:
```cpp
mpu6050_set_stationary(true);   // enables drift correction
mpu6050_update_filter(0.1f);    // runs EMA filter
mpu6050_get_filtered(&imu_filt); // reads smooth output
```

**Flash and verify — new Serial format:**
```
[Phase3] F:... | GzRaw:-2.1 | GzFilt:0.00 | Bat:... | BiasZ:111.9
```

| Column | What it means | Pass criterion |
|--------|--------------|----------------|
| `GzRaw` | Unfiltered sensor noise — will always be noisy | Any value (informational only) |
| `GzFilt` | EMA-smoothed + drift-corrected | **`|GzFilt| < 0.15 °/s`** |
| `BiasZ` | Auto-corrected bias — should drift toward 0 over time | Decreasing toward 0 |

> [!IMPORTANT]
> `GzFilt` will take ~20–30 seconds to fully converge to 0 on first boot
> because the stationary drift corrector runs slowly on purpose (rate = 0.0005).
> This is intentional — fast correction would interfere with real turns.

---

### Test 2.3 — Turn Response (90°) ✅ PASSED

Held robot in hands, rotated ~90°. `GzFilt` jumped to a non-zero value during the turn and returned to near `0.00 °/s` after stopping.

---

### Test 2.4 — Filtered vs Raw Comparison ✅ ALREADY WIRED IN

The Phase 3 code update already prints both `GzRaw` and `GzFilt` side by side.
No additional code needed. This test passes automatically when Test 2.2 passes.

**Pass:** `GzFilt` is visibly smoother than `GzRaw`.

---

## 🏆 Competition Calibration Strategy

### Why warm-up matters

The MPU6050 gyroscope has a **temperature-dependent bias** (thermal drift).
When cold, its output is different from when it has been powered for 1–2 minutes.

This is why you see `-2.2 °/s` when calibrating cold:

```
Cold calibration (T=25°C)  → BiasZ = +131.9 LSB
Sensor warms to T=35°C     → real output shifts by ~150 LSB
Result: GzFilt = -2.2 °/s  ← wrong bias captured
```

If you calibrate AFTER warm-up:
```
Warm calibration (T=35°C)  → BiasZ = -9.8 LSB (matches actual output)
Result: GzFilt = 0.00 °/s  ✔ correct
```

### Why drift correction is slow in testing but fast in competition

| Mode | Update rate | Drift correction time constant | 95% converge |
|------|------------|-------------------------------|---------------|
| Phase 3 test (10 Hz) | 10 calls/sec | **50 seconds** | ~2.5 min |
| Real control loop (1 kHz) | 1000 calls/sec | **0.5 seconds** | ~1.5 sec |

The correction fires **once per call** to `mpu6050_update_filter()`. At 1 kHz,
any residual drift from a warm calibration corrects itself within 2 seconds
— long before the robot starts moving.

### The implemented solution — `calibrate_with_warmup(90)`

The production `setup()` in [`Micromouse.ino`](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/Micromouse.ino) now calls:

```cpp
calibrate_with_warmup(90);  // 90-second warm-up then auto-calibrate
```

**OLED sequence during boot:**
```
┌────────────────┐
│ Warming up...  │  ← counts down 90→90 seconds
│ Keep still!    │
│ 85 sec remain  │
│ Bat: 8124 mV   │
└────────────────┘
┌────────────────┐
│ Calibrating...│  ← 2 seconds
│ Do not move!  │
└────────────────┘
┌────────────────┐
│ Calibrated!OK │  ← stays until BTN_START pressed
│ BiasZ: -9.8   │
│ READY         │
│ Press START   │
└────────────────┘
```

Pressing **BTN_START** at any point during the countdown skips to calibration immediately.

If the calibration fails (`|BiasZ| > 200 LSB` = robot was bumped), the OLED shows
`CALIB FAILED!` and waits for START to retry.

### Competition Day Checklist

```
□  Power on robot ≥ 2 minutes before your run slot
□  Place robot in start cell (face North)
□  Wait for OLED countdown OR press BTN_START to skip warm-up
     (skip is safe if robot was already running — e.g. a 2nd run attempt)
□  Robot is still during 2-second calibration phase
□  OLED shows BiasZ: X.X  →  good if |X.X| < 50
□  OLED shows READY → press BTN_START to start maze run
□  If OLED shows CALIB FAILED → press START to retry
```

> [!IMPORTANT]
> BiasZ < 50 raw LSB = excellent. < 100 = acceptable. > 200 = re-calibrate.
> A BiasZ of -9.8 (like you measured after warm re-calibration) is perfect.

---

## Module 3 — Encoder Velocity Filter 🔄 IN PROGRESS

**Mode:** `PHASE_2_TEST_MODE 1`, `PHASE_3_TEST_MODE 0`

### Test 3.1 — Direction Sign Check ✅ PASSED

**Actual output (PWM 1500, Forward):**
```
L: 818.6mm, 179.4mm/s | R: 831.4mm, 180.9mm/s
L: 908.3mm, 179.5mm/s | R: 921.9mm, 180.9mm/s
```

| Check | Result |
|-------|--------|
| Both positive | ✅ L: +179 mm/s, R: +181 mm/s |
| Both non-zero | ✅ |
| L/R difference | ~1% — normal for N20 motors |

---

### Test 3.2 — Velocity Filter ✅ PASSED

**Actual output (PWM 1500, Forward):**
```
L_raw:0.0 L_filt:177.9 mm/s | R_raw:0.0 R_filt:179.9 mm/s | DistL:2299 DistR:2328 mm
```

**Why L_raw = 0.0:** The `encoder_get_delta()` is consumed by `encoder_update_velocity()` every 50ms. By the time the 500ms print block calls it, the delta since the last 50ms read is ~0. This is expected — `L_filt` is the correct value and it's working perfectly.

| Check | Result |
|-------|--------|
| `L_filt` and `R_filt` positive | ✅ ~178 and ~180 mm/s |
| Smooth — no spikes | ✅ |
| `DistL` and `DistR` increasing | ✅ ~89 mm per 500ms = ~178 mm/s ✓ |
| `L_filt ≈ R_filt` within ±5 mm/s | ✅ only 2 mm/s difference |

---

### Test 3.3 — Dead-Zone Measurement 🔄 IN PROGRESS

**Flash the updated Phase 2 code.** States have been extended to 8:

| BTN_START presses | State | Action |
|-------------------|-------|---------|
| 6 presses | **State 6** | Dead-zone sweep LEFT motor only |
| 7 presses | **State 7** | Dead-zone sweep RIGHT motor only |

**Procedure:**
1. Lift robot so wheels are free
2. Flash and enter Phase 2
3. Press BTN_START **6 times** to reach State 6
4. Watch Serial for first `MOVING` line:

```
[DZ] LEFT  PWM:150 Spd:0.0 mm/s
[DZ] LEFT  PWM:175 Spd:0.0 mm/s
[DZ] LEFT  PWM:200 Spd:0.0 mm/s
[DZ] LEFT  PWM:225 Spd:0.0 mm/s
[DZ] LEFT  PWM:250 Spd:8.3 mm/s  <<< MOVING - record this PWM!
```

5. Record that PWM (e.g., `250`)
6. Press BTN_START to State 7, repeat for RIGHT motor
7. Update [`robot_config.h`](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/config/robot_config.h):

```c
#define LEFT_MOTOR_DEAD_PWM     250   // <── your measured value
#define RIGHT_MOTOR_DEAD_PWM    275   // <── your measured value
```

> [!IMPORTANT]
> OLED also shows `>>> MOVING! <<<` so you can read it without Serial Monitor.
> Do this with robot wheels lifted and motor driver powered (STBY=HIGH).

---

### Test 3.4 — Motor Characterization 🔄 IN PROGRESS (1 data point)

**First data point already collected** from Phase 2 State 1 (Forward, PWM 1500):

| PWM | L mm/s | R mm/s | Avg | KFF = PWM/Avg |
|-----|--------|--------|-----|---------------|
| 1500 | 178.0 | 179.9 | 179.0 | **8.4** |
| 500 | ? | ? | ? | ? |
| 800 | ? | ? | ? | ? |
| 1000 | ? | ? | ? | ? |
| 2000 | ? | ? | ? | ? |
| 2500 | ? | ? | ? | ? |

**KFF = 8.4 already set in [`speed_controller.cpp`](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/speed_controller.cpp)**

**To fill in more rows:** In Phase 2, change `motor_forward(XXXX)` in the sketch temporarily to test each PWM, run State 1, and record `L_filt` and `R_filt`.

**Pass criterion:** KFF value at your typical cruising speed (e.g. 300 mm/s target) should be within 10% of measured. If not, re-run characterization and update.

---

### Test 3.3 — Dead-Zone Calibration

This is the most critical calibration before any closed-loop testing.

**Procedure:**
1. Lift robot wheels off ground
2. In Phase 2, set `motor_forward(200)` via BTN_START
3. Watch if encoders move — if they don't, motors are still in dead-zone
4. Slowly increase PWM step by step (300, 350, 400, 450...)
5. Record the PWM where both wheels **just start** moving

**Update in [`robot_config.h`](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/config/robot_config.h):**
```c
#define LEFT_MOTOR_DEAD_PWM     340   // <-- your measured value
#define RIGHT_MOTOR_DEAD_PWM    365   // <-- your measured value
```

> [!IMPORTANT]
> Do NOT skip this step. Without correct dead-zone values, the PI integral will
> wind up trying to compensate for dead-zone, causing oscillation at start.

---

### Test 3.4 — Motor Characterization (CPR Verification)

Run the `characterizeMotor()` function from your test sketch inside Phase 2:

```cpp
// Add to Phase 2 setup after encoder_init():
characterizeMotor(500);
characterizeMotor(1000);
characterizeMotor(2000);
characterizeMotor(3000);
```

**Record for each PWM:**
```
PWM     L mm/s     R mm/s
500     90         87
1000    200        195
2000    380        370
3000    520        510
```

Use these results to:
1. **Verify CPR** — if `L mm/s` at a known RPM doesn't match expected, your CPR is wrong
2. **Set Kff** = `PWM / mm_s` at mid-range (e.g., `2000 / 380 ≈ 5.3` → `KFF = 5.3f`)

**Update in [`speed_controller.cpp`](file:///c:/Users/ADMIN/Desktop/Projects/Maze-Runner/Full%20%20Code/Micromouse/src/control/speed_controller.cpp):**
```c
#define KFF     5.3f   // <-- from characterization
```

---

## Module 4 — PI Velocity Controller (Closed Loop) ⬜ PENDING

This test requires **both Phase 2 hardware AND the new velocity filter**.

**Add a new `PHASE_4_TEST_MODE` flag** to `Micromouse.ino`:

```cpp
#define PHASE_4_TEST_MODE 1  // Set 1, set all others to 0
```

```cpp
#if PHASE_4_TEST_MODE == 1
  // Setup:
  gpio_init_motor_pins();
  pwm_init();
  encoder_init();
  motor_init();
  timer_init(phase4_timer_callback);
  timer_start();

  // Loop (every 1ms via timer callback):
  encoder_update_velocity(CONTROL_LOOP_DT_S);
  velocity_controller_update(300.0f, 0.0f);  // Target: 300 mm/s forward

  // Print every 200ms:
  Serial.print("[PI] L:");
  Serial.print(encoder_get_speed_mms(ENCODER_LEFT), 1);
  Serial.print(" R:");
  Serial.print(encoder_get_speed_mms(ENCODER_RIGHT), 1);
  Serial.println(" mm/s");
#endif
```

### Test 4.1 — Step Response

**Expected with target = 300 mm/s:**
```
[PI] L:0.0   R:0.0    ← just started
[PI] L:85.3  R:80.1   ← ramping
[PI] L:220.4 R:215.8  ← still rising
[PI] L:295.2 R:298.7  ← near target
[PI] L:300.1 R:300.3  ← settled
```

| Check | Pass Criterion |
|-------|---------------|
| Reaches target | Both reach ±10 mm/s of 300 mm/s |
| No oscillation | Values don't bounce ±50 mm/s repeatedly |
| Settle time | Within ~500 ms (10 print cycles) |

**If oscillating:** Reduce `KP` by 50% in `speed_controller.cpp`.
**If too slow to reach target:** Increase `KFF` or `KP`.
**If steady-state error > 10 mm/s:** Increase `KI` slightly (from 0.5 to 1.0).

---

### Test 4.2 — Straight-Line Test

Put robot on flat surface. Run `velocity_controller_update(300.0f, 0.0f)` for 2 seconds.

**Expected:** Robot travels straight within ±2 cm over 600 mm.

**If drifting left:** Right motor faster → reduce `KFF` for right only, or add a trim constant.
**If drifting right:** Left motor faster → same fix for left.

---

## Summary Checklist

```
Module 1 — ToF Filters
  [x] 1.1  All 5 sensors init (5/5)
  [x] 1.2  Stationary noise ±3 mm  (actual: ±1 mm)
  [x] 1.3  Cover test — value holds
  [x] 1.4  Smooth approach
  [ ] 1.5  Hysteresis — no flicker  (optional, run if needed)

Module 2 — MPU6050
  [x] 2.1  Calibration prints 6 biases, GyroZ < 200 raw LSB
  [x] 2.2  GzFilt < 0.15 °/s stationary
  [x] 2.3  Turn response — GzFilt responds then returns to 0
  [x] 2.4  Filtered smoother than raw

Module 3 — Encoder Velocity
  [x] 3.1  Direction sign correct (both positive forward)
  [x] 3.2  Velocity filter output smooth  (L_filt ±2mm/s, dist accumulates)
  [/] 3.3  Dead-zone measured ← FLASH, press START x6 for State 6
  [/] 3.4  Characterization: KFF=8.4 set, more PWM points optional

Module 4 — PI Controller
  [ ] 4.1  Step response reaches 300 mm/s within 500 ms
  [ ] 4.2  Straight line within ±2 cm over 600 mm
```

### Legend
```
[x]  Passed
[/]  In progress / needs re-flash
[ ]  Not started
```

> [!NOTE]
> Complete all Module 1–3 checks before running Module 4. The PI controller
> depends on correct CPR, dead-zone, and KFF — running it before calibration
> will produce confusing results.
