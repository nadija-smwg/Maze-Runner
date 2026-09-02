# Current Progress Report & Roadmap
**Last Updated: 2026-09-01 — Session 3 (Sensor Filtering + Encoder Velocity Complete)**

---

## 1. Overall Status

| Stage | Description | Status | Notes |
|-------|-------------|--------|-------|
| 1 | Hardware Bring-up | ✅ **DONE** | STM32, PWM, encoders, I2C, OLED |
| 2 | Sensor Filtering | ✅ **DONE** | ToF 5-stage pipeline, MPU6050 EMA + drift corrector |
| 3 | Motor/Encoder Foundation | 🔄 **75%** | Velocity LPF done; dead-zone + wheel-base pending |
| 4 | PI Velocity Controller | 🔄 **15%** | KFF=8.4 measured; Kp/Ki tuning pending |
| 5 | Heading Controller | ⬜ Not started | Sensor fusion wired, needs integration in control loop |
| 6 | Motion Primitives | ⬜ Not started | forward_mm, turn_deg, stop |
| 7 | Wall Following | ⬜ Not started | ToF lateral PID |
| 8 | Maze Solving | ⬜ Not started | Flood fill, 16×16 map |
| 9 | Speed Runs | ⬜ Not started | Trajectory optimization |
| 10 | Competition Optimization | ⬜ Not started | Final tuning |

---

## 2. Completed Milestones

### Hardware Foundation (Stage 1)
- [x] STM32F401CCU6 running reliably at 84 MHz
- [x] Register-level TIM1 PWM at 20 kHz
- [x] Hardware quadrature encoders (TIM2/TIM3) with 1820 CPR verified
- [x] MPU6050 fully calibrated with EMA filter and stationary drift correction
- [x] All 5 VL53L0X sensors initialized with ±1 mm noise (better than target)
- [x] Motor driver abstraction (TB6612FNG) with dead-zone compensation API
- [x] Modular architecture: `src/sensors/`, `src/hardware/`, `src/control/`

### Sensor Filtering (Stage 2) — All Hardware Verified
- [x] ToF: 5-stage pipeline (validity → offset → median-3 → jump reject → EMA)
- [x] ToF: Hysteresis-based wall detection (no flickering)
- [x] MPU6050: EMA filter with STATIONARY_GZ_THRESH = 3.5 °/s
- [x] MPU6050: Drift corrector converges in 1.5 sec at 1 kHz
- [x] Competition boot: `calibrate_with_warmup(90)` with OLED countdown
- [x] Sensor fusion: 98/2 complementary filter (gyro/encoder)

### Motor/Encoder Foundation (Stage 3 — 75% complete)
- [x] Encoder velocity LPF (alpha=0.25, at 20 Hz → smooth, no spikes)
- [x] Distance accumulation verified (89 mm per 500ms @ PWM 1500 ✓)
- [x] Motor characterization started: PWM 1500 → L=178 mm/s, R=180 mm/s
- [x] KFF = 8.4 measured and set in `speed_controller.cpp`
- [ ] Dead-zone measurement (Phase 2 States 6 + 7 — code ready, not yet measured)
- [ ] WHEEL_BASE_MM physical measurement
- [ ] Full characterization table (6 PWM points)

---

## 3. Immediate Goal: Complete Stage 3 → Begin Stage 4

### Remaining Stage 3 Tasks (do in order):

**Task A — Dead-Zone Measurement (30 minutes):**
```
1. Phase 2, BTN_START × 6 → State 6 (LEFT sweep)
2. Watch Serial for first '[DZ] LEFT  PWM:XXX  <<< MOVING'
3. Record PWM → update LEFT_MOTOR_DEAD_PWM in robot_config.h
4. Press BTN_START → State 7 (RIGHT sweep), repeat
5. Record PWM → update RIGHT_MOTOR_DEAD_PWM in robot_config.h
```

**Task B — Wheel Base Measurement (10 minutes):**
```
1. Place robot on paper, mark centre of both wheel contact points
2. Measure with calipers: center-to-center distance
3. Update WHEEL_BASE_MM in robot_config.h
4. Note: 1mm error in wheel base ≈ 1.2° error per 90° turn
```

**Task C — Full Characterization (20 minutes):**
```
In Phase 2 State 1 (FORWARD), change PWM to each value:
  motor_forward(500);  → record L_filt, R_filt
  motor_forward(800);
  motor_forward(1000);
  motor_forward(2000);
  motor_forward(2500);
  motor_forward(3000);
Fill in robot_config.h characterization table.
```

### Stage 4 — PI Controller (after above):
- Run Phase 4 step-response test (velocity_controller_update(300.0f, 0.0f))
- Tune Kp: target ±10 mm/s of setpoint without oscillation
- Tune Ki: zero steady-state error
- Verify straight-line: ±2 cm over 600 mm

---

## 4. robot_config.h — Parameter Status

| Parameter | Value | Status | How to Set |
|-----------|-------|--------|------------|
| `GEAR_RATIO` | 65.0 | ✅ Verified | N20 spec |
| `ENCODER_PPR` | 7.0 | ✅ Verified | N20 spec |
| `ENCODER_CPR` | 1820 | ✅ Verified | Phase 2 data consistent |
| `WHEEL_DIAMETER_MM` | 34.0 | ✅ Consistent | Phase 2 data consistent |
| `LEFT_ENCODER_CPR` | 1820 | ✅ OK | Phase 2 data consistent |
| `RIGHT_ENCODER_CPR` | 1820 | ✅ OK | Phase 2 data consistent |
| `LEFT_WHEEL_DIAMETER_MM` | 34.0 | ✅ OK | Phase 2 data consistent |
| `RIGHT_WHEEL_DIAMETER_MM` | 34.0 | ✅ OK | Phase 2 data consistent |
| `VELOCITY_LPF_ALPHA` | 0.25 | ✅ Tested | Phase 2 smooth output |
| `LEFT_MOTOR_DEAD_PWM` | **0** | ⬜ TODO | Phase 2 State 6 sweep |
| `RIGHT_MOTOR_DEAD_PWM` | **0** | ⬜ TODO | Phase 2 State 7 sweep |
| `WHEEL_BASE_MM` | **75.0** | ⬜ TODO | Caliper measurement |
| `SENSOR_FRONT_OFFSET_MM` | **30.0** | ⬜ TODO | Ruler from axle to ToF |
| `PWM_MAX` | 4199 | ✅ Fixed | TIM1 ARR for 20 kHz |
| `CONTROL_LOOP_FREQ_HZ` | 1000 | ✅ Fixed | 1 kHz timer |
| `BATTERY_FULL_MV` | 8400 | ✅ Fixed | 2S LiPo spec |
| `BATTERY_DIVIDER_RATIO` | **2.0** | ⬜ Verify | Measure with multimeter |

---

## 5. Future Stages

### Stage 5: Heading Controller
- Integrate `fusion_update()` into 1 kHz control loop callback
- Add heading correction term: `ω_correction = Kh × (target_heading - fusion_get_heading())`
- Heading Kp starting value: 2.0

### Stage 6: Motion Primitives
| Function | Description |
|----------|-------------|
| `motion_forward_mm(d)` | Drive d mm using encoder odometry + PI |
| `motion_turn_deg(a)` | Turn a degrees using gyro feedback |
| `motion_stop()` | Deceleration ramp to zero |
| `motion_align_to_wall()` | Square up using front sensors |
| `motion_center_in_cell()` | Lateral PID using L+R sensors |

### Stage 7: Wall Following
- Lateral error = `(dist_left - dist_right) / 2`
- Wall PID feeds into angular velocity command
- Target: ±3 mm lateral accuracy in 180 mm corridor

### Stage 8: Maze Solving
- 16×16 cell array (`uint8_t walls[16][16]`, `uint8_t dist[16][16]`)
- Flood fill from start → centre
- Exploration: navigate to lowest-cost unvisited cell
- Speed run: drive known path at maximum speed

---

## 6. Implementation Order (Updated)

```
DONE:
  [x] Motor driver
  [x] Encoder driver + velocity LPF
  [x] MPU6050 driver + EMA filter
  [x] ToF sensor driver + 5-stage filter
  [x] Sensor fusion (complementary filter)
  [x] Competition calibration (warm-up boot)
  [x] Motor characterization start (KFF=8.4)

CURRENT SPRINT:
  [ ] Dead-zone measurement → robot_config.h
  [ ] Wheel base measurement → robot_config.h
  [ ] Full characterization table
  [ ] PI speed controller tuning (Kp, Ki)
  [ ] Straight-line test

NEXT SPRINT:
  [ ] Heading controller
  [ ] motion_forward_mm()
  [ ] motion_turn_deg()
  [ ] Cell navigation test

LATER:
  [ ] Wall following PID
  [ ] Maze representation
  [ ] Flood fill
  [ ] Exploration
  [ ] Speed run
  [ ] Competition optimization
```

---

## 7. Known Issues & Notes

| Issue | Impact | Resolution |
|-------|--------|------------|
| MPU6050 cold-start bias drift ±2.3 °/s | GzFilt wrong until corrector converges | Fixed: warm-up + recalibrate; drift corrects in 1.5s at 1kHz |
| L_raw=0 in Phase 2 Serial | Display only, not real | delta consumed by velocity filter; L_filt is correct |
| Battery reads ~910 mV | Likely voltage divider ratio wrong | Measure actual bat voltage with multimeter, update BATTERY_DIVIDER_RATIO |
| WHEEL_BASE_MM not measured | Turns will be inaccurate | Measure with calipers before Stage 5 |

> [!IMPORTANT]
> Battery voltage reads ~910 mV in Serial but should be ~7400 mV for a 2S LiPo.
> This is a 8× error, suggesting BATTERY_DIVIDER_RATIO is ~0.25 instead of 2.0.
> Measure actual battery voltage and update the ratio before relying on battery monitoring.
