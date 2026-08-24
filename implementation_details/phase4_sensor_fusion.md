# Phase 4: Sensor Fusion & Localization (REVISED)

> **Goal:** Track the exact physical location of the robot (X, Y, Heading) in the real world. This phase combines the mathematical precision of the wheel encoders with the drift-resistant stability of the MPU6050 gyroscope to create a reliable global pose.
>
> **Status:** ⚠️ PARTIALLY COMPLETE — X-distance works, angle is broken.

---

## 🔴 Issues Found During Testing

### Issue 1: Heading Estimator Bypassed (alpha = 1.0)
The complementary filter in `heading_estimator.cpp` has `alpha = 1.0f`, meaning it only trusts the gyro and completely ignores encoder heading. This was done intentionally to isolate the gyro during Phase 5 hand-testing, but it means the Phase 4 heading is pure gyro integration with no drift correction.

### Issue 2: Encoder dtheta Not Passed to Heading Estimator
In `sensor_fusion.cpp`, the call `heading_estimator_update(imu.gyro_z_dps, 0.0f, dt)` passes `0.0f` instead of the actual encoder-derived delta-theta. The heading estimator has no encoder data to fuse with.

### Issue 3: Unthrottled Fusion Loop in Phase 4 Test
The Phase 4 test block runs `fusion_update(dt)` at the raw loop speed (~100kHz). The gyro's 0.85 EMA low-pass filter was tuned for 100Hz. At 100kHz, the filter barely moves and the gyro data is effectively discarded.

### Issue 4: WHEEL_BASE_MM Calibrated Against Wrong Reference
`WHEEL_BASE_MM = 127.1f` was tuned to make the displayed angle match 90° when the robot was turned physically. But the displayed angle was the gyro-only heading (alpha=1.0), not the encoder heading. The calibration was circular and meaningless.

---

## ✅ What Works Correctly

1. **Encoder distance (X/Y):** Pushing the robot straight by hand shows accurate mm values on OLED
2. **MPU6050 raw readings:** The gyro Z-axis reads correct deg/s values (verified in Phase 3)
3. **Gyro calibration:** Bias subtraction works — stationary readings are near zero
4. **Phase 5 gyro-only test:** When throttled to 100Hz, the hand-rotation test shows accurate 90° angles

---

## 📋 Fixes Required

### Fix 1: Throttle Fusion to 100Hz (10ms intervals)
**File:** `Micromouse.ino` Phase 4 test block

**Before:**
```cpp
float dt = (now - last_fusion_tick) / 1000.0f;
if (dt > 0.0f) {
    fusion_update(dt);
    last_fusion_tick = now;
}
```

**After (matching Phase 5 pattern):**
```cpp
static uint32_t last_fusion_tick = millis();
if (millis() - last_fusion_tick >= 10) {
    uint32_t now = millis();
    float dt = (now - last_fusion_tick) / 1000.0f;
    last_fusion_tick = now;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.01f;
    fusion_update(dt);
}
```

### Fix 2: Pass Real Encoder dtheta to Heading Estimator
**File:** `sensors/sensor_fusion.cpp`

The odometry module already computes `d_theta = (right_delta_mm - left_delta_mm) / WHEEL_BASE_MM` internally. We need to expose this value or compute it separately in the fusion layer.

**Approach:** Add a function `odometry_get_last_dtheta()` that returns the delta-theta computed during the last `odometry_update()` call. Then pass it to `heading_estimator_update()`.

### Fix 3: Set alpha to 0.98 (Proper Complementary Filter)
**File:** `localization/heading_estimator.cpp`

**Before:**
```cpp
float alpha = 1.0f;
```

**After:**
```cpp
float alpha = 0.98f;
```

This trusts the gyro 98% for short-term accuracy (immune to wheel slip) and the encoders 2% for long-term stability (prevents gyro drift).

### Fix 4: Re-calibrate WHEEL_BASE_MM
**File:** `config/robot_config.h`

**Procedure:**
1. Apply Fixes 1-3 first
2. Place robot on flat surface with tape marks at 0° and 90°
3. Manually spin robot exactly 360° using physical marks
4. Read the ENCODER-DERIVED angle (not gyro)
5. If encoder says 380°: `WHEEL_BASE_MM` is too small → increase it
6. If encoder says 340°: `WHEEL_BASE_MM` is too large → decrease it
7. Formula: `new_wheelbase = old_wheelbase × (encoder_angle / 360.0)`

---

## 🛠 Testing & Verification for Phase 4 (Revised)

### Test 1: Static Heading Drift
1. Flash with fixes applied
2. Place robot on flat surface, do not touch it
3. Watch heading for 60 seconds
4. **PASS:** Heading stays within ±1° of 0° (no drift)

### Test 2: 90° Hand Rotation
1. Turn robot exactly 90° by hand using right-angle reference
2. **PASS:** Both gyro heading AND encoder heading show ~90° (±3°)

### Test 3: 360° Full Rotation
1. Spin robot 360° by hand, return to start position
2. **PASS:** Heading returns to within ±5° of 0°

### Test 4: Linear Push Test
1. Push robot straight forward exactly 180mm (one maze cell)
2. **PASS:** X shows 180mm (±5mm), Y stays near 0 (±3mm), heading near 0° (±2°)
