# 📍 Micromouse Localization Module: Deep Dive & Tuning Guide

The `localization` module is the "inner ear and eyes" of your Micromouse. It answers the fundamental question: **"Where am I right now?"** 

Because wheels slip and gyro sensors drift, a single sensor is never enough. This module uses **Sensor Fusion** to combine wheel encoders (Odometry), the MPU6050 (Gyro), and the VL53L0X (Time-of-Flight Lasers) into one highly accurate `Pose` (X, Y, Theta).

Here is a full breakdown of how it works, how to tune it, and how to complete the final steps.

---

## 1. How It Works: The Three Pillars

The localization system is built on three main estimators that feed into each other.

### Pillar 1: Odometry (`odometry.cpp`)
**Goal:** Track movement purely by counting wheel rotations.
- **How it works:** It reads the change in the Left and Right encoders every millisecond.
  - Forward distance (`d_center`) = average of Left and Right distance.
  - Rotation (`d_theta`) = (Right - Left) / `WHEEL_BASE_MM`.
- **The Math:** It uses basic trigonometry to update the X and Y coordinates:
  - `X = X + d_center * cos(theta)`
  - `Y = Y + d_center * sin(theta)`
- **The Flaw:** Odometry is great for short distances, but if the wheels slip (dust on the maze floor, hard acceleration), the robot thinks it moved when it didn't. This error accumulates over time.

### Pillar 2: Heading Estimator (`heading_estimator.cpp`)
**Goal:** Fix the rotation errors from wheel slip using the Gyroscope.
- **How it works:** It uses a **Complementary Filter** to fuse the Gyro and the Encoders.
  - **Gyro:** Highly accurate in the short term, immune to wheel slip. But over 30 seconds, it slowly drifts.
  - **Encoders:** Never drift over time, but suffer from sudden wheel slip.
- **The Filter Equation:** 
  `fused_heading = (0.98 * gyro_heading) + (0.02 * encoder_heading)`
  This trusts the gyro 98% for immediate turns, but constantly pulls it 2% towards the encoders to prevent long-term drift.
- **Deadband:** If the encoders say the robot is perfectly still, it ignores tiny gyro readings to prevent noise from accumulating.

### Pillar 3: Position Estimator (`position_estimator.cpp`)
**Goal:** Fix the X/Y drift using the maze walls (ToF sensors).
- **How it works:** Even with perfect heading, the X/Y coordinates will slowly drift. But in a micromouse maze, we know the exact layout of the grid! Every cell is exactly 180mm wide.
- If the robot is driving perfectly straight North (Heading ~ 0°), and it sees a wall on its left, it knows that wall *must* be exactly at the edge of the cell.
- It calculates an `expected_y` based on the grid, subtracts the actual laser reading, and gently "tugs" the robot's internal `Y` coordinate towards reality using an Exponential Moving Average (EMA).

---

## 2. How to Tune the Localization Values

Tuning must be done in a specific order: **Physical Constants → Heading → Position**.

### Step 1: Tune Odometry (Physical Constants)
*Files involved: `config/robot_config.h`*

If odometry is wrong, everything else fails. 
1. **Drive exactly 1 meter (1000mm) forward.**
   - If the robot thinks it drove 950mm, your wheels are smaller than you thought. 
   - **Fix:** Adjust `WHEEL_DIAMETER` or `GEAR_RATIO` in `robot_config.h` until `MM_PER_COUNT` is perfect.
2. **Command the robot to spin exactly 360 degrees (10 times) in place.**
   - Measure the actual physical rotation. If it spun 370 degrees, your wheel base measurement is wrong.
   - **Fix:** Adjust `WHEEL_BASE_MM`. Increase the number if it turns too much; decrease it if it turns too little.

### Step 2: Tune the Heading Estimator (Complementary Filter)
*Files involved: `localization/heading_estimator.cpp`*

Look at this line:
```cpp
float alpha = 0.98f; 
_fused_heading_deg = alpha * (_fused_heading_deg + gyro_dtheta_deg) + (1.0f - alpha) * encoder_heading_deg;
```
- **`alpha` (0.98f):** This is the trust factor for the gyro. 
  - If your robot stutters when turning, the encoders are slipping and fighting the gyro. **Increase `alpha` to `0.99f` or `0.995f`**.
  - If your robot drives straight for a long time but slowly veers off course, the gyro is drifting. **Decrease `alpha` to `0.95f`** to trust the wheels more.

### Step 3: Tune the Deadband
*Files involved: `localization/heading_estimator.cpp`*

```cpp
if (fabs(encoder_dtheta_rad) < 0.001f && fabs(gyro_z_dps) < 1.0f)
```
- If the robot is standing completely still but the heading slowly creeps up on the OLED screen, your gyro is noisy.
- **Fix:** Increase the gyro threshold from `1.0f` to `2.0f`.

---

## 3. How to Complete the Final Step (Position Estimator)

Right now, the wall corrections in `position_estimator.cpp` are **commented out** because the code was being tested on an open floor. If you ran this on a floor without maze walls, seeing a random object (like a shoe) would completely ruin the X/Y coordinates.

Now that you are moving to the maze, you need to complete this file.

### Actions to take in `position_estimator.cpp`:

**1. Uncomment the correction block (Lines 42-54):**
Remove the `/*` and `*/` surrounding the wall correction logic.

**2. Update the Target Distance:**
Since your robot is now ~120mm wide (as updated in the `distance_manager`), the distance to the wall when perfectly centered is `30mm`. 
The old code assumed: `expected_y = cell_center + 90.0f` (edge of cell).

Let's rewrite the logic to be more robust. Replace the commented-out block with this:

```cpp
    // 4. Wall Corrections (Only if driving perfectly straight)
    // To simplify for now, we only correct when facing North (Heading ~ 0)
    // Tolerance: Must be within 3 degrees of perfect North
    if (fabs(fused_heading_deg) < 3.0f) {
        
        // Find the absolute center of the cell we are currently in
        float current_cell_center_y = cell_to_mm(mm_to_cell(_best_pose.y_mm));
        
        // If we see a left wall, it means there is a wall at +90mm from cell center.
        // The robot's left ToF sensor sits roughly 60mm from the robot center (120mm wide).
        if (distance_has_wall_left()) {
            float measured_dist = distance_get_mm(TOF_LEFT);
            // Where the robot SHOULD be based on the laser:
            // Wall is at (cell_center + 90). Robot center = Wall - measured_dist - half_width
            float laser_calculated_y = (current_cell_center_y + 90.0f) - measured_dist - 60.0f;
            
            // Gently pull our estimated Y towards the laser calculated Y (95% Odometry, 5% Laser)
            _best_pose.y_mm = (0.95f * _best_pose.y_mm) + (0.05f * laser_calculated_y);
        }
        
        // Same logic for the right wall (Right wall is at -90mm from cell center)
        if (distance_has_wall_right()) {
            float measured_dist = distance_get_mm(TOF_RIGHT);
            float laser_calculated_y = (current_cell_center_y - 90.0f) + measured_dist + 60.0f;
            
            _best_pose.y_mm = (0.95f * _best_pose.y_mm) + (0.05f * laser_calculated_y);
        }
    }
```

### 4. Tuning the Position Estimator (The `0.05f` factor)
Look at the `0.05f` (5%) weight in the equations above.
- This means every millisecond, we trust odometry 95% and the lasers 5%. 
- If the robot "snaps" violently left or right when it detects a wall, **decrease this to `0.01f` or `0.02f`** (so it corrects slower and smoother).
- If the robot crashes into the wall before the correction has time to pull it back to the center, **increase this to `0.10f`**.

### Summary of Completion Steps:
1. Ensure `WHEEL_BASE_MM` and `MM_PER_COUNT` are physically validated.
2. Replace the commented block in `position_estimator.cpp` with the math provided above, which correctly accounts for the new 120mm robot width (60mm half-width).
3. Test the robot driving down a straight hallway of maze walls. Watch the X/Y coordinates on Serial/OLED to ensure they don't wildly jump when walls appear and disappear.

---

## 5. Tuning the PID Controllers (Control Layer)

While the localization module tells the robot where it is, the **PID Controllers** take that information and calculate exactly how much PWM to apply to the motors to fix any errors. 

Tuning PID must be done from the bottom up: **Tune Speed first, then Heading**.

### Step 1: Tune Speed Control (`speed_controller.cpp`)
**Goal:** Make the wheels spin at exactly the commanded speed (mm/s) regardless of battery voltage or friction.

*Constants:*
- `SPEED_KP = 2.0f`, `SPEED_KI = 1.0f`, `SPEED_KD = 0.0f`
- `FEEDFORWARD_KV = 5.0f`

**Tuning Procedure:**
1. **Feedforward (`FEEDFORWARD_KV`):** This is the most important part for speed. Set all PID gains to `0.0f`. Command the robot to drive 500 mm/s. Look at the actual speed (via encoders). Adjust `FEEDFORWARD_KV` until the robot naturally drives at roughly 500 mm/s without any PID helping it. 
   *(Formula: `FEEDFORWARD_KV` ≈ `PWM_MAX` / `max_physical_speed_mm_s`)*
2. **Proportional (`SPEED_KP`):** Increase `SPEED_KP` slightly until the robot quickly responds to speed changes but doesn't oscillate or vibrate.
3. **Integral (`SPEED_KI`):** Add a small amount of `SPEED_KI` to eliminate steady-state error (e.g., if you command 500 but it maxes out at 490, `KI` will slowly push it up to 500 over time).
4. **Derivative (`SPEED_KD`):** Usually kept at `0.0f` for wheel speed control, as encoder speed readings are too noisy for derivative math.

### Step 2: Tune Heading Control (`heading_controller.cpp`)
**Goal:** Keep the robot driving perfectly straight or turn exactly to a target heading.

*Constants:*
- `HEADING_KP = 5.0f`, `HEADING_KI = 0.0f`, `HEADING_KD = 0.1f`

**Tuning Procedure:**
1. **Proportional (`HEADING_KP`):** Set `KI` and `KD` to `0.0f`. Command the robot to drive straight down a hallway. If it drifts off course, increase `HEADING_KP`. If the robot wiggles left and right violently (oscillating) as it drives, `KP` is too high.
2. **Derivative (`HEADING_KD`):** Add `KD` to dampen the oscillation. If `KP` makes it wiggle, `KD` acts like a shock absorber to smooth out the steering. Too much `KD` will make the steering sluggish and cause high-frequency buzzing/vibrating in the motors.
3. **Integral (`HEADING_KI`):** Usually kept at `0.0f` for heading in a micromouse. Only add a tiny amount if the robot consistently drives slightly off-center (e.g., it always maintains 2 degrees to the left of your target).
