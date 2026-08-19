# Phase 5: High-Level Motion & Wall Following

> **Goal:** Connect the pure C algorithmic kinematic profiles to the PID control loops. This gives the robot the ability to smoothly accelerate, cruise, and brake precisely into the center of a maze cell, while using the side walls to correct its alignment.

---

## 1. S-Curve Velocity Generation (`motion/motion_profiles.cpp`)
*   **Source Logic:** `src/motion_profile.c`
*   **Purpose:** Prevent the robot from requesting an instant jump from 0 to 100% PWM. Sudden acceleration causes the wheels to spin out, completely ruining the odometry tracking from Phase 4.
*   **Execution Steps:**
    1.  Port the math from `motion_profile.c`.
    2.  When a command like `DRIVE_STRAIGHT(180mm)` is issued, the profiler calculates three phases:
        *   **Acceleration Ramp:** Increases velocity targets using a sine wave or trapezoidal ramp until it hits `MAX_SPEED`.
        *   **Cruise:** Holds `MAX_SPEED`.
        *   **Deceleration Ramp:** Constantly checks `remaining_distance`. When `remaining_distance` < `stopping_distance_threshold`, it begins ramping the target velocity smoothly down to 0.
    3.  Every 1 millisecond, the profiler outputs the *current target velocity* (`target_v`) and *current target heading* (`target_w`).

## 2. PID Motion Controller (`control/motion_controller.cpp`)
*   **Purpose:** The master loop that runs every 1 millisecond. It takes the target speeds from the motion profiler, compares them to the actual speeds from the odometry, and adjusts the motor PWMs.
*   **Execution Steps:**
    1.  **Read Targets:** Fetch `target_v` and `target_w` from the motion profile.
    2.  **Read Actuals:** Fetch `actual_v` and `actual_w` from `odometry.cpp`.
    3.  **Calculate Errors:**
        *   `error_v = target_v - actual_v;`
        *   `error_w = target_w - actual_w;`
    4.  **Run PID:** Feed the errors into the PID math blocks implemented in Phase 2.
        *   `correction_v = PID_Compute(&speed_pid, error_v);`
        *   `correction_w = PID_Compute(&heading_pid, error_w);`
    5.  **Mixer / Kinematics:** Combine the linear speed correction and the angular rotation correction into specific wheel commands:
        *   `pwm_left = correction_v - correction_w;`
        *   `pwm_right = correction_v + correction_w;`
    6.  Send `pwm_left` and `pwm_right` to `motor_set_speed()`.

## 3. Wall Following PD Controller (`control/wall_follower.cpp`)
*   **Purpose:** Even with perfect odometry, the robot will slowly drift laterally over a long maze run. The L-90 and R-90 Time-of-Flight sensors are used to measure the distance to the side walls and mathematically pull the robot back to the dead-center of the 18cm cell.
*   **Execution Steps:**
    1.  Fetch `left_distance` and `right_distance` from the VL53L0X sensors.
    2.  If both walls are present:
        *   `centering_error = left_distance - right_distance;`
    3.  If only one wall is present (e.g. left wall):
        *   `centering_error = left_distance - TARGET_DISTANCE_TO_WALL_MM;`
    4.  Run a Proportional-Derivative (PD) loop on this error to generate a `wall_correction` value.
    5.  Inject this `wall_correction` directly into the `target_w` (heading) of the `motion_controller` so the robot imperceptibly steers itself back to the center line.

---

## 🛠 Testing & Verification for Phase 5
1.  **Flash the robot.**
2.  Set the robot on a flat surface and trigger a test command: `DRIVE_STRAIGHT(500mm)`.
3.  Observe the robot. It should gently ramp up speed, cruise quickly, and smoothly ramp down, coming to a complete stop *exactly* 500mm away without skidding.
4.  **Wall Following Test:** Place the robot in a physical maze corridor. Push it slightly off-center. When it drives, it should actively steer itself back to the middle of the corridor based on the ToF sensor feedback.
