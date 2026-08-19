# Phase 4: Sensor Fusion & Localization

> **Goal:** Track the exact physical location of the robot (X, Y, Heading) in the real world. This phase combines the mathematical precision of the wheel encoders with the drift-resistant stability of the MPU6050 gyroscope to create a reliable global pose.

---

## 1. Physical Dimension Calibration
*   **Purpose:** The algorithms in this phase are completely dependent on accurate real-world measurements. If these are wrong, the robot will crash into walls.
*   **Execution Steps:**
    1.  Open `src/config/robot_config.h` (or directly inside `odometry.cpp`).
    2.  Update the constants from the outdated testing codes to the true, verified constants:
        ```cpp
        #define ENCODER_CPR   1820.0f
        #define WHEELBASE_MM  75.0f
        #define MM_PER_COUNT  0.05869f 
        // 0.05869 is derived from: (PI * 34.0mm wheel diameter) / 1820 CPR
        ```

## 2. Odometry Math (`localization/odometry.cpp`)
*   **Purpose:** Calculate how far the robot has traveled and rotated by measuring the difference in encoder counts every 1 millisecond.
*   **Execution Steps:**
    1.  Inside the 1kHz timer interrupt, read the current encoder counts: `curr_left = encoder_get_count(LEFT); curr_right = encoder_get_count(RIGHT);`
    2.  Calculate the change since the last millisecond:
        *   `delta_left_ticks = curr_left - prev_left;`
        *   `delta_right_ticks = curr_right - prev_right;`
    3.  Convert ticks to physical millimeters:
        *   `dist_left_mm = delta_left_ticks * MM_PER_COUNT;`
        *   `dist_right_mm = delta_right_ticks * MM_PER_COUNT;`
    4.  **Kinematics Math:**
        *   Linear distance traveled this millisecond: `delta_distance = (dist_left_mm + dist_right_mm) / 2.0;`
        *   Rotation this millisecond: `delta_theta = (dist_right_mm - dist_left_mm) / WHEELBASE_MM;`
    5.  Update the global coordinate frame:
        *   `global_X += delta_distance * cos(global_heading);`
        *   `global_Y += delta_distance * sin(global_heading);`
        *   `global_heading += delta_theta;`

## 3. Sensor Fusion / Complementary Filter (`sensors/sensor_fusion.cpp`)
*   **Purpose:** The encoder math (Odometry) is perfectly accurate *unless the wheels slip*. If the wheels slip during a fast turn, `delta_theta` will be wrong, and the robot will be permanently lost. The MPU6050 Gyro measures true rotation regardless of wheel slip, but it slowly drifts over time. We must fuse them.
*   **Execution Steps:**
    1.  Read the raw rotation rate from the Gyro in degrees/second (`gyro_rate_z`).
    2.  Integrate the gyro rate to find rotation: `gyro_delta_theta = gyro_rate_z * 0.001s;`
    3.  **The Complementary Filter Math:**
        ```cpp
        // We trust the Gyro 98% for short-term fast movements (immune to wheel slip)
        // We trust the Encoders 2% for long-term stability (immune to gyro drift)
        
        fused_heading = 0.98 * (fused_heading + gyro_delta_theta) + 0.02 * (global_heading_from_odometry);
        ```
    4.  This `fused_heading` is what the PID controllers will actually use to steer the robot.

---

## 🛠 Testing & Verification for Phase 4
1.  **Flash the robot.**
2.  Enable telemetry output over Serial or OLED.
3.  **Linear Test:** Push the robot straight forward exactly 100mm using a ruler. Verify that `global_X` increases by exactly 100.0, and `global_Y` / `fused_heading` remain near zero. If it reads 120mm or 80mm, adjust `MM_PER_COUNT`.
4.  **Rotation Test:** Place the robot on the floor and spin it physically in place by exactly 90 degrees. Verify that `fused_heading` reads `~1.57 radians` (or 90 degrees). If it is off, adjust `WHEELBASE_MM`.
