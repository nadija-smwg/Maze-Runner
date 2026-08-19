# Phase 3: Sensing & I2C Devices

> **Goal:** Give the robot spatial awareness. We will bring the I2C bus online to communicate with the OLED screen, the MPU6050 gyroscope for rotation tracking, and the 5-sensor VL53L0X array for detecting maze walls.

---

## 1. OLED Display Driver (`display/oled_driver.cpp`)
*   **Purpose:** Provide on-board debugging telemetry without needing a USB cable tethered to a laptop.
*   **Execution Steps:**
    1.  Initialize the `Adafruit_SSD1306` library over the `Wire` (I2C) interface.
    2.  Create helper functions like `oled_print_state()` to display the current battery voltage, robot state (Idle/Exploring), and IMU heading on the 128x64 screen.

## 2. MPU6050 Gyroscope Integration (`sensors/mpu6050.cpp`)
*   **Source Logic:** `Testing Codes/1.MPU6050.ino`
*   **Purpose:** Measure the robot's angular velocity (how fast it is spinning) to ensure it drives perfectly straight and executes exact 90-degree turns.
*   **Execution Steps:**
    1.  **Initialization:** Send the wake-up command over I2C to `0x68`. Configure the gyro for ±500 degrees/second scale (optimal for micromouse speeds).
    2.  **Calibration:** Port `calibrateMPU()`. On startup, while the robot is perfectly still, take 1000 readings of the Z-axis gyro (`gz`). Average them to find the `gyroBiasZ` (the zero-offset noise).
    3.  **Polling Flow:** We are bypassing the MPU `INT` pin. Instead, every time the 1kHz control loop fires, we will directly read I2C register `0x47` (GYRO_ZOUT_H/L).
    4.  **Math:** Subtract `gyroBiasZ` from the raw reading, and multiply by the sensitivity scale factor to get degrees per second.

## 3. VL53L0X Array Multiplexing (`sensors/vl53l0x.cpp`)
*   **Source Logic:** `Testing Codes/VL53L0X/Codefor5TOF.ino`
*   **Purpose:** Initialize 5 identical I2C sensors that all share the same default address (`0x29`) by physically turning them on one by one using `XSHUT` pins.
*   **Execution Steps:**
    1.  Configure `PA4, PA15, PB3, PB1, PC14` as GPIO OUTPUTs.
    2.  Write `LOW` to all 5 pins. (All sensors are now hardware-disabled and off the I2C bus).
    3.  **Sensor 1 (Front):** Write `PA4` `HIGH`. Wait 10ms for boot. Call `Adafruit_VL53L0X::begin(0x30)`. The front sensor now responds only to `0x30`.
    4.  **Sensor 2 (Front-Left):** Write `PA15` `HIGH`. Wait 10ms. Call `begin(0x31)`.
    5.  Repeat this sequential wake-up process for all 5 sensors, assigning addresses `0x30` through `0x34`.

## 4. Wall Detection Logic (`sensors/distance_manager.cpp`)
*   **Purpose:** Continuously poll the VL53L0X sensors and translate millimeter distances into boolean logic for the maze solver.
*   **Execution Steps:**
    1.  In a non-blocking loop (or using the ToF sensor's async measurement mode), read the distance from all 5 sensors.
    2.  Apply threshold logic based on a standard 18cm maze cell.
    3.  **Flow:**
        *   If `front_sensor_mm < 120` -> `wall_front = true`
        *   If `left_90_sensor_mm < 100` -> `wall_left = true`
        *   If `right_90_sensor_mm < 100` -> `wall_right = true`

---

## 🛠 Testing & Verification for Phase 3
1.  **Flash the robot.**
2.  Observe the OLED screen on boot. It should display "Calibrating IMU..." for 3 seconds. Do not touch the robot.
3.  Once booted, the OLED should display live Z-axis rotation rates and ToF distances.
4.  Wave your hand in front of the 5 sensors. Verify that the correct sensor reading on the screen drops.
5.  Rotate the robot by hand. Verify that the Gyro Z value responds rapidly and returns to exactly 0 when stopped.
