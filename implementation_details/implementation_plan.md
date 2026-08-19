# 🧠 Master Micromouse Implementation Plan

> **Objective:** A highly detailed, step-by-step master plan for extracting raw hardware logic from `Testing Codes/` and advanced pathfinding algorithms from `src/`, integrating them into the modular C++ architecture of `Full Code/Micromouse/`.
> 
> *Crucially, all hardcoded dimension values in the testing codes must be updated to the true robot dimensions (`ENCODER_CPR = 1820`, `WHEELBASE_MM = 75.0`, `MM_PER_COUNT = 0.05869`) during the porting process.*

---

## Proposed Changes: Phased Technical Integration

### Phase 1: Hardware Abstraction & Basic I/O
*This phase gets the simplest hardware working. (Note: Basic GPIO and dual LED logic is already completed and synced with `pin_config.h`)*

*   **[MODIFY]** `hardware/gpio.cpp` - Setup standard pins.
*   **[MODIFY]** `hardware/button.cpp` - Implement button reading and debounce logic for `PB5` and `PB4`.
*   **[MODIFY]** `hardware/battery.cpp` - Implement ADC reads on `PB0`. Use a voltage divider ratio to calculate battery voltage and mapped percentage.
*   **[MODIFY]** `hardware/timer.cpp` - Implement a hardware timer interrupt that triggers exactly at 1kHz. This is the heartbeat of the robot that will call `motion_controller_update()`.

### Phase 2: Actuation & Low-Level Kinematics
*Porting the raw motor and encoder timer logic from testing codes.*

*   **[MODIFY]** `hardware/pwm.cpp` - **Port from:** `Testing Codes/4.Motors_Combine_With_Motors.ino`
    - Manually configure STM32 Timer 1 (TIM1) via registers for 20 kHz PWM.
    - Set `GPIOA->MODER` for PA8 and PA9 to Alternate Function (AF1).
    - Map `TIM1->CCR1` and `CCR2` to `pwm_set_duty()`.
*   **[MODIFY]** `hardware/motor.cpp` - **Port from:** `Testing Codes/4.Motors_Combine_With_Motors.ino`
    - Implement directional wrapper functions using `AIN1`/`AIN2` (`PB12`, `PB13`) and `BIN1`/`BIN2` (`PB15`, `PA10`).
*   **[MODIFY]** `hardware/encoder.cpp` - **Port from:** `Testing Codes/3.Motors_With_Encoders.ino`
    - Extract `setupLeftEncoder()` (TIM2 on PA0/PA1) and `setupRightEncoder()` (TIM3 on PA6/PA7).
    - Configure both timers in **Encoder Mode 3** (`TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1`) for 4X quadrature decoding.
*   **[MODIFY]** `control/pid.cpp` - Implement the standard mathematical PID loop (Proportional, Integral, Derivative) with anti-windup constraints for velocity and heading control.

### Phase 3: Sensing & I2C Devices
*Porting the IMU and Time-of-Flight sensor arrays.*

*   **[MODIFY]** `display/oled_driver.cpp` - Implement `Adafruit_SSD1306` initialization for headless debugging.
*   **[MODIFY]** `sensors/mpu6050.cpp` - **Port from:** `Testing Codes/1.MPU6050.ino`
    - Port `calibrateMPU()` to average 1000 samples of the Z-axis gyro (`gz`) to compute `gyroBiasZ` at startup.
    - Do **not** configure the MPU6050 `INT` pin. The 1kHz fast loop will poll I2C register `0x47` instead.
*   **[MODIFY]** `sensors/vl53l0x.cpp` - **Port from:** `Testing Codes/VL53L0X/Codefor5TOF.ino`
    - Implement `XSHUT` multiplexing: Pull all 5 pins (`PA4, PA15, PB3, PB1, PC14`) LOW to disable sensors. Pull them HIGH one by one, calling `sensor.begin(new_addr)` to assign unique I2C addresses (e.g., `0x30`, `0x31`, etc.) without collision.
*   **[MODIFY]** `sensors/distance_manager.cpp` - Translate raw mm readings from the 5 VL53L0X sensors into boolean wall states (e.g., `front_wall_detected = (front_distance_mm < 120)`).

### Phase 4: Sensor Fusion & Localization
*Fusing encoders and gyroscope into a stable global pose using corrected dimensions.*

*   **[MODIFY]** `localization/odometry.cpp` 
    - **CRITICAL UPDATE:** Replace test code constants with the true physical constants of the robot:
      - `ENCODER_CPR = 1820.0f`
      - `WHEELBASE_MM = 75.0f`
      - `MM_PER_COUNT = 0.05869f`
    - **Pose Math:** `Δd = ((ΔLeft + ΔRight) / 2.0) * MM_PER_COUNT`
    - **Pose Math:** `Δθ = ((ΔRight - ΔLeft) * MM_PER_COUNT) / WHEELBASE_MM`
*   **[MODIFY]** `sensors/sensor_fusion.cpp`
    - Integrate the raw Z-axis rotation rate over time (`dt = 0.001s`).
    - Apply a Complementary Filter to prevent wheel slip from corrupting the heading: `fused_yaw = 0.98 * (fused_yaw + (gyro_rate_z * dt)) + 0.02 * (odometry_yaw)`.

### Phase 5: High-Level Motion & Wall Following
*Integrating the pure C algorithm kinematics from `src/` into the firmware controllers.*

*   **[MODIFY]** `control/motion_controller.cpp` - Hook up the master control loop running on the 1kHz timer. It calculates trajectory errors and feeds them to the PID controllers.
*   **[MODIFY]** `motion/motion_profiles.cpp` - **Port from:** `src/motion_profile.c`
    - Integrate the S-Curve velocity ramps to prevent wheel slip on hard accelerations.
*   **[MODIFY]** `motion/turn_control.cpp` & `control/wall_follower.cpp` - **Port from:** `src/path_smoother.c`
    - Implement path smoothing (translating `[NORTH, NORTH]` into `CMD_STRAIGHT(2)`).
    - Use side ToF sensors (L-90 and R-90) to calculate a PD error to keep the robot perfectly centered in the 18cm maze cell.

### Phase 6: Maze Solving & State Machine (The Brain)
*Integrating the pure C graph algorithms from `src/` into the robot state machine.*

*   **[MODIFY]** `maze/maze.cpp`, `maze/flood_fill.cpp` - **Port from:** `src/maze.h` and `src/flood_fill.c`.
    - Integrate the multi-source BFS flood-fill algorithm. This acts like water flowing backward from the center of the maze to the robot, recalculating shortest paths dynamically as new walls are found.
*   **[MODIFY]** `robot/robot_state_machine.cpp` - **Port from:** `src/solver.c` and `src/dijkstra_weighted.c`.
    - **Search Run Logic:** Feed walls into `solver_record_walls()`, then ask for the next move via `solver_search_step()`.
    - **Fast Run Logic:** Use the Weighted Dijkstra algorithm. It computes paths on a 1,024-state 3D space `(X, Y, Heading)`. It adds heavy cost penalties to turns (`+12`) compared to driving straight (`+10`), ensuring the robot prioritizes straightaways for maximum speed.
*   **[MODIFY]** `Micromouse.ino` - Tie all module `init()` functions together in `setup()` and link the main polling loop.

---

## User Review Required

> [!IMPORTANT]
> This master implementation plan combines the phase-by-phase approach with explicit hardware-to-firmware mapping, function signatures, mathematical dimension corrections, and deep algorithmic breakdowns.
>
> If this level of technical detail satisfies the requirements for the implementation plan, please click **Proceed** so I can begin generating code for **Phase 1** and **Phase 2**.
