# Micromouse Implementation Plan

Based on the analysis of the `Full Code` directory, the project currently consists of a well-structured architectural skeleton. The header files define a clean API, and `Micromouse.ino` has the high-level `setup()` and `loop()` structure, but almost all the underlying `.cpp` files contain `TODO` comments and empty functions.

This plan breaks down the implementation of the remaining `TODO`s into logical, manageable, and incremental steps. This ensures that we can test and verify each layer before building on top of it.

## User Review Required
> [!IMPORTANT]
> Since we are developing and testing on the physical robot, this plan is designed for iterative deployment. We will complete one phase, you will flash it to the robot and verify the behavior, and then we will move to the next phase. 
> 
> Please review this updated plan and click **Proceed** to authorize the start of **Phase 1**.

---

## Proposed Changes: Phased Implementation

### Phase 1: Hardware Abstraction & Basic I/O (Easiest)
This phase gets the simplest hardware working. We can test this by turning on LEDs, reading the button, and making sounds on the robot.
*   **[MODIFY]** `hardware/gpio.cpp` - Setup motor and standard pins.
*   **[MODIFY]** `hardware/led.cpp` - Implement LED toggling and states.
*   **[MODIFY]** `hardware/button.cpp` - Implement button reading and debounce logic.
*   **[MODIFY]** `hardware/buzzer.cpp` - Implement tone generation for startup sounds.
*   **[MODIFY]** `hardware/battery.cpp` - Implement ADC reads to calculate battery voltage and percentage.
*   **[MODIFY]** `hardware/timer.cpp` - Implement hardware/software timer for the control loop interrupt.

### Phase 2: Actuation & Low-Level Control
Getting the motors spinning and reading how far they have spun. We will port your existing tested logic for this.
*   **[MODIFY]** `hardware/pwm.cpp` & `hardware/motor.cpp` - Port the raw Timer 1 PWM configuration and directional control logic directly from `Testing Codes/4.Motors_Combine_With_Motors/4.Motors_Combine_With_Motors.ino`.
*   **[MODIFY]** `hardware/encoder.cpp` - Port the Timer 2 and Timer 3 hardware quadrature encoder reading logic directly from `Testing Codes/3.Motors_With_Encoders/3.Motors_With_Encoders.ino`.
*   **[MODIFY]** `control/pid.cpp` - Implement the standard PID mathematical calculation.

### Phase 3: Sensing & I2C Devices
Bringing the "eyes" and "balance" of the mouse online.
*   **[MODIFY]** `display/oled_driver.cpp` - Get the screen working for easier debugging without a PC.
*   **[MODIFY]** `sensors/mpu6050.cpp` - Port the raw I2C logic and roll/pitch/yaw filtering math from `Testing Codes/1.MPU6050/1.MPU6050.ino` into this clean module.
*   **[MODIFY]** `sensors/vl53l0x.cpp` & `sensors/distance_manager.cpp` - Implement the XSHUT pin toggling sequence and use the **Adafruit_VL53L0X** library to assign unique I2C addresses and read distances.

### Phase 4: Sensor Fusion & Localization
Combining sensor data to figure out where the robot is.
*   **[MODIFY]** `localization/odometry.cpp` - Convert encoder ticks into linear distance (X/Y) and angular rotation.
*   **[MODIFY]** `sensors/sensor_fusion.cpp` - Combine the MPU6050 Yaw data with the Encoder odometry to create a robust, slip-resistant heading estimate.

### Phase 5: High-Level Motion & Wall Following
Making the robot move intelligently rather than just spinning wheels. We will use the existing C code from your `src/` directory.
*   **[MODIFY]** `control/motion_controller.cpp` - The core control loop running on a timer. It uses PID to achieve target linear/angular velocities.
*   **[MODIFY]** `motion/motion_profiles.cpp` - Integrate the trapezoidal velocity profiles from `src/motion_profile.c`.
*   **[MODIFY]** `motion/turn_control.cpp` & `motion/wall_follower.cpp` - Integrate the path smoothing logic from `src/path_smoother.c` and use the side ToF sensors to calculate a PD error to keep the robot perfectly centered in the maze cell.

### Phase 6: Maze Solving & State Machine (Most Complex)
The actual "brain" of the Micromouse. We will port the advanced algorithms directly from your `src/` directory.
*   **[MODIFY]** `maze/maze.cpp`, `maze/flood_fill.cpp` - Port the 16x16 maze array and the BFS flood-fill algorithm from `src/maze.h` and `src/flood_fill.c`.
*   **[MODIFY]** `robot/robot_state_machine.cpp` & `robot/mission_manager.cpp` - Integrate the fast-run Dijkstra logic from `src/dijkstra_weighted.c` and `src/solver.c`.
*   **[MODIFY]** `Micromouse.ino` - Uncomment the remaining integration code (watchdogs, maze init, etc.)

---

## Verification Plan

Because you have the physical robot, we will use it as our primary testbed.
After completing each phase, we will add a temporary testing block in `Micromouse.ino`'s `setup()` or `loop()`. 
You will flash this to the robot, observe the physical behavior (e.g., LED blinks, motors spin, OLED displays values), and report back. Only after a phase is verified on-device will we move to the next.
