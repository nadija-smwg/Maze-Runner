# Build and Implementation Order

This document outlines the recommended order for implementing the bodies of the generated `.c` and `.cpp` files.

## Phase 1: Hardware Bring-Up
1. `config/pin_config.h` & `config/robot_config.h` (Verify pins)
2. `hardware/led.cpp`, `hardware/button.cpp`, `hardware/buzzer.cpp`
3. `hardware/motor.cpp` & `hardware/pwm.cpp` (Test spinning motors)
4. `hardware/encoder.cpp` (Test reading ticks)
5. `tests/test_motors.cpp` & `tests/test_encoders.cpp`

## Phase 2: Sensors & I2C
1. `sensors/mpu6050.cpp` (Test gyro reading)
2. `sensors/vl53l0x.cpp` (Test distance reading)
3. `display/oled_driver.cpp` (Get data onto the screen)

## Phase 3: Control Loop & Localization
1. `hardware/timer.cpp` (Get the 1kHz interrupt running)
2. `localization/odometry.cpp` (Track x, y, theta)
3. `sensors/sensor_fusion.cpp` (Combine gyro and encoders)
4. `control/pid.cpp` & `control/speed_controller.cpp` (Velocity control)
5. `control/heading_controller.cpp` (Drive straight)

## Phase 4: Motion & Maze Execution
1. `motion/motion_profile.cpp` & `control/trajectory_controller.cpp` (Smooth accel/decel)
2. `control/cell_controller.cpp` & `control/turn_controller.cpp` (Move 1 cell, turn 90)
3. `robot/command_executor.cpp`
4. Combine with the `maze/` logic for the search run.
