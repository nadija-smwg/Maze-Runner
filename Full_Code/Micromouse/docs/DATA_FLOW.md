# Data Flow

## 1. The 1kHz Control Loop (Interrupt Context)

Triggered by `hardware/timer` (TIM4) every 1ms.

1. **Read Encoders**: `encoder_read_deltas()` gets ticks since last ms.
2. **Read Gyro**: `mpu6050_read_raw()` gets Z-axis rotation rate.
3. **Sensor Fusion**: `fusion_update()` computes current heading and forward speed.
4. **Odometry**: `odometry_update()` computes new x, y coordinates.
5. **Trajectory Planning**: `trajectory_controller_update()` computes the target speed and heading for this exact millisecond based on the active motion profile.
6. **Wall Following**: If walls are present, `wall_follower_update()` computes a heading correction based on `distance_manager` data.
7. **Heading Control**: `heading_controller_update()` computes the angular velocity needed to achieve the target heading + wall correction.
8. **Kinematics**: `velocity_controller_update()` converts target forward speed and angular velocity into target left and right wheel speeds.
9. **Speed Control**: `speed_controller_update()` runs two PIDs to compare target wheel speeds against actual encoder speeds, outputting left and right PWM values.
10. **Actuation**: `motor_set_both()` writes the PWM to TIM1 registers and sets direction pins.

## 2. The Main Loop (User Context)

Runs continuously, preempted by the 1kHz timer.

1. **Update UI**: `button_update()`, `menu_update()`, `oled_update()`.
2. **Update Distance Sensors**: I2C is slow, so `distance_manager_update()` polls the VL53L0X sensors in the main loop. Data is stored in variables accessed by the control loop.
3. **Battery Monitoring**: Read ADC and update LEDs if low.
4. **State Machine / AI**: `fsm_update()` runs the active mode (e.g., `search_mode_update()`).
   - If idle in a cell, read walls, update maze map, run flood fill, generate next command, push to `command_executor`.
