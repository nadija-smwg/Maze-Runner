# Data Flow

## Active Control Architecture (Phase 5.5)

### 1. The 1kHz Control Loop (ISR Context — `motion_controller_update()`)

Triggered by TIM4 every 1ms. **NO I2C reads allowed here.**

```
encoder_update_velocity(0.001s)
  ├─ encoder_get_delta(LEFT)   → dL counts → LPF (α=0.25) → v_L mm/s
  ├─ encoder_get_delta(RIGHT)  → dR counts → LPF (α=0.25) → v_R mm/s
  └─ odometry_update(dL_mm, dR_mm)
       ├─ d_center = (dL + dR) / 2
       ├─ d_theta  = (dR - dL) / WHEEL_BASE_MM
       ├─ theta   += d_theta → normalize [-π, +π]
       ├─ x       += d_center × cos(theta)
       └─ y       += d_center × sin(theta)

mpu6050_get_filtered(&imu)          ← READ CACHED (written in main loop)
  └─ imu.gyro_z_dps

heading_estimator_update(gyro_z_dps, encoder_theta_rad, 0.001s)
  ├─ if |gyro_z_dps| < 1.2 °/s → clamp to 0 (deadband)
  ├─ apply GYRO_MULTIPLIER_LEFT/RIGHT (0.60) based on sign
  ├─ gyro_dtheta = gz_dps × (π/180) × dt
  ├─ fused = 0.999 × (fused + gyro_dtheta) + 0.001 × encoder_theta
  └─ normalize [-π, +π]

[if TURNING]:
  heading_err = current_heading - turn_target
  normalize heading_err to [-π, +π]
  w = -KP_TURN × heading_err   (KP_TURN = 5.0)
  clamp w to [-MAX_TURN=6.0, +MAX_TURN=6.0]
  enforce MIN_TURN=1.5 anti-stall
  velocity_controller_update(v=0, w)  → done

[if CELL MOVING]:
  dist = |odometry_x_mm|
  straight_motion_update(dist)  → profile_get_speed() → v (mm/s)
  heading_err = 0 - fused_heading
  w = KP_HEADING × heading_err  (KP_HEADING = 2.0)
  lateral_err = distance_get_centering_error()   ← cached from main loop
  w += wall_follower_update(lateral_err, 0.001)  ← PD: KP=0.010, KD=0.005
  velocity_controller_update(v, w)

velocity_controller_update(v, w):
  v_L = v - w × (WHEEL_BASE/2)   (WHEEL_BASE = 95.3mm)
  v_R = v + w × (WHEEL_BASE/2)
  actual_L = encoder_get_speed_mms(LEFT)
  actual_R = encoder_get_speed_mms(RIGHT)
  speed_controller_update(v_L, v_R, actual_L, actual_R, 0.001)

speed_controller_update():
  PWM_L = KFF×v_L + KP×err_L + KI×∫err_L   (KFF=2.6, KP=5.0, KI=1.0)
  PWM_R = KFF×v_R + KP×err_R + KI×∫err_R
  anti-windup clamp: ±1000 PWM units
  motor_set_speed_compensated(LEFT,  PWM_L)  ← adds LEFT_MOTOR_DEAD_PWM=300
  motor_set_speed_compensated(RIGHT, PWM_R)  ← adds RIGHT_MOTOR_DEAD_PWM=250
```

### 2. Main Loop (~200Hz base)

Runs continuously, preempted by 1kHz ISR.

```
mpu6050_update_filter(0.005s):          ← I2C read, ~200Hz
  Read raw 14 bytes from MPU6050
  Apply bias offsets (calibrated 1000 samples)
  EMA: accel_filt = 0.90×prev + 0.10×new
       gyro_filt  = 0.80×prev + 0.20×new
  If stationary AND |gz_filt| < 3.5°/s:
    bias_z slowly corrects at rate 0.002/update

mpu6050_set_stationary(motion_is_idle())

distance_manager_update():              ← I2C, ~100Hz (every 10ms)
  For each of 5 sensors (FRONT, FL, FR, LEFT, RIGHT):
    vl53l0x_read_distance_mm()
    tof_filter_process():
      [1] validity: reject < 20mm or > 500mm
      [2] calibration offset (signed, per-sensor)
      [3] median-3 (ring buffer of 3 samples)
      [4] jump reject: |median - previous| > 50mm
      [5] EMA: α=0.30
  Update wall hysteresis flags:
    wall_left:  < 110mm → true,  > 130mm → false
    wall_right: < 110mm → true,  > 130mm → false
    wall_front: < 140mm → true,  > 160mm → false

button_update() + led_update()
OLED update (~20Hz)
Serial debug print (~20Hz)
```

### 3. Notes on sensor_fusion.cpp (DEAD CODE)

`sensor_fusion.cpp` implements a second heading fusion system (98% gyro / 2% encoder in degrees). **`fusion_update()` is never called in any active test mode loop().** The active heading estimator is `heading_estimator.cpp` (99.9% gyro / 0.1% encoder in radians), called from the 1kHz ISR. `sensor_fusion.cpp` should be removed in Phase 6 cleanup.

### 4. Phase 6 Data Flow (Planned — Not Yet Implemented)

```
After each cell completes:
  distance_manager_update()
  maze_update_walls(x, y, heading,
                    front=distance_has_wall_front(),
                    left=distance_has_wall_left(),
                    right=distance_has_wall_right())
  flood_fill(maze, goal_cells)
  next_dir = maze_get_next_direction(x, y, heading)
  cmd = direction_to_motion_command(next_dir)
  motion_execute_command(&cmd)
  → wait for motion_is_idle()
  → repeat
```
