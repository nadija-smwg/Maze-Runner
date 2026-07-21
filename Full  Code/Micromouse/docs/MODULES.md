# Module Responsibilities

## `config/`
- **`pin_config.h`**: Hardware pin assignments (GPIO, Timers, I2C, SPI).
- **`robot_config.h`**: Physical constants (wheel radius, track width, gear ratio, sensor offsets).
- **`config.h`**: Inherited algorithmic constants.

## `hardware/`
- **`motor`**: TB6612FNG direction control.
- **`pwm`**: TIM1 register-level fast PWM generation.
- **`encoder`**: TIM2/TIM3 hardware quadrature encoder reading.
- **`timer`**: TIM4 1kHz interrupt for the control loop.
- **`battery`**: ADC reading for 2S LiPo voltage.
- **`button`**, **`buzzer`**, **`led`**: Basic UI hardware abstractions.

## `sensors/`
- **`mpu6050`**: I2C reading of gyro/accel, gyro zero-rate calibration.
- **`vl53l0x`**: I2C initialization sequence using XSHUT, distance polling.
- **`distance_manager`**: Converts raw mm to wall presence booleans and centering errors.
- **`sensor_fusion`**: Combines gyro rate and encoder odometry to maintain heading.

## `localization/`
- **`odometry`**: Tracks x, y, theta using wheel encoders.
- **`position_estimator`**: Corrects odometry using wall distances (when aligned).
- **`coordinate_transform`**: Converts between global mm (x, y) and maze cells (col, row).

## `control/`
- **`pid`**: Generic PID class.
- **`motion_controller`**: The 1kHz entry point. Calls all sub-controllers.
- **`speed_controller`**: Enforces target mm/s for left/right wheels.
- **`heading_controller`**: Steers robot to target angle.
- **`wall_follower`**: Applies correction to heading based on side wall distances.
- **`trajectory_controller`**: Follows generated motion profiles over time.

## `motion/`
- **`motion_profile`**: Generates trapezoidal/s-curve speed targets.
- **`straight_motion`**: Moves forward X distance.
- **`arc_motion` / `rolling_turn`**: Executes smooth curves.
- **`look_ahead`**: Calculates corner entry speeds.

## `maze/`
- **`maze`**, **`flood_fill`**, **`solver`**: The core AI (copied from original).
- **`maze_explorer`**: Decides when to switch from searching to returning.

## `robot/`
- **`robot_state_machine`**: Global state (Boot, Idle, Run, Error).
- **`mission_manager`**: Coordinates the entire run sequence.
- **`command_executor`**: Feeds `MotionCommand`s to the motion controllers sequentially.
