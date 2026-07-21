# Initialization Sequence

The Arduino `setup()` function in `Micromouse.ino` orchestrates the boot sequence:

1. **Hardware Init**:
   - `gpio_init()`, `pwm_init()`, `encoder_init()`
   - `button_init()`, `led_init()`, `buzzer_init()`
   - `battery_init()`

2. **I2C & Display Init**:
   - `Wire.begin()`
   - `oled_init()`
   - Display boot logo / version.

3. **Sensor Init**:
   - `mpu6050_init()`
   - `vl53l0x_init_all()` (Handles XSHUT sequence)

4. **Software Modules Init**:
   - `odometry_init()`
   - `fusion_init()`
   - `motion_controller_init()` (Initializes all PIDs)
   - `maze_init()`

5. **Calibration**:
   - Require robot to be perfectly still.
   - `calibrate_gyro()` runs for ~1-2 seconds.
   - Beep to indicate readiness.

6. **Start Control Loop**:
   - `timer_init(motion_controller_update)`
   - `timer_start()`

7. **Enter FSM**:
   - `fsm_set_state(STATE_IDLE)`
