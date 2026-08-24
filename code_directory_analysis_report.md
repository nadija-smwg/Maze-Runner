# Micromouse Code Directory Analysis Report

This report analyzes the entire `Maze-Runner/Full_Code/Micromouse` codebase, detailing the purpose of each directory and the use cases of all source files within them. The code follows a clean, modular architecture, splitting responsibilities across different specialized modules inside the `src/` directory.

## 📁 Root Directory
Contains the core project file and high-level project folders.
- **`Micromouse.ino`**: The main Arduino sketch file. It contains the `setup()` and `loop()` functions, acting as the entry point that initializes all hardware and starts the high-level robot state machine.
- **`docs/`**: Documentation files for the project.
- **`examples/`**: Example scripts or sub-projects.
- **`tests/`**: Unit tests or test scripts.
- **`src/`**: Contains the entirety of the robot's modular C/C++ source code.

---

## 📁 `src/config/`
Stores global configurations, tunings, and hardware pin mappings.
- **`config.h`**: General global configuration variables and compiler flags.
- **`pin_config.h`**: Maps microcontroller pins to specific hardware peripherals (e.g., motors, sensors, LEDs).
- **`robot_config.h`**: Contains physical robot parameters (e.g., wheel diameter, track width) and high-level tunings.

---

## 📁 `src/control/`
Implements the closed-loop control algorithms that drive the robot accurately.
- **`cell_controller`**: Manages alignment and navigation strictly within a single maze cell.
- **`heading_controller`**: Keeps the robot pointing at the correct angle using gyroscope feedback.
- **`motion_controller`**: High-level coordinator that translates pathing commands into target speeds and headings.
- **`pid`**: A generic Proportional-Integral-Derivative (PID) controller implementation used by the other controllers.
- **`speed_controller`**: Ensures the wheels spin at the target velocities using encoder feedback.
- **`trajectory_controller`**: Calculates the precise paths (trajectories) the robot must follow dynamically.
- **`turn_controller`**: Specifically handles in-place or moving turns accurately.
- **`velocity_controller`**: An abstraction for overall forward and angular velocity targets.
- **`wall_follower`**: Adjusts the robot's heading to stay perfectly centered between maze walls using side distance sensors.

---

## 📁 `src/display/`
Manages the user interface via an onboard OLED screen.
- **`debug_screen`**: Renders raw sensor data and internal state for debugging purposes.
- **`menu`**: Provides an interactive menu system to select modes (Search, Fast Run, Calibrate).
- **`oled_driver`**: Low-level wrapper around the Adafruit SSD1306 OLED library for drawing primitives.
- **`status_screen`**: Displays high-level status (battery voltage, current mode, time).

---

## 📁 `src/hardware/`
Low-level hardware abstraction layer (HAL) for microcontroller peripherals.
- **`battery`**: Reads the battery voltage via an ADC pin to prevent over-discharge.
- **`button`**: Handles user input buttons with debouncing logic.
- **`buzzer`**: Controls the piezo buzzer for audio feedback/beeps.
- **`encoder`**: Reads the quadrature encoders on the wheels to track physical distance traveled.
- **`gpio`**: Wrapper for standard General Purpose Input/Output operations.
- **`led`**: Controls status LEDs (blinking, toggling).
- **`motor`**: Translates target speeds into PWM signals for the motor drivers.
- **`pwm`**: Low-level hardware timer configurations for generating Pulse Width Modulation signals.
- **`timer`**: Configures hardware timers and interrupts used for the main control loops.

---

## 📁 `src/localization/`
Tracks the robot's position and orientation in physical space.
- **`coordinate_transform`**: Converts between global maze coordinates and local robot coordinates.
- **`heading_estimator`**: Integrates gyroscope data to determine the robot's current angle.
- **`odometry`**: Calculates absolute position (X, Y, Theta) based on wheel encoder ticks.
- **`pose`**: Defines the data structures for representing a spatial Pose.
- **`position_estimator`**: Combines odometry and wall-sensing to produce a highly accurate estimate of the robot's location.

---

## 📁 `src/maze/`
The "brain" of the mouse containing maze-solving and pathfinding algorithms.
- **`dijkstra_weighted`**: Pathfinding algorithm used to find the shortest/fastest route through the known maze, factoring in turn costs.
- **`flood_fill`**: Algorithm used to explore the maze and find the center by updating distances to the goal.
- **`maze.h`**: Data structures representing the 16x16 grid, wall definitions, and cell states.
- **`maze_explorer`**: Logic that dictates how the robot explores unknown cells.
- **`path_smoother`**: Optimizes step-by-step paths (e.g. forward, turn right, forward) into smooth, continuous curves (e.g. diagonal runs).
- **`solver`**: The overarching solver that manages the known map and requests paths to the center or back to start.

---

## 📁 `src/motion/`
Generates smooth velocity and acceleration profiles to prevent the tires from slipping.
- **`arc_motion`**: Calculates wheel speeds for driving in curved arcs.
- **`look_ahead`**: Advanced algorithm that looks ahead at upcoming corners to pre-calculate deceleration.
- **`motion_profile`**: Generates smooth acceleration/deceleration trapezoidal or S-curve velocity targets.
- **`rolling_turn`**: Logic for executing high-speed, smooth turns without stopping.
- **`s_curve`**: A jerk-limited velocity profile generator for even smoother acceleration.
- **`straight_motion`**: Profile for moving perfectly straight between cells.

---

## 📁 `src/robot/`
High-level state machines that tie everything together into autonomous behaviors.
- **`command_executor`**: Translates solver directions (North, South, East, West) into physical motion commands (Forward, Turn Right).
- **`competition_state`**: Tracks overall run times and competition metrics.
- **`fast_run_mode`**: The behavior mode used when the maze is solved and the robot dashes to the center as fast as possible.
- **`mission_manager`**: Coordinates switching between exploration, returning to start, and fast runs.
- **`robot_state_machine`**: The overarching state machine (Init -> Idle -> Search -> Fast Run -> Error).
- **`search_mode`**: The behavior mode used for methodically exploring the maze and discovering walls.

---

## 📁 `src/sensors/`
Abstractions for reading and processing raw sensor data.
- **`calibration`**: Routines to calibrate the gyro offsets and distance sensor baselines.
- **`distance_manager`**: High-level interface for reading all distance sensors collectively.
- **`mpu6050`**: I2C driver for the MPU6050 IMU (gyroscope and accelerometer).
- **`sensor_fusion`**: Fuses data from multiple sensors (IMU + Encoders) for better accuracy.
- **`sensor_manager`**: Initializes and polls all sensors at a fixed frequency.
- **`vl53l0x`**: I2C driver for the VL53L0X Time-of-Flight laser distance sensors used for wall detection.

---

## 📁 `src/utils/`
Helper functions, math routines, and generic utilities.
- **`filters`**: Software filters (e.g., low-pass, moving average) to smooth out noisy sensor data.
- **`logger`**: System for logging errors and events.
- **`math_utils`**: Common math functions, angle normalization, and fast approximations.
- **`ring_buffer`**: A circular buffer implementation used for storing streams of data or UART packets.
- **`serial_debug`**: Wraps `Serial.print` functions for printing formatted debug text to a PC over USB.
- **`timing`**: Functions for non-blocking delays and tracking execution time limits.
