# Robot State Machine

The top-level behavior is governed by `robot_state_machine`.

## States

- **`STATE_BOOT`**: Power on, initializing hardware, calibrating gyro. Motors disabled.
- **`STATE_IDLE`**: Waiting for user input. OLED shows menu. Control loop maintains 0 velocity.
- **`STATE_CALIBRATING`**: Running advanced calibration (e.g., wall sensor tuning).
- **`STATE_SEARCH_RUN`**:
  - Explores the maze.
  - Pauses in center of cells to update map and run flood fill.
  - Moves cell-by-cell.
- **`STATE_RETURN_START`**:
  - Goal found. Calculates shortest path back to (0,0).
  - Uses fast straight lines, does not stop to scan walls.
- **`STATE_FAST_RUN`**:
  - Calculates optimal diagonal path.
  - Executes high-speed rolling turns and s-curve straights.
- **`STATE_ERROR`**:
  - Battery critical, sensor failed, or picked up (wheels free-spinning).
  - Motors disabled, buzzer sounds alarm.
