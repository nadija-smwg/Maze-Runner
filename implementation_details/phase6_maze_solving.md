# Phase 6: Maze Solving & State Machine (The Brain)

> **Goal:** The ultimate phase. We transplant the advanced pure C algorithmic intelligence from the `src/` folder and connect it to the top-level state machine. The robot will now make autonomous decisions, explore the maze, map the walls, and execute a high-speed optimization run to the center.

---

## 1. Algorithmic Back-End Porting (`src/` → `Full_Code/`)
*   **Purpose:** Simply include the raw, hardware-agnostic C code into the firmware build system.
*   **Execution Steps:**
    1.  Copy `flood_fill.c`, `dijkstra_weighted.c`, `path_smoother.c`, and `solver.c` into the firmware source tree.
    2.  Because these are written in pure C, ensure they are wrapped in `extern "C"` when included in the C++ firmware files.

## 2. The Master State Machine (`robot/robot_state_machine.cpp`)
*   **Purpose:** The highest level logic loop in the robot. It decides if the robot is waiting for the start button, exploring, returning, or doing a speed run.
*   **Execution Steps:**
    1.  **`STATE_IDLE`:** Wait for the user to press `BTN_START`.
    2.  **`STATE_SEARCHING` (The Flood Fill Pipeline):**
        *   *Step A:* Stop in the center of the current cell. Read the ToF sensors.
        *   *Step B:* Call `solver_record_walls(&solver, front, left, right)`. This updates the internal 16x16 map array.
        *   *Step C:* Call `solver_search_step(&solver)`. The Flood Fill algorithm (BFS) instantly re-evaluates the entire maze based on the new wall data and returns the optimal `Direction` (North, South, East, West) to move next.
        *   *Step D:* Command the `motion_controller` to turn the robot to face that direction, and then drive forward exactly 180mm (one cell).
        *   *Step E:* Repeat until `solver_at_goal(&solver)` returns true.
    3.  **`STATE_COMPUTING_FAST_PATH` (The Dijkstra Pipeline):**
        *   *Step A:* Now that the maze is solved, call `solver_compute_fast_path(&solver)`.
        *   *Step B:* The Weighted Dijkstra algorithm runs on a 1024-state space. It penalizes turns (cost +12) over straights (cost +10). It finds the absolute mathematically fastest physical driving line.
        *   *Step C:* The `path_smoother.c` compresses the cell-by-cell path into high level commands (e.g., `[CMD_STRAIGHT(5), CMD_SMOOTH_TURN_90_RIGHT]`).
    4.  **`STATE_FAST_RUN`:**
        *   *Step A:* Fetch the next command via `solver_get_next_command()`.
        *   *Step B:* Pass this command to the `motion_profiles` (Phase 5). The robot executes a massive, non-stop sequence of S-curve straights and rolling arc turns all the way to the center of the maze.

## 3. The Main Loop Integration (`Micromouse.ino`)
*   **Purpose:** The entry point that ties everything together.
*   **Execution Steps:**
    1.  **`setup()`:** Call all initialization functions in order:
        ```cpp
        gpio_init();
        timer_init();
        pwm_init();
        encoder_init();
        mpu6050_init();
        vl53l0x_init_all();
        solver_init(&solver);
        ```
    2.  **`loop()`:** Keep the loop incredibly clean. It should just handle non-blocking updates:
        ```cpp
        button_update();
        led_update();
        distance_manager_update();
        robot_state_machine_update(); // The brain does the rest
        ```

---

## 🛠 Final Testing & Verification
1.  **Flash the robot.**
2.  Place the robot in a physical maze.
3.  Press `BTN_START`.
4.  **Verification:** The robot should cautiously drive cell-by-cell, turning to map the walls. If it hits a dead end, it should turn around and explore a different path until it reaches the center.
5.  Return it to the start cell. Press `BTN_MODE` to trigger the fast run.
6.  **Verification:** The robot should sprint to the center, combining straightaways into single fluid motions and carving through corners without stopping.
