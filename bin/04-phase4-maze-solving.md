# Phase 4 — Maze Solving Algorithms (The DSA)

Prerequisite: motion primitives from Phase 3 (`move_forward_one_cell()`,
`turn_to(theta)`), and wall-sensing from Phase 2 (IR/ToF).

This phase leans on your existing DSA strength — I'll go faster here on
algorithm mechanics and focus on the **embedded-specific constraints**
(memory layout, fixed-size structures, no dynamic allocation, integer-only
math where it matters) that differ from a typical desktop implementation.

## 4.1 Modeling the 16×16 Maze in Memory

### Core representation
A standard micromouse maze is 16×16 cells, each cell up to 4 walls
(N/E/S/W), walls shared between adjacent cells. On an embedded target with
128KB RAM, memory is not the bottleneck here (256 cells × a few bytes is
trivial) — but **avoid `std::vector`, `std::map`, heap allocation of any
kind** in the maze/control path. Use fixed-size arrays. Heap fragmentation
and unpredictable allocation latency are unacceptable in a real-time embedded
loop, and on Cortex-M4F without an MMU, a fragmented heap can hard-fault you
mid-run with no graceful recovery.

```cpp
#define MAZE_SIZE 16

// Bitmask per cell: bit0=N, bit1=E, bit2=S, bit3=W wall present
enum WallBit : uint8_t { WALL_N = 0x01, WALL_E = 0x02, WALL_S = 0x04, WALL_W = 0x08 };

struct Cell {
    uint8_t walls;       // known walls (bitmask)
    uint8_t visited;      // has the mouse physically been here
    uint16_t flood_value; // Manhattan-ish flood-fill distance to goal
};

Cell maze[MAZE_SIZE][MAZE_SIZE];  // static allocation — 16*16*~4 bytes = 1KB, negligible
```

Direction handling — represent absolute heading as an enum and keep a small
lookup table rather than branchy if/else chains (cleaner, and this is exactly
the kind of thing your DSA background will want to keep tidy):

```cpp
enum Direction : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

const int8_t DX[4] = { 0, 1, 0, -1 };  // NORTH, EAST, SOUTH, WEST
const int8_t DY[4] = { 1, 0, -1, 0 };
const uint8_t DIR_WALL_BIT[4] = { WALL_N, WALL_E, WALL_S, WALL_W };
const uint8_t OPPOSITE[4] = { SOUTH, WEST, NORTH, EAST };
```

### Recording a wall (and its mirror in the adjacent cell)
Walls are shared — if cell (3,3) has a wall on its East side, cell (4,3) must
have a wall on its West side. Always update both sides to keep the maze
self-consistent; a mismatch here is a classic and painful micromouse bug.

```cpp
void set_wall(uint8_t x, uint8_t y, Direction dir) {
    maze[x][y].walls |= DIR_WALL_BIT[dir];
    int8_t nx = x + DX[dir], ny = y + DY[dir];
    if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
        maze[nx][ny].walls |= DIR_WALL_BIT[OPPOSITE[dir]];
    }
}

bool has_wall(uint8_t x, uint8_t y, Direction dir) {
    return maze[x][y].walls & DIR_WALL_BIT[dir];
}
```

**Checkpoint 4.1:** Write and unit-test (on your desktop, not even the MCU —
this logic is hardware-independent, test it in plain C++ with a quick
`main()` and asserts before ever flashing it) `set_wall`/`has_wall`, confirm
mirrored-wall consistency across a handful of manually constructed cases,
including edge cells (x=0, y=15, etc., where "out of bounds" neighbors must
be handled safely).

---

## 4.2 Flood Fill Algorithm

Flood fill (a BFS variant) computes, for every cell, the shortest known
distance to the goal **given currently known walls** — recomputed every time
new walls are discovered. This is the textbook micromouse algorithm because
it's cheap to recompute incrementally and gives you an admissible "go this
way" decision at every cell without needing a full path precomputed upfront.

### Standard (non-diagonal) flood fill
Goal for a standard competition maze is the **center 2×2 block** (cells
(7,7),(7,8),(8,7),(8,8) for a 16×16 maze), not a single cell.

```cpp
void flood_fill(void) {
    // Initialize all to "infinity"
    for (int x = 0; x < MAZE_SIZE; x++)
        for (int y = 0; y < MAZE_SIZE; y++)
            maze[x][y].flood_value = 0xFFFF;

    // BFS queue — fixed-size ring buffer, NOT std::queue (no heap allocation)
    struct QItem { uint8_t x, y; };
    static QItem queue[MAZE_SIZE * MAZE_SIZE];
    int head = 0, tail = 0;

    // Seed with goal cells (multi-source BFS)
    const uint8_t goal_cells[4][2] = {{7,7},{7,8},{8,7},{8,8}};
    for (auto &g : goal_cells) {
        maze[g[0]][g[1]].flood_value = 0;
        queue[tail++] = { g[0], g[1] };
    }

    while (head < tail) {
        QItem cur = queue[head++];
        uint16_t dist = maze[cur.x][cur.y].flood_value;

        for (int d = 0; d < 4; d++) {
            if (has_wall(cur.x, cur.y, (Direction)d)) continue; // can't pass through a known wall
            int8_t nx = cur.x + DX[d], ny = cur.y + DY[d];
            if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
            if (maze[nx][ny].flood_value > dist + 1) {
                maze[nx][ny].flood_value = dist + 1;
                queue[tail++] = { (uint8_t)nx, (uint8_t)ny };
            }
        }
    }
}
```

Complexity: O(N) where N = 256 cells — trivial even on a 100MHz M4, runs in
low single-digit microseconds. You can afford to **recompute the entire flood
fill after every single new wall discovery** rather than trying to do
clever incremental updates — premature optimization here buys you nothing
and adds bug surface. This is a case where "the dumb thing is actually
correct engineering" for this scale of problem.

### Greedy navigation using the flood values
At each cell during the search run, look at all 4 neighbors not blocked by a
known wall, move toward the lowest `flood_value` neighbor (ties broken by a
consistent rule, e.g. prefer continuing straight to reduce turns/time):

```cpp
Direction choose_next_direction(uint8_t x, uint8_t y, Direction current_heading) {
    uint16_t best_val = 0xFFFF;
    Direction best_dir = current_heading;
    bool found = false;

    for (int d = 0; d < 4; d++) {
        if (has_wall(x, y, (Direction)d)) continue;
        int8_t nx = x + DX[d], ny = y + DY[d];
        if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
        uint16_t v = maze[nx][ny].flood_value;
        if (v < best_val || (v == best_val && d == current_heading)) {
            best_val = v;
            best_dir = (Direction)d;
            found = true;
        }
    }
    return found ? best_dir : current_heading; // no valid move = dead end, handled by caller
}
```

### The search loop (ties sensing + algorithm together)
```cpp
void search_run_step(void) {
    // 1. Sense walls at current cell using front/side sensors, call set_wall()
    //    for each detected wall relative to current absolute heading.
    sense_and_record_walls(mouse_x, mouse_y, mouse_heading);

    // 2. Recompute flood fill with latest wall knowledge.
    flood_fill();

    // 3. Decide and execute next move.
    Direction next = choose_next_direction(mouse_x, mouse_y, mouse_heading);
    if (next != mouse_heading) {
        turn_to(heading_to_theta(next));   // Phase 3 primitive
        mouse_heading = next;
    }
    move_forward_one_cell();               // Phase 3 primitive
    mouse_x += DX[next];
    mouse_y += DY[next];
    maze[mouse_x][mouse_y].visited = 1;
}
```

**Checkpoint 4.2:** Desktop-test `flood_fill()` against several hand-crafted
partial mazes (including one with a deliberately-blocked direct path,
verifying it correctly routes around). Confirm goal cells always reach
`flood_value = 0` and unreachable cells (fully walled off, if you construct
such a test) stay at `0xFFFF`.

---

## 4.3 Translating Path Into Motion — The Locomotion Bridge

This is the layer that turns "go EAST" into actual PID-driven motor commands,
and it's where search-run robustness lives or dies.

```cpp
void move_forward_one_cell(void) {
    float target_dist = CELL_SIZE_MM; // typically 180mm
    float start_dist = get_traveled_distance_mm(); // derived from odometry
    speed_pid.reset();
    while (get_traveled_distance_mm() - start_dist < target_dist) {
        // control loop (Phase 3) runs in background via TIM10 ISR/flag;
        // here we just update the target_speed_mm_s profile, e.g. trapezoidal:
        target_speed_mm_s = trapezoidal_profile(get_traveled_distance_mm() - start_dist,
                                                   target_dist, MAX_SPEED, ACCEL);
        check_wall_sensors_for_safety(); // abort/replan if unexpected wall detected mid-cell
    }
}
```

**Trapezoidal velocity profile** — don't command a step-function target speed
(instant 0→max), it saturates the PID and causes wheel slip/jerk. Ramp up,
cruise, ramp down as you approach the next cell boundary (especially
important right before a turn, where you want near-zero forward velocity to
turn cleanly in place, unless you're implementing smooth diagonal/corner-
cutting motion — an advanced Phase 5 topic).

```cpp
float trapezoidal_profile(float dist_so_far, float total_dist, float max_speed, float accel) {
    float decel_dist = (max_speed * max_speed) / (2 * accel);
    if (dist_so_far < decel_dist && dist_so_far < total_dist - decel_dist) {
        return sqrtf(2 * accel * dist_so_far);           // accelerating
    } else if (dist_so_far > total_dist - decel_dist) {
        float remaining = total_dist - dist_so_far;
        return sqrtf(fmaxf(2 * accel * remaining, 0.0f));  // decelerating
    }
    return max_speed;                                     // cruising
}
```

**Checkpoint 4.3:** Integrate sensing + flood fill + motion primitives and run
one full autonomous search of a small test maze (even a 4x4 subsection is
fine for initial validation), confirming the mouse reaches the goal without
manual intervention, and that discovered walls persist correctly across the
run.

---

## Phase 4 Summary — What You Should Now Be Able To Do
- Represent the maze with fixed-size, statically-allocated structures
  appropriate for an embedded target (no heap in the control path).
- Implement multi-source BFS flood fill correctly, including wall-mirroring
  consistency and edge/goal handling.
- Drive a full sense → recompute → decide → move loop that ties Phase 2
  sensing and Phase 3 motion control to the maze-solving logic.
- Use a trapezoidal velocity profile instead of step commands for smoother,
  more accurate cell-to-cell motion.

Confirm and we'll move to **[Phase 5: Competition
Optimization](05-phase5-competition-optimization.md)** — the search-run/
fast-run state machine, speed tuning strategy, and the debugging/robustness
work that actually wins or loses runs at competition.
