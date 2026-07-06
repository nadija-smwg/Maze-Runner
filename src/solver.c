/**
 * @file solver.c
 * @brief Top-level solver orchestrator implementation.
 */

#include "solver.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialization
 * ═══════════════════════════════════════════════════════════════════════════ */

void solver_init(Solver *s) {
    maze_init_unknown(&s->maze);
    s->mouse_x = START_X;
    s->mouse_y = START_Y;
    s->mouse_heading = DIR_NORTH;
    s->mode = SOLVER_SEARCH;
    s->cmd_index = 0;
    s->dijkstra_result.found = false;
    s->dijkstra_result.path_length = 0;
    s->smooth_path.num_commands = 0;

    /* Mark start cell as visited. */
    s->maze.cells[START_X][START_Y].visited = 1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Wall Recording
 *
 *  Converts sensor readings (front/left/right relative to mouse heading)
 *  into absolute wall directions and records them in the maze.
 * ═══════════════════════════════════════════════════════════════════════════ */

void solver_record_walls(Solver *s,
                         bool front_wall,
                         bool left_wall,
                         bool right_wall)
{
    Direction heading = s->mouse_heading;
    uint8_t x = s->mouse_x;
    uint8_t y = s->mouse_y;

    /* Front = current heading. */
    if (front_wall) {
        maze_set_wall(&s->maze, x, y, heading);
    }

    /* Left = (heading + 3) % 4    i.e. counter-clockwise. */
    Direction left_dir = (Direction)((heading + 3) % 4);
    if (left_wall) {
        maze_set_wall(&s->maze, x, y, left_dir);
    }

    /* Right = (heading + 1) % 4   i.e. clockwise. */
    Direction right_dir = (Direction)((heading + 1) % 4);
    if (right_wall) {
        maze_set_wall(&s->maze, x, y, right_dir);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Search Step (Flood Fill)
 * ═══════════════════════════════════════════════════════════════════════════ */

Direction solver_search_step(Solver *s) {
    /* Recompute flood fill from goal cells with latest wall knowledge. */
    flood_fill_compute(&s->maze, GOAL_CELLS, NUM_GOAL_CELLS);

    /* Choose best direction. */
    return flood_fill_choose_direction(&s->maze,
                                      s->mouse_x, s->mouse_y,
                                      s->mouse_heading);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Position Advancement
 * ═══════════════════════════════════════════════════════════════════════════ */

void solver_advance(Solver *s, Direction dir) {
    s->mouse_heading = dir;
    int8_t new_x = (int8_t)s->mouse_x + DX[dir];
    int8_t new_y = (int8_t)s->mouse_y + DY[dir];

    /* Bounds safety — should never happen if walls are correct,
     * but prevents catastrophic array-out-of-bounds on bad sensor data. */
    if (new_x < 0 || new_x >= MAZE_SIZE || new_y < 0 || new_y >= MAZE_SIZE)
        return;  /* reject invalid move */

    s->mouse_x = (uint8_t)new_x;
    s->mouse_y = (uint8_t)new_y;
    s->maze.cells[s->mouse_x][s->mouse_y].visited = 1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Goal / Start Checks
 * ═══════════════════════════════════════════════════════════════════════════ */

bool solver_at_goal(const Solver *s) {
    for (uint8_t g = 0; g < NUM_GOAL_CELLS; g++) {
        if (s->mouse_x == GOAL_CELLS[g][0] &&
            s->mouse_y == GOAL_CELLS[g][1]) {
            return true;
        }
    }
    return false;
}

bool solver_at_start(const Solver *s) {
    return s->mouse_x == START_X && s->mouse_y == START_Y;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Fast-Run Path Computation (Dijkstra + Smoother)
 * ═══════════════════════════════════════════════════════════════════════════ */

bool solver_compute_fast_path(Solver *s,
                              uint8_t from_x, uint8_t from_y,
                              Direction heading)
{
    /* Run weighted Dijkstra on the (now fully/partially explored) maze. */
    dijkstra_compute(&s->maze,
                     from_x, from_y, heading,
                     GOAL_CELLS, NUM_GOAL_CELLS,
                     &s->dijkstra_result);

    if (!s->dijkstra_result.found) {
        s->smooth_path.num_commands = 0;
        return false;
    }

    /* Smooth the raw waypoint path into optimized motion commands. */
    path_smooth(s->dijkstra_result.path,
                s->dijkstra_result.path_length,
                &s->smooth_path);

    s->cmd_index = 0;
    s->mode = SOLVER_FAST_RUN;

    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Fast-Run Command Iteration
 * ═══════════════════════════════════════════════════════════════════════════ */

const MotionCommand* solver_get_next_command(Solver *s) {
    if (s->cmd_index >= s->smooth_path.num_commands)
        return (const MotionCommand*)0;  /* NULL — fast run complete */

    return &s->smooth_path.commands[s->cmd_index++];
}

bool solver_fast_run_complete(const Solver *s) {
    return s->cmd_index >= s->smooth_path.num_commands;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Debug / Telemetry
 * ═══════════════════════════════════════════════════════════════════════════ */

uint16_t solver_get_flood_value(const Solver *s, uint8_t x, uint8_t y) {
    if (x >= MAZE_SIZE || y >= MAZE_SIZE)
        return FLOOD_INFINITY;
    return s->maze.cells[x][y].flood_value;
}

uint16_t solver_get_fast_run_cost(const Solver *s) {
    return s->dijkstra_result.total_cost;
}
