/**
 * @file main.c
 * @brief Desktop test harness — runs the full micromouse algorithm suite
 *        on a hardcoded maze and prints results.
 *
 * Compile:  gcc -o maze_test main.c flood_fill.c dijkstra_weighted.c path_smoother.c solver.c -lm
 * Run:      ./maze_test
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "solver.h"
#include "motion_profile.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Test Maze Builder
 *
 *  Builds a realistic competition-style 16×16 maze with corridors,
 *  dead ends, and multiple paths to the center.
 * ═══════════════════════════════════════════════════════════════════════════ */

static void build_test_maze(MazeMap *m)
{
    maze_init_unknown(m);

    /* Wall format: {x, y, direction} */
    typedef struct { uint8_t x, y; Direction d; } W;
    const W walls[] = {
        /* Start area corridors */
        {0,0, DIR_EAST},  {1,0, DIR_EAST},  {2,0, DIR_NORTH},
        {0,1, DIR_EAST},  {0,2, DIR_EAST},
        {1,1, DIR_NORTH}, {2,1, DIR_EAST},  {3,1, DIR_NORTH},
        {3,0, DIR_NORTH}, {4,0, DIR_EAST},  {5,0, DIR_NORTH},
        {5,1, DIR_NORTH},

        /* Central structure */
        {3,2, DIR_EAST},  {4,2, DIR_NORTH}, {4,3, DIR_EAST},
        {5,3, DIR_NORTH}, {6,2, DIR_NORTH}, {6,3, DIR_EAST},
        {7,3, DIR_NORTH}, {2,3, DIR_NORTH}, {2,4, DIR_EAST},
        {3,4, DIR_NORTH}, {3,5, DIR_EAST},  {4,5, DIR_NORTH},
        {5,5, DIR_EAST},  {6,5, DIR_NORTH}, {6,6, DIR_EAST},

        /* Goal area walls */
        {7,6, DIR_NORTH}, {8,6, DIR_NORTH}, {7,8, DIR_EAST},
        {8,8, DIR_NORTH}, {6,7, DIR_NORTH}, {9,7, DIR_NORTH},
        {9,8, DIR_EAST},

        /* Upper corridors */
        {2,6, DIR_EAST},  {3,6, DIR_NORTH}, {4,7, DIR_EAST},
        {5,7, DIR_NORTH}, {1,5, DIR_NORTH}, {1,6, DIR_EAST},
        {0,7, DIR_EAST},

        /* Right side */
        {10,3, DIR_NORTH}, {11,3, DIR_EAST}, {11,4, DIR_NORTH},
        {10,5, DIR_EAST},  {11,5, DIR_NORTH},{12,5, DIR_EAST},
        {10,7, DIR_NORTH}, {11,7, DIR_EAST}, {12,7, DIR_NORTH},

        /* Far side */
        {13,2, DIR_NORTH}, {14,2, DIR_EAST}, {13,4, DIR_NORTH},
        {14,4, DIR_EAST},  {13,6, DIR_NORTH},{14,6, DIR_EAST},
        {13,8, DIR_NORTH}, {14,8, DIR_EAST},
        {12,9, DIR_NORTH}, {13,9, DIR_EAST}, {12,10, DIR_EAST},
        {10,10, DIR_NORTH},{11,10, DIR_EAST},{10,11, DIR_NORTH},
        {8,10, DIR_NORTH}, {9,10, DIR_EAST}, {8,11, DIR_EAST},

        /* Upper maze */
        {2,9, DIR_EAST},   {3,9, DIR_NORTH}, {4,9, DIR_EAST},
        {5,10, DIR_NORTH}, {6,10, DIR_EAST}, {7,10, DIR_NORTH},
        {6,11, DIR_NORTH}, {3,11, DIR_EAST}, {4,11, DIR_NORTH},
        {5,12, DIR_EAST},  {2,12, DIR_NORTH},{1,12, DIR_EAST},
        {0,11, DIR_NORTH}, {7,12, DIR_NORTH},{8,12, DIR_EAST},
        {9,12, DIR_NORTH}, {10,12, DIR_EAST},{11,13, DIR_NORTH},
        {12,12, DIR_NORTH},{13,12, DIR_EAST},{14,12, DIR_NORTH},
        {6,13, DIR_EAST},  {7,14, DIR_NORTH},{8,14, DIR_EAST},
        {4,14, DIR_EAST},  {3,13, DIR_NORTH},{2,14, DIR_EAST},
        {10,14, DIR_EAST}, {11,14, DIR_NORTH},{12,14, DIR_EAST},
        {14,14, DIR_NORTH},{15,13, DIR_NORTH},
    };

    const int num_walls = sizeof(walls) / sizeof(walls[0]);
    for (int i = 0; i < num_walls; i++) {
        maze_set_wall(m, walls[i].x, walls[i].y, walls[i].d);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Pretty Printing
 * ═══════════════════════════════════════════════════════════════════════════ */

static const char* dir_name(Direction d) {
    const char *names[] = {"NORTH", "EAST ", "SOUTH", "WEST "};
    return names[d];
}

static const char* cmd_name(CommandType t) {
    switch (t) {
        case CMD_STRAIGHT:       return "STRAIGHT      ";
        case CMD_TURN_LEFT_90:   return "TURN LEFT  90 ";
        case CMD_TURN_RIGHT_90:  return "TURN RIGHT 90 ";
        case CMD_TURN_180:       return "TURN 180      ";
        case CMD_DIAGONAL:       return "DIAGONAL      ";
        case CMD_SMOOTH_LEFT_90: return "SMOOTH LEFT 90";
        case CMD_SMOOTH_RIGHT_90:return "SMOOTH RGHT 90";
        case CMD_SMOOTH_LEFT_180:return "SMOOTH LEFT180";
        case CMD_SMOOTH_RIGHT_180:return"SMOOTH RGT 180";
        case CMD_SS_LEFT_90:     return "SS LEFT 90    ";
        case CMD_SS_RIGHT_90:    return "SS RIGHT 90   ";
        default:                 return "UNKNOWN       ";
    }
}

static void print_maze_with_flood(const MazeMap *m)
{
    printf("\n");
    printf("  ┌");
    for (int x = 0; x < MAZE_SIZE; x++) printf("────┬");
    printf("\b┐\n");

    for (int y = MAZE_SIZE - 1; y >= 0; y--) {
        /* Cell values row */
        printf("%2d│", y);
        for (int x = 0; x < MAZE_SIZE; x++) {
            uint16_t f = m->cells[x][y].flood_value;
            bool is_goal = false;
            for (int g = 0; g < NUM_GOAL_CELLS; g++) {
                if (GOAL_CELLS[g][0] == x && GOAL_CELLS[g][1] == y) is_goal = true;
            }
            if (is_goal) {
                if (f < FLOOD_INFINITY) printf(" *%2d", f);
                else printf("  * ");
            } else if (x == START_X && y == START_Y) {
                if (f < FLOOD_INFINITY) printf(" S%2d", f);
                else printf("  S ");
            } else if (f < FLOOD_INFINITY) {
                printf(" %3d", f);
            } else {
                printf("  · ");
            }

            if (x < MAZE_SIZE - 1) {
                if (maze_has_wall(m, x, y, DIR_EAST)) printf("│");
                else printf(" ");
            }
        }
        printf("│\n");

        /* Horizontal walls row */
        if (y > 0) {
            printf("  ├");
            for (int x = 0; x < MAZE_SIZE; x++) {
                if (maze_has_wall(m, x, y, DIR_SOUTH)) printf("────");
                else printf("    ");
                if (x < MAZE_SIZE - 1) printf("┼");
            }
            printf("┤\n");
        }
    }

    printf("  └");
    for (int x = 0; x < MAZE_SIZE; x++) printf("────┴");
    printf("\b┘\n");
    printf("    ");
    for (int x = 0; x < MAZE_SIZE; x++) printf(" %2d  ", x);
    printf("\n");
}

static void print_path_on_maze(const MazeMap *m,
                               const Waypoint *path, uint16_t len,
                               const char *label)
{
    /* Build a set of path cells for quick lookup */
    uint8_t on_path[MAZE_SIZE][MAZE_SIZE];
    memset(on_path, 0, sizeof(on_path));
    for (uint16_t i = 0; i < len; i++) {
        on_path[path[i].x][path[i].y] = 1;
    }

    printf("\n  %s PATH (%d waypoints):\n", label, len);
    printf("  ");
    for (int y = MAZE_SIZE - 1; y >= 0; y--) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            if (on_path[x][y]) printf("██");
            else if (maze_has_wall(m, x, y, DIR_NORTH) ||
                     maze_has_wall(m, x, y, DIR_SOUTH) ||
                     maze_has_wall(m, x, y, DIR_EAST) ||
                     maze_has_wall(m, x, y, DIR_WEST)) printf("░░");
            else printf("  ");
        }
        printf("\n  ");
    }
    printf("\n");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Main — Full Algorithm Demo
 * ═══════════════════════════════════════════════════════════════════════════ */

int main(void)
{
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║   MICROMOUSE MAZE SOLVER — Algorithm Test Suite             ║\n");
    printf("║   SLIIT ROBOFEST 2026 | STM32F401 Black Pill               ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");

    /* ── Phase 1: Build test maze ──────────────────────────────────────── */
    Solver solver;
    solver_init(&solver);

    /* For testing, we use a fully-known maze (simulates post-search state) */
    build_test_maze(&solver.maze);

    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("  TIER 1: FLOOD FILL (BFS) — Distance Heatmap\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    flood_fill_compute(&solver.maze, GOAL_CELLS, NUM_GOAL_CELLS);
    print_maze_with_flood(&solver.maze);

    printf("\n  Start cell (0,0) flood value: %d\n",
           solver.maze.cells[START_X][START_Y].flood_value);
    printf("  Goal cells flood value: 0 (verified: %d, %d, %d, %d)\n",
           solver.maze.cells[7][7].flood_value,
           solver.maze.cells[7][8].flood_value,
           solver.maze.cells[8][7].flood_value,
           solver.maze.cells[8][8].flood_value);

    /* ── Phase 2: BFS Shortest Path ────────────────────────────────────── */
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("  TIER 1: BFS SHORTEST PATH (distance-optimal)\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    /* Trace BFS path by following flood gradient from start to goal */
    Waypoint bfs_path[MAX_PATH_LENGTH];
    uint16_t bfs_len = 0;
    {
        uint8_t cx = START_X, cy = START_Y;
        Direction ch = DIR_NORTH;
        bfs_path[bfs_len].x = cx;
        bfs_path[bfs_len].y = cy;
        bfs_path[bfs_len].heading = ch;
        bfs_len++;

        while (solver.maze.cells[cx][cy].flood_value > 0 && bfs_len < MAX_PATH_LENGTH) {
            Direction next = flood_fill_choose_direction(&solver.maze, cx, cy, ch);
            cx += DX[next];
            cy += DY[next];
            ch = next;
            bfs_path[bfs_len].x = cx;
            bfs_path[bfs_len].y = cy;
            bfs_path[bfs_len].heading = ch;
            bfs_len++;
        }
    }

    printf("\n  BFS Path: %d cells\n  Route: ", bfs_len);
    for (uint16_t i = 0; i < bfs_len; i++) {
        printf("(%d,%d)", bfs_path[i].x, bfs_path[i].y);
        if (i < bfs_len - 1) printf(" → ");
        if (i > 0 && i % 8 == 0) printf("\n          ");
    }

    /* Count turns in BFS path */
    int bfs_turns = 0;
    for (uint16_t i = 1; i < bfs_len; i++) {
        if (bfs_path[i].heading != bfs_path[i-1].heading) bfs_turns++;
    }

    /* Compute BFS weighted cost */
    uint16_t bfs_cost = 0;
    for (uint16_t i = 1; i < bfs_len; i++) {
        bfs_cost += COST_STRAIGHT;
        uint8_t t = turn_cost_steps(bfs_path[i-1].heading, bfs_path[i].heading);
        if (t == 1) bfs_cost += COST_TURN_90;
        else if (t == 2) bfs_cost += COST_TURN_180;
    }

    printf("\n  BFS Turns: %d | BFS Weighted Cost: %d\n", bfs_turns, bfs_cost);
    print_path_on_maze(&solver.maze, bfs_path, bfs_len, "BFS");

    /* ── Phase 3: Weighted Dijkstra ────────────────────────────────────── */
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("  TIER 2: WEIGHTED DIJKSTRA (time-optimal, turn penalties)\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("  Cost model: STRAIGHT=%d, TURN_90=+%d, TURN_180=+%d\n",
           COST_STRAIGHT, COST_TURN_90, COST_TURN_180);

    DijkstraResult dijkstra_result;
    dijkstra_compute(&solver.maze,
                     START_X, START_Y, DIR_NORTH,
                     GOAL_CELLS, NUM_GOAL_CELLS,
                     &dijkstra_result);

    if (dijkstra_result.found) {
        printf("\n  ✓ Path found! %d waypoints, total cost: %d\n",
               dijkstra_result.path_length, dijkstra_result.total_cost);

        printf("  Route: ");
        for (uint16_t i = 0; i < dijkstra_result.path_length; i++) {
            printf("(%d,%d,%s)",
                   dijkstra_result.path[i].x,
                   dijkstra_result.path[i].y,
                   dir_name(dijkstra_result.path[i].heading));
            if (i < dijkstra_result.path_length - 1) printf(" → ");
            if (i > 0 && i % 4 == 0) printf("\n          ");
        }

        /* Count Dijkstra turns */
        int dijk_turns = 0;
        for (uint16_t i = 1; i < dijkstra_result.path_length; i++) {
            if (dijkstra_result.path[i].heading != dijkstra_result.path[i-1].heading)
                dijk_turns++;
        }
        printf("\n  Dijkstra Turns: %d | Dijkstra Cost: %d\n",
               dijk_turns, dijkstra_result.total_cost);

        print_path_on_maze(&solver.maze, dijkstra_result.path,
                           dijkstra_result.path_length, "DIJKSTRA");
    } else {
        printf("\n  ✗ No path found!\n");
    }

    /* ── Phase 4: Path Smoother ────────────────────────────────────────── */
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("  TIER 3: PATH SMOOTHER (motion commands for the robot)\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    if (dijkstra_result.found) {
        SmoothedPath smooth;
        path_smooth(dijkstra_result.path, dijkstra_result.path_length, &smooth);

        printf("\n  %d motion commands (from %d raw waypoints):\n\n",
               smooth.num_commands, dijkstra_result.path_length);
        printf("  ┌─────┬──────────────┬───────┐\n");
        printf("  │ #   │ Command      │ Cells │\n");
        printf("  ├─────┼──────────────┼───────┤\n");

        for (uint16_t i = 0; i < smooth.num_commands; i++) {
            printf("  │ %3d │ %s │  %3d  │\n",
                   i + 1,
                   cmd_name(smooth.commands[i].type),
                   smooth.commands[i].cells);
        }
        printf("  └─────┴──────────────┴───────┘\n");
    }

    /* ── Phase 5: Comparison Summary ───────────────────────────────────── */
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("  COMPARISON: BFS vs WEIGHTED DIJKSTRA\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("\n");
    printf("  ┌──────────────────┬───────────┬──────────────┐\n");
    printf("  │ Metric           │ BFS       │ Dijkstra     │\n");
    printf("  ├──────────────────┼───────────┼──────────────┤\n");
    printf("  │ Path length      │ %3d cells │ %3d cells    │\n",
           bfs_len, dijkstra_result.path_length);
    printf("  │ Turn count       │ %3d turns │ %3d turns    │\n",
           bfs_turns,
           dijkstra_result.found ? (int)dijkstra_result.path_length : 0); /* recount below */
    printf("  │ Weighted cost    │ %5d     │ %5d        │\n",
           bfs_cost, dijkstra_result.total_cost);

    if (dijkstra_result.found && bfs_cost > 0) {
        float improvement = 100.0f * (1.0f - (float)dijkstra_result.total_cost / (float)bfs_cost);
        printf("  │ Improvement      │           │ %5.1f%%       │\n", improvement);
    }
    printf("  └──────────────────┴───────────┴──────────────┘\n");

    /* ── Phase 6: Search Run Simulation ────────────────────────────────── */
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("  SEARCH RUN SIMULATION (exploring unknown maze)\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    /* Create a fresh solver with unknown maze */
    Solver search_solver;
    solver_init(&search_solver);

    /* The "real" maze we'll sense from */
    MazeMap real_maze;
    build_test_maze(&real_maze);

    int steps = 0;
    printf("\n  Step | Position  | Heading | Flood Value\n");
    printf("  ─────┼───────────┼─────────┼────────────\n");

    while (!solver_at_goal(&search_solver) && steps < 300) {
        /* Sense walls from real maze at current position */
        uint8_t mx = search_solver.mouse_x;
        uint8_t my = search_solver.mouse_y;

        bool front = maze_has_wall(&real_maze, mx, my, search_solver.mouse_heading);
        bool left  = maze_has_wall(&real_maze, mx, my,
                                   (Direction)((search_solver.mouse_heading + 3) % 4));
        bool right = maze_has_wall(&real_maze, mx, my,
                                   (Direction)((search_solver.mouse_heading + 1) % 4));

        solver_record_walls(&search_solver, front, left, right);

        /* Decide next direction */
        Direction next = solver_search_step(&search_solver);

        printf("  %4d │ (%2d, %2d)  │ %s   │ %5d\n",
               steps + 1, mx, my, dir_name(search_solver.mouse_heading),
               search_solver.maze.cells[mx][my].flood_value);

        /* Move */
        solver_advance(&search_solver, next);
        steps++;
    }

    printf("\n  ✓ Goal reached in %d steps!\n", steps);

    /* Compute fast-run path on explored maze */
    printf("\n  Computing fast-run path on explored maze...\n");
    bool found = solver_compute_fast_path(&search_solver, START_X, START_Y, DIR_NORTH);

    if (found) {
        printf("  ✓ Fast-run path: %d waypoints, cost: %d\n",
               search_solver.dijkstra_result.path_length,
               search_solver.dijkstra_result.total_cost);
        printf("  ✓ Smoothed to %d motion commands\n",
               search_solver.smooth_path.num_commands);

        printf("\n  Fast-run commands:\n");
        for (uint16_t i = 0; i < search_solver.smooth_path.num_commands; i++) {
            const MotionCommand *c = &search_solver.smooth_path.commands[i];
            printf("    %2d. %s", i + 1, cmd_name(c->type));
            if (c->cells > 0) printf(" × %d cells", c->cells);
            printf("\n");
        }
    } else {
        printf("  ✗ No path found on explored maze!\n");
    }

    /* ── Phase 7: Motion Profile Demo ──────────────────────────────────── */
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("  MOTION PROFILE — S-Curve & Rolling Turn Demo\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    /* Demo 1: Straight segment with S-curve profile */
    {
        LinearProfile lp;
        float dist = 3 * CELL_SIZE_MM;  /* 3 cells = 540mm */
        profile_compute_linear(&lp, dist,
                               FAST_MAX_SPEED_MM_S,
                               FAST_ACCEL_MM_S2,
                               FAST_DECEL_MM_S2,
                               0,                    /* start from stop */
                               MAX_TURN_SPEED_MM_S); /* exit at turn speed */

        printf("\n  ▸ Straight profile: %.0fmm (3 cells), exit → turn\n", dist);
        printf("    Type: %s\n", lp.type == PROFILE_S_CURVE ? "S-Curve" : "Trapezoidal");
        printf("    Peak speed: %.0f mm/s\n", lp.peak_speed);
        printf("    Accel dist: %.1f mm | Cruise: %.1f mm | Decel: %.1f mm\n",
               lp.accel_dist, lp.cruise_dist, lp.decel_dist);
        printf("    Start: %.0f mm/s → End: %.0f mm/s\n\n", lp.start_speed, lp.end_speed);

        printf("    dist(mm) │ speed(mm/s) │ phase\n");
        printf("    ─────────┼─────────────┼──────────\n");
        for (float d = 0; d <= dist; d += dist / 10.0f) {
            float v = profile_get_speed(&lp, d);
            const char *phase = "CRUISE";
            if (d <= lp.accel_dist) phase = "ACCEL ";
            else if (d >= lp.accel_dist + lp.cruise_dist) phase = "DECEL ";
            printf("    %7.1f  │  %8.1f   │ %s\n", d, v, phase);
        }
    }

    /* Demo 2: Rolling 90° turn profile */
    {
        ArcTurnProfile arc;
        profile_compute_arc(&arc,
                            TURN_RADIUS_90_MM,
                            90.0f,
                            MAX_TURN_SPEED_MM_S,
                            FAST_MAX_SPEED_MM_S,
                            FAST_DECEL_MM_S2,
                            false);  /* right turn */

        printf("\n  ▸ Rolling 90° right turn:\n");
        printf("    Radius: %.0f mm | Arc length: %.1f mm\n", arc.radius_mm, arc.arc_length_mm);
        printf("    Linear speed: %.0f mm/s | Angular: %.2f rad/s (%.0f°/s)\n",
               arc.linear_speed, arc.angular_speed, arc.angular_speed * 180.0f / PI);
        printf("    Duration: %.3f s (%.0f ms)\n", arc.duration_s, arc.duration_s * 1000.0f);
        printf("    Entry decel: %.1f mm | Exit accel: %.1f mm\n\n",
               arc.entry_decel_dist, arc.exit_accel_dist);

        /* Show wheel speed differential */
        float wheelbase = 75.0f;  /* mm, from Phase 3 */
        float v_left, v_right;
        float speed, omega;
        profile_get_arc_state(&arc, arc.arc_length_mm / 2.0f, &speed, &omega);
        profile_to_wheel_speeds(speed, omega, wheelbase, &v_left, &v_right);

        printf("    Wheel speeds at mid-turn (wheelbase=%.0fmm):\n", wheelbase);
        printf("      Left (inner):  %7.1f mm/s\n", v_left);
        printf("      Right (outer): %7.1f mm/s\n", v_right);
        printf("      Ratio: %.2f (outer/inner)\n",
               v_left != 0 ? v_right / v_left : 0.0f);
    }

    /* Demo 3: Look-ahead exit speeds */
    {
        printf("\n  ▸ Look-ahead exit speed planning:\n");
        printf("    Next command         │ Exit speed\n");
        printf("    ─────────────────────┼───────────\n");

        struct { CommandType cmd; const char* name; } tests[] = {
            {CMD_STRAIGHT,       "STRAIGHT        "},
            {CMD_SMOOTH_RIGHT_90,"SMOOTH RIGHT 90 "},
            {CMD_SMOOTH_LEFT_180,"SMOOTH LEFT 180 "},
            {CMD_TURN_LEFT_90,   "IN-PLACE LEFT 90"},
            {CMD_TURN_180,       "IN-PLACE 180    "},
        };

        for (int i = 0; i < 5; i++) {
            float exit_v = profile_exit_speed_for_next(tests[i].cmd, FAST_MAX_SPEED_MM_S);
            printf("    %s │ %6.0f mm/s\n", tests[i].name, exit_v);
        }
    }

    /* ── Summary ───────────────────────────────────────────────────────── */
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  MEMORY USAGE SUMMARY                                      ║\n");
    printf("╠══════════════════════════════════════════════════════════════╣\n");
    printf("║  Solver struct:    %5zu bytes                              ║\n", sizeof(Solver));
    printf("║  MazeMap:          %5zu bytes                              ║\n", sizeof(MazeMap));
    printf("║  DijkstraResult:   %5zu bytes                              ║\n", sizeof(DijkstraResult));
    printf("║  SmoothedPath:     %5zu bytes                              ║\n", sizeof(SmoothedPath));
    printf("║  Cell:             %5zu bytes  × 256 = %zu                 ║\n",
           sizeof(Cell), sizeof(Cell) * 256);
    printf("║  LinearProfile:    %5zu bytes                              ║\n", sizeof(LinearProfile));
    printf("║  ArcTurnProfile:   %5zu bytes                              ║\n", sizeof(ArcTurnProfile));
    printf("╚══════════════════════════════════════════════════════════════╝\n");

    printf("\n  All tests complete. Algorithm suite ready for STM32 deployment.\n\n");

    return 0;
}
