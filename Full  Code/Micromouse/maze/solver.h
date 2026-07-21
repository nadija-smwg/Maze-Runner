/**
 * @file solver.h
 * @brief Top-level maze solver orchestrator — ties flood fill, weighted
 *        Dijkstra, and path smoother together into a clean API for the
 *        robot's state machine.
 *
 * Usage:
 *   1. Call solver_init() at startup.
 *   2. During search run: call solver_search_step() once per cell.
 *      It returns the next direction to move. Caller does the physical move.
 *   3. When search run reaches the goal: call solver_compute_fast_path().
 *   4. During fast run: call solver_get_next_command() repeatedly.
 */

#ifndef SOLVER_H
#define SOLVER_H

#include "maze.h"
#include "flood_fill.h"
#include "dijkstra_weighted.h"
#include "path_smoother.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Solver operating mode. */
typedef enum {
    SOLVER_SEARCH,      /**< Exploring unknown maze (flood fill)      */
    SOLVER_FAST_RUN,    /**< Executing precomputed optimal path       */
    SOLVER_IDLE         /**< Not actively solving                     */
} SolverMode;

/** Full solver state. */
typedef struct {
    MazeMap        maze;            /**< The maze map being explored       */
    uint8_t        mouse_x;         /**< Current cell X                    */
    uint8_t        mouse_y;         /**< Current cell Y                    */
    Direction      mouse_heading;   /**< Current absolute heading          */
    SolverMode     mode;            /**< Current operating mode            */

    /* Fast-run data (populated by solver_compute_fast_path). */
    DijkstraResult dijkstra_result; /**< Raw optimal path                  */
    SmoothedPath   smooth_path;     /**< Optimized motion commands         */
    uint16_t       cmd_index;       /**< Next command to execute           */
} Solver;

/* ─── Initialization ──────────────────────────────────────────────────── */

/**
 * Initialize the solver: set start position, unknown maze, search mode.
 */
void solver_init(Solver *s);

/* ─── Search Run (Tier 1: Flood Fill) ─────────────────────────────────── */

/**
 * Record walls detected at the current cell.
 * Call this after the robot's sensors have been read.
 *
 * @param s          The solver state
 * @param front_wall True if a wall is detected in front of the mouse
 * @param left_wall  True if a wall is detected to the left
 * @param right_wall True if a wall is detected to the right
 */
void solver_record_walls(Solver *s,
                         bool front_wall,
                         bool left_wall,
                         bool right_wall);

/**
 * Compute the next direction to move during the search run.
 * Internally recomputes flood fill, then chooses the best neighbor.
 *
 * After calling this, the robot should:
 *   1. Turn to the returned direction (if different from current heading).
 *   2. Move forward one cell.
 *   3. Call solver_advance() to update the solver's position.
 *
 * @param s The solver state
 * @return  The direction the mouse should face next
 */
Direction solver_search_step(Solver *s);

/**
 * Advance the solver's mouse position by one cell in direction `dir`.
 * Call this AFTER the physical move completes.
 */
void solver_advance(Solver *s, Direction dir);

/**
 * Check if the mouse is currently at a goal cell.
 */
bool solver_at_goal(const Solver *s);

/**
 * Check if the mouse is at the start cell.
 */
bool solver_at_start(const Solver *s);

/* ─── Fast Run (Tier 2+3: Dijkstra + Smoother) ────────────────────────── */

/**
 * Compute the time-optimal fast-run path using the fully explored maze.
 * Runs weighted Dijkstra from start cell to goal, then smooths the path.
 *
 * @param s          The solver state (maze must be explored)
 * @param from_x     Starting X for the fast run
 * @param from_y     Starting Y for the fast run
 * @param heading    Starting heading for the fast run
 * @return           True if a path was found
 */
bool solver_compute_fast_path(Solver *s,
                              uint8_t from_x, uint8_t from_y,
                              Direction heading);

/**
 * Get the next motion command in the fast-run sequence.
 * Returns NULL when all commands have been consumed (fast run complete).
 */
const MotionCommand* solver_get_next_command(Solver *s);

/**
 * Check if the fast run has more commands remaining.
 */
bool solver_fast_run_complete(const Solver *s);

/* ─── Debug / Telemetry ───────────────────────────────────────────────── */

/**
 * Get the flood value of cell (x,y) — useful for UART/SWO telemetry.
 */
uint16_t solver_get_flood_value(const Solver *s, uint8_t x, uint8_t y);

/**
 * Get the total cost of the computed fast-run path.
 */
uint16_t solver_get_fast_run_cost(const Solver *s);

#ifdef __cplusplus
}
#endif

#endif /* SOLVER_H */
