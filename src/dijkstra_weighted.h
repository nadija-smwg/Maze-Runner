/**
 * @file dijkstra_weighted.h
 * @brief Weighted Dijkstra pathfinder with turn penalties for fast-run
 *        optimal path computation.
 *
 * Tier 2 algorithm: state = (x, y, heading), 16×16×4 = 1024 states.
 * Uses a fixed-size binary min-heap as priority queue — zero heap allocation.
 *
 * The key insight: a path with fewer turns is faster even if it's longer
 * in cell count, because each 90° turn costs ~150ms of decel+rotate+accel
 * while a straight cell traversal at cruise costs ~100ms.
 */

#ifndef DIJKSTRA_WEIGHTED_H
#define DIJKSTRA_WEIGHTED_H

#include "maze.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Result of a Dijkstra computation. */
typedef struct {
    Waypoint path[MAX_PATH_LENGTH];  /**< ordered start→goal waypoints     */
    uint16_t path_length;            /**< number of waypoints in `path`    */
    uint16_t total_cost;             /**< total weighted cost of the path  */
    bool     found;                  /**< true if a path to goal exists    */
} DijkstraResult;

/**
 * Compute the time-optimal path from (start_x, start_y, start_heading)
 * to any of the goal cells, using weighted costs that penalize turns.
 *
 * Cost model (from config.h):
 *   - Moving straight into the next cell: COST_STRAIGHT
 *   - Moving after a 90° turn:  COST_STRAIGHT + COST_TURN_90
 *   - Moving after a 180° turn: COST_STRAIGHT + COST_TURN_180
 *
 * The maze must be fully (or partially) explored; only known walls are
 * respected. Call this after the search run completes for the best result.
 *
 * @param m              Pointer to the explored MazeMap
 * @param start_x        Starting cell X coordinate
 * @param start_y        Starting cell Y coordinate
 * @param start_heading  Starting heading direction
 * @param goal_cells     Array of [x,y] goal coordinates
 * @param num_goals      Number of goal cells
 * @param result         Output: the optimal path and its cost
 */
void dijkstra_compute(const MazeMap *m,
                      uint8_t start_x, uint8_t start_y,
                      Direction start_heading,
                      const uint8_t goal_cells[][2],
                      uint8_t num_goals,
                      DijkstraResult *result);

#ifdef __cplusplus
}
#endif

#endif /* DIJKSTRA_WEIGHTED_H */
