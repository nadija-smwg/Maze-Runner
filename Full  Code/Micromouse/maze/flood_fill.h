/**
 * @file flood_fill.h
 * @brief Modified Flood Fill (multi-source BFS) for micromouse search-run navigation.
 *
 * Tier 1 algorithm: explores unknown maze by computing shortest Manhattan-ish
 * distance from every cell to the goal, recomputed each time new walls are
 * discovered. Uses a fixed-size ring buffer queue — zero heap allocation.
 */

#ifndef FLOOD_FILL_H
#define FLOOD_FILL_H

#include "maze.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Recompute flood fill distances from scratch.
 * Seeds BFS from `goal_cells` (array of [x,y] pairs, `num_goals` entries).
 * After return, every reachable cell's `flood_value` is its shortest
 * distance (in cells) to the nearest goal cell, respecting known walls.
 *
 * Complexity: O(MAZE_SIZE²) = O(256), runs in < 10µs on Cortex-M4 @ 100MHz.
 */
void flood_fill_compute(MazeMap *m,
                        const uint8_t goal_cells[][2],
                        uint8_t num_goals);

/**
 * Choose the best next direction from cell (x,y) given current heading.
 *
 * Strategy: move toward the neighbor with the lowest flood_value.
 * Tie-breaking: prefer continuing straight (if PREFER_STRAIGHT), then
 * left, right, reverse — minimizes unnecessary turns during exploration.
 *
 * Returns the chosen Direction. If no valid move exists (dead end with all
 * walls), returns current_heading (caller should handle as "stuck").
 */
Direction flood_fill_choose_direction(const MazeMap *m,
                                     uint8_t x, uint8_t y,
                                     Direction current_heading);

#ifdef __cplusplus
}
#endif

#endif /* FLOOD_FILL_H */
