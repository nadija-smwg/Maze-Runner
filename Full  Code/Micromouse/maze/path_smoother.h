/**
 * @file path_smoother.h
 * @brief Path post-processor: converts raw waypoints into optimized
 *        motion commands (straight-run merging, turn classification,
 *        optional diagonal detection).
 *
 * Tier 3: takes the Dijkstra path output and produces a compact command
 * list that the locomotion layer can execute directly.
 */

#ifndef PATH_SMOOTHER_H
#define PATH_SMOOTHER_H

#include "maze.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Result of path smoothing. */
typedef struct {
    MotionCommand commands[MAX_COMMANDS];
    uint16_t      num_commands;
} SmoothedPath;

/**
 * Convert a raw waypoint path into optimized motion commands.
 *
 * Operations:
 *   1. Straight-run merging: consecutive same-direction waypoints →
 *      single CMD_STRAIGHT(n_cells).
 *   2. Turn classification: heading changes → CMD_TURN_LEFT_90,
 *      CMD_TURN_RIGHT_90, or CMD_TURN_180.
 *   3. Diagonal detection (if ENABLE_DIAGONALS): patterns like
 *      Forward→Right→Forward are replaced by CMD_DIAGONAL.
 *
 * @param waypoints     Input array of ordered waypoints (start→goal)
 * @param num_waypoints Number of waypoints
 * @param result        Output: the smoothed command list
 */
void path_smooth(const Waypoint *waypoints,
                 uint16_t num_waypoints,
                 SmoothedPath *result);

#ifdef __cplusplus
}
#endif

#endif /* PATH_SMOOTHER_H */
