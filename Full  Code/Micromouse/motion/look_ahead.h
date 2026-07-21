/**
 * @file look_ahead.h
 * @brief Look-ahead speed planner for upcoming turns.
 */

#ifndef LOOK_AHEAD_H
#define LOOK_AHEAD_H

#include "../maze/maze.h"

/**
 * @brief Compute safe exit speed for current straight segment.
 *
 * Analyzes upcoming commands to ensure there is enough distance
 * to decelerate for a turn or stop.
 *
 * @param commands Array of remaining motion commands
 * @param num_commands Number of commands remaining
 * @param current_cruise_speed Current target cruise speed
 * @return Safe exit speed (mm/s)
 */
float look_ahead_compute_exit_speed(const MotionCommand* commands,
                                    int num_commands,
                                    float current_cruise_speed);

#endif /* LOOK_AHEAD_H */
