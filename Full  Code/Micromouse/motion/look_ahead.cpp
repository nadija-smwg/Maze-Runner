/**
 * @file look_ahead.cpp
 * @brief Look-ahead planner implementation.
 * @see look_ahead.h
 */

#include "look_ahead.h"
#include "motion_profile.h"

float look_ahead_compute_exit_speed(const MotionCommand* commands,
                                    int num_commands,
                                    float current_cruise_speed) {
    /**
     * TODO:
     * 1. Look at commands[1].
     * 2. If it's a smooth turn, exit speed = turn speed limit.
     * 3. If it's a stop or in-place turn, exit speed = 0.
     * 4. If it's another straight, look further ahead.
     */
    return 0.0f;
}
