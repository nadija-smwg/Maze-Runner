/**
 * @file straight_motion.h
 * @brief Straight-line motion segment management.
 */

#ifndef STRAIGHT_MOTION_H
#define STRAIGHT_MOTION_H

#include <stdbool.h>
#include "motion_profile.h"

/**
 * @brief Start a straight line move.
 *
 * @param distance_mm Distance to move (mm)
 * @param start_speed Initial speed (mm/s)
 * @param end_speed Target exit speed (mm/s)
 * @param max_speed Cruise speed (mm/s)
 */
void straight_motion_start(float distance_mm,
                           float start_speed,
                           float end_speed,
                           float max_speed);

/**
 * @brief Update the straight motion state.
 *
 * @param distance_traveled_mm Current distance measured by odometry
 */
void straight_motion_update(float distance_traveled_mm);

/**
 * @brief Check if the straight motion is complete.
 */
bool straight_motion_is_complete(void);

/**
 * @brief Get the target linear velocity.
 */
float straight_motion_get_target_speed(void);

#endif /* STRAIGHT_MOTION_H */
