/**
 * @file odometry.h
 * @brief Encoder-based dead reckoning.
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "pose.h"

/**
 * @brief Initialize odometry tracking.
 */
void odometry_init(void);

/**
 * @brief Update the odometry pose based on encoder deltas.
 *
 * @param left_delta_mm Left wheel travel distance
 * @param right_delta_mm Right wheel travel distance
 */
void odometry_update(float left_delta_mm, float right_delta_mm);

/**
 * @brief Get the current odometry pose.
 */
Pose odometry_get_pose(void);

/**
 * @brief Force the current pose to a known value.
 */
void odometry_set_pose(Pose new_pose);

#endif /* ODOMETRY_H */
