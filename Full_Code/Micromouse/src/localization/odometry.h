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
 */
void odometry_update(void);

/**
 * @brief Get the current odometry pose.
 */
Pose odometry_get_pose(void);

/**
 * @brief Force the current pose to a known value.
 */
void odometry_set_pose(Pose new_pose);

#endif /* ODOMETRY_H */
