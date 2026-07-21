/**
 * @file position_estimator.h
 * @brief Full position estimation combining odometry and sensor corrections.
 */

#ifndef POSITION_ESTIMATOR_H
#define POSITION_ESTIMATOR_H

#include "pose.h"

/**
 * @brief Initialize position estimator.
 */
void position_estimator_init(void);

/**
 * @brief Update position estimate.
 *
 * Fuses encoder odometry with ToF sensor wall detections to correct drift.
 *
 * @param dt Time step
 */
void position_estimator_update(float dt);

/**
 * @brief Get the current best estimate of the robot's pose.
 */
Pose position_estimator_get_pose(void);

#endif /* POSITION_ESTIMATOR_H */
