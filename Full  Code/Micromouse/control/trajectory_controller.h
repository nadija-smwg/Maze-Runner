/**
 * @file trajectory_controller.h
 * @brief Tracks a motion trajectory.
 */

#ifndef TRAJECTORY_CONTROLLER_H
#define TRAJECTORY_CONTROLLER_H

#include <stdbool.h>

/**
 * @brief Initialize trajectory controller.
 */
void trajectory_controller_init(void);

/**
 * @brief Update the trajectory controller.
 *
 * Checks current time/distance against the motion profile
 * and updates target velocity and heading.
 *
 * @param dt Time step
 */
void trajectory_controller_update(float dt);

/**
 * @brief Check if current trajectory is complete.
 */
bool trajectory_is_complete(void);

#endif /* TRAJECTORY_CONTROLLER_H */
