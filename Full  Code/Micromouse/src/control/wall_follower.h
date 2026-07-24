/**
 * @file wall_follower.h
 * @brief PID controller for centering the robot using wall sensors.
 */

#ifndef WALL_FOLLOWER_H
#define WALL_FOLLOWER_H

#include "pid.h"

/**
 * @brief Initialize wall follower.
 */
void wall_follower_init(void);

/**
 * @brief Update the wall follower.
 *
 * @param lateral_error_mm Centering error from distance_manager
 * @param dt Time step in seconds
 * @return Angular velocity correction (rad/s or mm/s differential)
 */
float wall_follower_update(float lateral_error_mm, float dt);

/**
 * @brief Enable or disable wall following.
 */
void wall_follower_enable(bool enable);

/**
 * @brief Reset wall follower state.
 */
void wall_follower_reset(void);

#endif /* WALL_FOLLOWER_H */
