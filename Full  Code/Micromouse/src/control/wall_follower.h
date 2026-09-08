/**
 * @file wall_follower.h
 * @brief PD controller for centering the robot using side ToF sensors.
 *
 * Step 5.5 — Wall Following Controller.
 *
 * The wall follower computes an angular velocity correction (rad/s)
 * from the lateral centering error reported by distance_manager.
 * This correction is added to the heading correction already provided
 * by the gyro, and both are passed together to velocity_controller_update().
 *
 * Gains are defined in robot_config.h:
 *   KP_WALL, KD_WALL, MAX_WALL_CORRECTION_RAD_S
 */

#ifndef WALL_FOLLOWER_H
#define WALL_FOLLOWER_H

#include "pid.h"
#include <stdbool.h>

/**
 * @brief Initialize wall follower and reset PID state.
 */
void wall_follower_init(void);

/**
 * @brief Update the wall follower — compute angular correction.
 *
 * @param lateral_error_mm Centering error from distance_get_centering_error()
 *                         Positive = robot shifted LEFT, must steer RIGHT.
 * @param dt               Time step in seconds (same as CONTROL_LOOP_DT_S)
 * @return Angular velocity correction (rad/s) to add to heading correction.
 *         Returns 0 if wall follower is disabled.
 */
float wall_follower_update(float lateral_error_mm, float dt);

/**
 * @brief Enable or disable wall following.
 *
 * Resets PID state when disabled to prevent windup carry-over.
 *
 * @param enable true to start correcting, false to stop.
 */
void wall_follower_enable(bool enable);

/**
 * @brief Reset PID integrator/derivative state.
 */
void wall_follower_reset(void);

/**
 * @brief Query whether wall following is currently active.
 */
bool wall_follower_is_enabled(void);

/**
 * @brief Dynamically set the PD gains for live tuning.
 *
 * @param kp Proportional gain (rad/s per mm error)
 * @param kd Derivative gain
 */
void wall_follower_set_gains(float kp, float kd);

#endif /* WALL_FOLLOWER_H */
