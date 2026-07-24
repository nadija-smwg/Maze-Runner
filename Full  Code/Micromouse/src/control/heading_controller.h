/**
 * @file heading_controller.h
 * @brief PID controller for robot heading (steering).
 *
 * Computes an angular velocity correction to maintain a target heading.
 */

#ifndef HEADING_CONTROLLER_H
#define HEADING_CONTROLLER_H

#include "pid.h"

/**
 * @brief Initialize heading controller.
 */
void heading_controller_init(void);

/**
 * @brief Update the heading controller.
 *
 * @param target_heading_deg Target absolute heading
 * @param current_heading_deg Current measured heading (from fusion)
 * @param dt Time step in seconds
 * @return Angular velocity correction (rad/s or mm/s differential)
 *
 * TODO: Compute PID output based on heading error.
 */
float heading_controller_update(float target_heading_deg,
                                float current_heading_deg,
                                float dt);

/**
 * @brief Reset the heading controller state.
 */
void heading_controller_reset(void);

#endif /* HEADING_CONTROLLER_H */
