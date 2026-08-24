/**
 * @file rolling_turn.h
 * @brief Rolling turn execution logic.
 */

#ifndef ROLLING_TURN_H
#define ROLLING_TURN_H

#include <stdbool.h>
#include "motion_profile.h"
#include "../maze/maze.h"

/**
 * @brief Initialize a rolling turn.
 *
 * @param turn Turn type (CMD_SMOOTH_LEFT_90, etc.)
 * @param entry_speed Current forward speed (mm/s)
 */
void rolling_turn_start(CommandType turn, float entry_speed);

/**
 * @brief Update the rolling turn state.
 *
 * @param dt Time step
 */
void rolling_turn_update(float dt);

/**
 * @brief Check if the rolling turn is complete.
 */
bool rolling_turn_is_complete(void);

/**
 * @brief Get the target velocities for the current step.
 *
 * @param out_v Pointer to store target linear velocity
 * @param out_omega Pointer to store target angular velocity
 */
void rolling_turn_get_targets(float *out_v, float *out_omega);

#endif /* ROLLING_TURN_H */
