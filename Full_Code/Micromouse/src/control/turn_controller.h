/**
 * @file turn_controller.h
 * @brief Executes in-place and rolling turns.
 */

#ifndef TURN_CONTROLLER_H
#define TURN_CONTROLLER_H

#include <stdbool.h>
#include "../maze/maze.h"

/**
 * @brief Start an in-place turn.
 *
 * @param turn Desired turn (TURN_RIGHT_90, TURN_LEFT_90, TURN_180)
 */
void turn_start_inplace(TurnType turn);

/**
 * @brief Start a rolling (smooth) turn.
 *
 * @param turn Desired turn (TURN_RIGHT_90, TURN_LEFT_90, TURN_180)
 * @param entry_speed Linear speed at turn entry (mm/s)
 */
void turn_start_rolling(TurnType turn, float entry_speed);

/**
 * @brief Check if the current turn is complete.
 *
 * @return true if complete
 */
bool turn_is_complete(void);

/**
 * @brief Force stop the current turn.
 */
void turn_stop(void);

#endif /* TURN_CONTROLLER_H */
