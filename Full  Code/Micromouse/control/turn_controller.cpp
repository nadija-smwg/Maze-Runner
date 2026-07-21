/**
 * @file turn_controller.cpp
 * @brief Turn controller implementation.
 * @see turn_controller.h
 */

#include "turn_controller.h"

void turn_start_inplace(TurnType turn) {
    /**
     * TODO:
     * 1. Calculate target heading change (+90, -90, +180).
     * 2. Set heading_controller target.
     * 3. Enable in-place rotation mode in motion controller.
     */
}

void turn_start_rolling(TurnType turn, float entry_speed) {
    /**
     * TODO:
     * 1. Configure arc_motion module with radius, angle, and entry speed.
     * 2. Start arc_motion.
     */
}

bool turn_is_complete(void) {
    /**
     * TODO: Check if heading error is within TURN_TOLERANCE_DEG
     * and angular velocity is near zero (for in-place) or arc_motion is done (for rolling).
     */
    return true;
}

void turn_stop(void) {
    /** TODO: Abort turn and hold current heading. */
}
