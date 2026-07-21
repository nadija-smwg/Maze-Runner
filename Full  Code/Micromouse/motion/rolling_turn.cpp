/**
 * @file rolling_turn.cpp
 * @brief Rolling turn implementation.
 * @see rolling_turn.h
 */

#include "rolling_turn.h"

static ArcTurnProfile _arc_profile;

void rolling_turn_start(CommandType turn, float entry_speed) {
    /**
     * TODO:
     * 1. Determine radius, angle, and direction based on 'turn'.
     * 2. Call profile_compute_arc(&_arc_profile, ...).
     */
}

void rolling_turn_update(float dt) {
    /** TODO: Integrate dt to track position along arc */
}

bool rolling_turn_is_complete(void) {
    /** TODO: Check if current distance >= _arc_profile.arc_length_mm */
    return true;
}

void rolling_turn_get_targets(float *out_v, float *out_omega) {
    /**
     * TODO:
     * Call profile_get_arc_state(&_arc_profile, current_dist, out_v, out_omega).
     */
}
