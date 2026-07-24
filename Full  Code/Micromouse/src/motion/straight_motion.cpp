/**
 * @file straight_motion.cpp
 * @brief Straight line motion implementation.
 * @see straight_motion.h
 */

#include "straight_motion.h"
#include "../config/config.h"

static LinearProfile _linear_profile;
static float _current_target_speed = 0.0f;

void straight_motion_start(float distance_mm,
                           float start_speed,
                           float end_speed,
                           float max_speed) {
    /**
     * TODO:
     * profile_compute_linear(&_linear_profile, distance_mm, max_speed, ...);
     */
}

void straight_motion_update(float distance_traveled_mm) {
    /**
     * TODO:
     * _current_target_speed = profile_get_speed(&_linear_profile, distance_traveled_mm);
     */
}

bool straight_motion_is_complete(void) {
    /** TODO: Return true if distance_traveled >= total_distance */
    return true;
}

float straight_motion_get_target_speed(void) {
    return _current_target_speed;
}
