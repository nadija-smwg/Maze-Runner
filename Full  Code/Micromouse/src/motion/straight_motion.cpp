/**
 * @file straight_motion.cpp
 * @brief Straight line motion implementation — trapezoidal velocity profile.
 * @see straight_motion.h
 *
 * Uses the motion_profile API to smoothly ramp from start_speed to
 * max_speed (accel phase), cruise, then ramp down to end_speed (decel phase).
 *
 * This eliminates:
 *   - Wheel slip caused by instant speed step commands
 *   - Overshoot caused by no deceleration before the stop point
 *
 * Completion:
 *   Profile is done when distance_traveled >= total_dist_mm AND
 *   speed has decayed to <= end_speed + 1.0 mm/s margin.
 */

#include "straight_motion.h"
#include "../config/robot_config.h"
#include "motion_profile.h"


static LinearProfile _linear_profile;
static float _current_target_speed = 0.0f;
static bool  _active = false;
static float _last_dist_mm = 0.0f;

void straight_motion_start(float distance_mm, float start_speed,
                           float end_speed, float max_speed) {
  profile_compute_linear(&_linear_profile, distance_mm, max_speed,
                         SEARCH_ACCEL_MM_S2, /* acceleration rate */
                         SEARCH_DECEL_MM_S2, /* deceleration rate */
                         start_speed, end_speed);
  _current_target_speed = start_speed;
  _last_dist_mm = 0.0f;
  _active = true;
}

void straight_motion_update(float distance_traveled_mm) {
  if (!_active)
    return;
  _last_dist_mm = distance_traveled_mm;
  _current_target_speed =
      profile_get_speed(&_linear_profile, distance_traveled_mm);
}

bool straight_motion_is_complete(void) {
  if (!_active)
    return true;
  /* Done when we have reached or exceeded the total profile distance
   * AND speed has wound down to within 1 mm/s of the target end speed. */
  return (_last_dist_mm >= _linear_profile.total_dist_mm) &&
         (_current_target_speed <= _linear_profile.end_speed + 1.0f);
}

void straight_motion_stop(void) {
  _active = false;
  _current_target_speed = 0.0f;
  _last_dist_mm = 0.0f;
}

float straight_motion_get_target_speed(void) { return _current_target_speed; }

float straight_motion_get_total_dist(void) {
  return _linear_profile.total_dist_mm;
}
