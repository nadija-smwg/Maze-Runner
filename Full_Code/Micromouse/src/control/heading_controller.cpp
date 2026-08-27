/**
 * @file heading_controller.cpp
 * @brief Heading controller implementation.
 * @see heading_controller.h
 */

#include "heading_controller.h"
#include "../utils/math_utils.h"
#include "pid.h"

// PID constants for heading control (needs tuning)
#define HEADING_KP 0.8f
#define HEADING_KI 0.0f
#define HEADING_KD 0.05f
#define MAX_OMEGA_RAD_S 5.0f // Max angular velocity correction (rad/s)

static PID _heading_pid(HEADING_KP, HEADING_KI, HEADING_KD, -MAX_OMEGA_RAD_S,
                        MAX_OMEGA_RAD_S);

void heading_controller_init(void) { _heading_pid.reset(); }

float heading_controller_update(float target_heading_deg,
                                float current_heading_deg, float dt) {
  // 1. Calculate error = target - current
  float error = target_heading_deg - current_heading_deg;

  // 2. Wrap error to [-180, 180]
  while (error > 180.0f)
    error -= 360.0f;
  while (error < -180.0f)
    error += 360.0f;

  // To allow the PID class to correctly calculate derivative on measurement,
  // we pass equivalent non-wrapped values.
  float continuous_setpoint = target_heading_deg;
  float continuous_measurement = target_heading_deg - error;

  // 3. Compute PID output
  float omega =
      _heading_pid.compute(continuous_setpoint, continuous_measurement, dt);

  // 4. Return angular velocity correction
  return omega;
}

void heading_controller_reset(void) { _heading_pid.reset(); }

void heading_controller_set_gains(float kp, float ki, float kd) {
    _heading_pid.set_gains(kp, ki, kd);
}
