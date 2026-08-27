/**
 * @file speed_controller.cpp
 * @brief Speed controller implementation.
 * @see speed_controller.h
 */

#include "speed_controller.h"
#include "../config/robot_config.h"
#include "../hardware/motor.h"
#include "pid.h"

// PID constants for wheel speed (needs tuning)
#define SPEED_KP 0.05f
#define SPEED_KI 0.02f
#define SPEED_KD 0.0f

// Feedforward constant: roughly PWM_MAX / max_speed_mm_s.
// Calibrated value based on test data for ~150-300 mm/s operating range.
#define FEEDFORWARD_KV 3.5f

static PID _left_pid(SPEED_KP, SPEED_KI, SPEED_KD, -PWM_MAX, PWM_MAX);
static PID _right_pid(SPEED_KP, SPEED_KI, SPEED_KD, -PWM_MAX, PWM_MAX);

void speed_controller_init(void) {
  _left_pid.reset();
  _right_pid.reset();
}

void speed_controller_update(float target_left_speed_mm_s,
                             float target_right_speed_mm_s,
                             float current_left_speed_mm_s,
                             float current_right_speed_mm_s, float dt) {

  // 1 & 2. Compute PID outputs
  float left_pid_out =
      _left_pid.compute(target_left_speed_mm_s, current_left_speed_mm_s, dt);
  float right_pid_out =
      _right_pid.compute(target_right_speed_mm_s, current_right_speed_mm_s, dt);

  // 3. Apply feedforward
  float left_ff = target_left_speed_mm_s * FEEDFORWARD_KV;
  float right_ff = target_right_speed_mm_s * FEEDFORWARD_KV;

  // 4. Combine and constrain
  float left_total = left_ff + left_pid_out;
  float right_total = right_ff + right_pid_out;

  // Clamp to PWM range
  if (left_total > PWM_MAX)
    left_total = PWM_MAX;
  if (left_total < -PWM_MAX)
    left_total = -PWM_MAX;
  if (right_total > PWM_MAX)
    right_total = PWM_MAX;
  if (right_total < -PWM_MAX)
    right_total = -PWM_MAX;

  // 5. Convert to PWM and set motor speeds
  motor_set_both((int16_t)left_total, (int16_t)right_total);
}

void speed_controller_reset(void) {
  _left_pid.reset();
  _right_pid.reset();
}

void speed_controller_set_gains(float kp, float ki, float kd) {
  _left_pid.set_gains(kp, ki, kd);
  _right_pid.set_gains(kp, ki, kd);
}
