/**
 * @file speed_controller.cpp
 * @brief Speed controller implementation with per-motor feedforward.
 * @see speed_controller.h
 *
 * Each wheel has its own PID + feedforward. The feedforward constants
 * (KV_LEFT, KV_RIGHT) can differ to compensate for motor asymmetry
 * (one motor being physically weaker than the other).
 *
 * Feedforward gives ~90% of the required PWM instantly.
 * PID corrects the remaining ~10% for battery voltage changes, friction, etc.
 */

#include "speed_controller.h"
#include "../config/robot_config.h"
#include "../hardware/motor.h"
#include "pid.h"

// ═══════════════════════════════════════════════════════════════════════════
//  PID Gains (conservative — let feedforward do most of the work)
// ═══════════════════════════════════════════════════════════════════════════
// With 1kHz control loop and ~0.58 encoder counts/ms, high PID gains
// amplify quantization noise. Keep KP very low and KD at zero.
#define SPEED_KP 0.05f
#define SPEED_KI 0.02f
#define SPEED_KD 0.0f

// ═══════════════════════════════════════════════════════════════════════════
//  Per-Motor Feedforward Constants
// ═══════════════════════════════════════════════════════════════════════════
// KV = PWM / speed_mm_s. Calibrate separately for each motor.
// If one motor is weaker, it needs a higher KV to produce the same speed.
// To calibrate: run motors at known PWM values (500, 1000, 1500, 2000),
// measure actual speed via Serial, compute KV = PWM / speed, average.
#define FEEDFORWARD_KV_LEFT  3.5f
#define FEEDFORWARD_KV_RIGHT 3.5f

static PID _left_pid(SPEED_KP, SPEED_KI, SPEED_KD, -PWM_MAX, PWM_MAX);
static PID _right_pid(SPEED_KP, SPEED_KI, SPEED_KD, -PWM_MAX, PWM_MAX);

// Track last PWM outputs for debugging (displayed on OLED and Serial)
static int16_t _last_left_pwm = 0;
static int16_t _last_right_pwm = 0;

void speed_controller_init(void) {
  _left_pid.reset();
  _right_pid.reset();
  _last_left_pwm = 0;
  _last_right_pwm = 0;
}

void speed_controller_update(float target_left_speed_mm_s,
                             float target_right_speed_mm_s,
                             float current_left_speed_mm_s,
                             float current_right_speed_mm_s, float dt) {

  // 1. Compute PID corrections
  float left_pid_out =
      _left_pid.compute(target_left_speed_mm_s, current_left_speed_mm_s, dt);
  float right_pid_out =
      _right_pid.compute(target_right_speed_mm_s, current_right_speed_mm_s, dt);

  // 2. Apply per-motor feedforward
  // Feedforward gives the motor approximately the right PWM without waiting
  // for PID to ramp up. Like setting the gas pedal to roughly the right position.
  float left_ff = target_left_speed_mm_s * FEEDFORWARD_KV_LEFT;
  float right_ff = target_right_speed_mm_s * FEEDFORWARD_KV_RIGHT;

  // 3. Combine feedforward + PID and clamp
  float left_total = left_ff + left_pid_out;
  float right_total = right_ff + right_pid_out;

  if (left_total > PWM_MAX)
    left_total = PWM_MAX;
  if (left_total < -PWM_MAX)
    left_total = -PWM_MAX;
  if (right_total > PWM_MAX)
    right_total = PWM_MAX;
  if (right_total < -PWM_MAX)
    right_total = -PWM_MAX;

  // 4. Store for debugging and set motors
  _last_left_pwm = (int16_t)left_total;
  _last_right_pwm = (int16_t)right_total;
  motor_set_both(_last_left_pwm, _last_right_pwm);
}

void speed_controller_reset(void) {
  _left_pid.reset();
  _right_pid.reset();
  _last_left_pwm = 0;
  _last_right_pwm = 0;
}

void speed_controller_set_gains(float kp, float ki, float kd) {
  _left_pid.set_gains(kp, ki, kd);
  _right_pid.set_gains(kp, ki, kd);
}

int16_t speed_controller_get_left_pwm(void) {
  return _last_left_pwm;
}

int16_t speed_controller_get_right_pwm(void) {
  return _last_right_pwm;
}

