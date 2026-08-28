/**
 * @file wall_follower.cpp
 * @brief Simple PD controller for wall following using ToF sensors.
 * @see wall_follower.h
 */

#include "wall_follower.h"
#include "../hardware/motor.h"
#include "../config/robot_config.h"

// PD Tuning values. 
// KP = How hard to steer back to center.
// KD = How much to resist fast steering (dampening).
static float _kp = 5.0f;  // Default: start small (e.g. 5 PWM units per mm of error)
static float _kd = 1.0f;  // Default dampening

static float _last_error = 0.0f;
static int16_t _last_correction = 0;

void wall_follower_init(void) {
    _last_error = 0.0f;
    _last_correction = 0;
}

void wall_follower_update(float lateral_error_mm, float dt) {
    if (dt <= 0.0f) dt = 0.01f; // Prevent divide by zero

    // 1. Proportional term
    float P = _kp * lateral_error_mm;
    
    // 2. Derivative term (rate of change of error)
    float error_deriv = (lateral_error_mm - _last_error) / dt;
    float D = _kd * error_deriv;
    
    _last_error = lateral_error_mm;
    
    // 3. Calculate total PWM correction
    // If error is positive, we are shifted LEFT. 
    // Need to steer right -> slow down right motor, speed up left motor.
    int16_t correction = (int16_t)(P + D);
    _last_correction = correction;
    
    // 4. Apply to base PWM (inverted polarity for this specific hardware)
    int16_t left_pwm  = WALL_FOLLOW_BASE_PWM_LEFT - correction;
    int16_t right_pwm = WALL_FOLLOW_BASE_PWM_RIGHT + correction;
    
    // 5. Clamp to absolute limits
    if (left_pwm > PWM_MAX) left_pwm = PWM_MAX;
    if (left_pwm < -PWM_MAX) left_pwm = -PWM_MAX;
    if (right_pwm > PWM_MAX) right_pwm = PWM_MAX;
    if (right_pwm < -PWM_MAX) right_pwm = -PWM_MAX;
    
    // 6. Drive motors!
    motor_set_both(left_pwm, right_pwm);
}

int16_t wall_follower_get_last_correction(void) {
    return _last_correction;
}

float wall_follower_get_kp(void) {
    return _kp;
}

void wall_follower_set_kp(float new_kp) {
    _kp = new_kp;
}

