/**
 * @file pid.cpp
 * @brief PID controller implementation.
 * @see pid.h
 */

#include "pid.h"
#include "../utils/math_utils.h"

PID::PID(float kp, float ki, float kd, float out_min, float out_max)
    : _kp(kp), _ki(ki), _kd(kd), _out_min(out_min), _out_max(out_max),
      _integral(0.0f), _prev_error(0.0f), _prev_measurement(0.0f) {}

float PID::compute(float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) {
        dt = 0.001f; // Default to 1 ms if dt is invalid to avoid division by zero
    }

    // 1. Proportional error
    float error = setpoint - measurement;
    float p_term = _kp * error;

    // 2. Integral term with anti-windup clamping
    _integral += _ki * error * dt;
    _integral = math_constrain(_integral, _out_min, _out_max);

    // 3. Derivative term on measurement (prevents derivative kick on step setpoint changes)
    float d_term = _kd * (measurement - _prev_measurement) / dt;

    // 4. Calculate total control output
    float output = p_term + _integral - d_term;
    output = math_constrain(output, _out_min, _out_max);

    // 5. Update state variables for next iteration
    _prev_error = error;
    _prev_measurement = measurement;

    return output;
}

void PID::reset() {
    _integral = 0.0f;
    _prev_error = 0.0f;
    _prev_measurement = 0.0f;
}

void PID::set_gains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}
