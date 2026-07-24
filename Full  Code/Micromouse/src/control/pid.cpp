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
    /**
     * TODO: Implement PID math.
     * 1. Calculate error = setpoint - measurement.
     * 2. Proportional term: P = kp * error.
     * 3. Integral term: I = I + ki * error * dt. (Include anti-windup clamping).
     * 4. Derivative term: D = kd * (measurement - prev_measurement) / dt. (Derivative on measurement to avoid kick).
     * 5. Output = clamp(P + I - D, out_min, out_max).
     * 6. Save state.
     * 7. Return output.
     */
    return 0.0f;
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
