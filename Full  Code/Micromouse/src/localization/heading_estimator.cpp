/**
 * @file heading_estimator.cpp
 * @brief Heading estimator implementation.
 * @see heading_estimator.h
 */

#include "heading_estimator.h"

static float _fused_heading_rad = 0.0f;

void heading_estimator_init(void) {
    _fused_heading_rad = 0.0f;
}

void heading_estimator_update(float gyro_z_dps, float encoder_heading_rad, float dt) {
    // Gyro: fast response, drifts over time
    // Note: If gyro reads NEGATIVE when spinning clockwise, we might need a minus sign here
    float gyro_dtheta = (gyro_z_dps * 0.017453f) * dt;

    // Complementary filter:
    // 98% gyro  → fast, immune to wheel slip
    // 2%  encoder → long-term, drift-free
    _fused_heading_rad = 0.98f * (_fused_heading_rad + gyro_dtheta)
                       + 0.02f * encoder_heading_rad;

    // Normalize to [-π, π]
    while (_fused_heading_rad >  3.14159f) _fused_heading_rad -= 6.28318f;
    while (_fused_heading_rad < -3.14159f) _fused_heading_rad += 6.28318f;
}

float heading_estimator_get(void) {
    return _fused_heading_rad;
}
