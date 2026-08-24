/**
 * @file heading_estimator.cpp
 * @brief Heading estimator implementation.
 * @see heading_estimator.h
 */

#include "heading_estimator.h"
#include "odometry.h"
#include "pose.h"
#include <math.h>

static float _fused_heading_deg = 0.0f;

void heading_estimator_init(void) {
    _fused_heading_deg = 0.0f;
}

void heading_estimator_update(float gyro_z_dps, float encoder_dtheta_rad, float dt) {
    // 1. Convert Gyro rate to angle delta
    float gyro_dtheta_deg = gyro_z_dps * dt;

    // 2. Deadband: If encoders say we aren't moving, and gyro rate is tiny, ignore gyro noise!
    // Approximating encoder speed: if delta theta is very small.
    // We should also check linear speed, but dtheta is a good proxy for rotation.
    if (fabs(encoder_dtheta_rad) < 0.001f && fabs(gyro_z_dps) < 1.0f) {
        gyro_dtheta_deg = 0.0f; 
    }

    // Get absolute encoder heading in degrees
    float encoder_heading_deg = odometry_get_pose().theta_rad * (180.0f / 3.14159265f);

    // 3. Complementary Filter
    // Temporarily set alpha to 1.0f (100% Gyro, 0% Encoders) for Phase 5 Testing.
    // This allows us to trust the pure gyro angle so we can calibrate the wheels against it!
    float alpha = 1.0f; // tau / (tau + dt);

    _fused_heading_deg = alpha * (_fused_heading_deg + gyro_dtheta_deg) + (1.0f - alpha) * encoder_heading_deg;
    
    // Normalize to [-180, 180]
    _fused_heading_deg = pose_normalize_angle_deg(_fused_heading_deg);
}

float heading_estimator_get(void) {
    return _fused_heading_deg;
}
