/**
 * @file heading_estimator.cpp
 * @brief Heading estimator implementation.
 * @see heading_estimator.h
 */

#include "heading_estimator.h"
#include "../config/robot_config.h"
#include <math.h>

static float _fused_heading_rad = 0.0f;

/* Gyro noise floor (°/s): tiny residual after stopping that would integrate
 * into false heading drift. Readings below this threshold are clamped to 0. */
#define GYRO_DEADBAND_DPS 1.2f

void heading_estimator_init(void) {
    _fused_heading_rad = 0.0f;
}

void heading_estimator_update(float gyro_z_dps, float encoder_heading_rad, float dt) {
    // Deadband: suppress tiny residual noise after turn stops or when idle.
    // This stops heading from drifting when the robot is sitting still.
    if (fabsf(gyro_z_dps) < GYRO_DEADBAND_DPS) {
        gyro_z_dps = 0.0f;
    } else {
        // Apply empirical calibration scale factor for clone MPU6050s
        if (gyro_z_dps > 0.0f) {
            gyro_z_dps *= GYRO_MULTIPLIER_LEFT;   // CCW Rotation
        } else {
            gyro_z_dps *= GYRO_MULTIPLIER_RIGHT;  // CW Rotation
        }
    }

    // Gyro: fast response, drifts over time
    float gyro_dtheta = (gyro_z_dps * 0.017453f) * dt;

    // Complementary filter at 1kHz (dt=0.001s):
    // 0.999 gyro / 0.001 encoder -> Time constant tau = 1.0 seconds
    // This allows the gyro to fully dominate during fast 90-degree turns (which take ~0.5s),
    // preventing wheel-slip (skid-steer) from distorting the turn angle.
    _fused_heading_rad = 0.999f * (_fused_heading_rad + gyro_dtheta)
                       + 0.001f * encoder_heading_rad;

    // Normalize to [-π, π]
    while (_fused_heading_rad >  3.14159f) _fused_heading_rad -= 6.28318f;
    while (_fused_heading_rad < -3.14159f) _fused_heading_rad += 6.28318f;
}

float heading_estimator_get(void) {
    return _fused_heading_rad;
}
