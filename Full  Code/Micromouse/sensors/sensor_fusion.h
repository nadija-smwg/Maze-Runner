/**
 * @file sensor_fusion.h
 * @brief Combines IMU, encoders, and ToF sensors for state estimation.
 *
 * Uses complementary or Kalman filters to provide robust estimates of
 * heading and velocity.
 */

#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <stdint.h>

/**
 * @brief Initialize the sensor fusion filters.
 */
void fusion_init(void);

/**
 * @brief Update the fusion estimate with new sensor data.
 *
 * @param dt Time step in seconds
 *
 * TODO: Read IMU Z-gyro rate and Encoder wheel speeds.
 * TODO: Combine them to estimate current heading and velocity.
 */
void fusion_update(float dt);

/**
 * @brief Get the fused heading estimate.
 *
 * @return Heading angle in degrees
 */
float fusion_get_heading(void);

/**
 * @brief Reset the heading estimate (e.g., at start or when squared up against a wall).
 *
 * @param new_heading Absolute heading to set
 */
void fusion_reset_heading(float new_heading);

/**
 * @brief Get the fused linear velocity estimate.
 *
 * @return Speed in mm/s
 */
float fusion_get_velocity(void);

#endif /* SENSOR_FUSION_H */
