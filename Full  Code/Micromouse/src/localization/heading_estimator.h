/**
 * @file heading_estimator.h
 * @brief Heading estimator fusing IMU and encoder data.
 */

#ifndef HEADING_ESTIMATOR_H
#define HEADING_ESTIMATOR_H

/**
 * @brief Initialize heading estimator.
 */
void heading_estimator_init(void);

/**
 * @brief Update heading estimate.
 *
 * @param gyro_z_dps Z-axis gyro reading
 * @param encoder_dtheta_rad Change in heading from encoders
 * @param dt Time step
 */
void heading_estimator_update(float gyro_z_dps, float encoder_dtheta_rad, float dt);

/**
 * @brief Get the fused heading.
 */
float heading_estimator_get(void);

#endif /* HEADING_ESTIMATOR_H */
