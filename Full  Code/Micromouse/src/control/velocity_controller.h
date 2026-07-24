/**
 * @file velocity_controller.h
 * @brief Velocity setpoint manager.
 *
 * Converts unicycle model velocities (v, ω) into differential wheel speeds.
 */

#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

/**
 * @brief Initialize velocity controller.
 */
void velocity_controller_init(void);

/**
 * @brief Update velocities and send to speed controller.
 *
 * @param linear_velocity_mm_s Target forward velocity
 * @param angular_velocity_rad_s Target angular velocity
 */
void velocity_controller_update(float linear_velocity_mm_s,
                                float angular_velocity_rad_s);

#endif /* VELOCITY_CONTROLLER_H */
