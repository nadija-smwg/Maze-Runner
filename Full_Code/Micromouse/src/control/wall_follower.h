/**
 * @file wall_follower.h
 * @brief Simple PD controller for wall following using ToF sensors.
 *
 * Abandons velocity PIDs in favor of direct PWM manipulation based on
 * lateral error (distance from center).
 */

#ifndef WALL_FOLLOWER_H
#define WALL_FOLLOWER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Base PWM speeds for open-loop forward movement.
 * If one motor is weaker, increase its base PWM here so the robot
 * drives mostly straight even before the PD controller kicks in.
 */
#define WALL_FOLLOW_BASE_PWM_LEFT  1500
#define WALL_FOLLOW_BASE_PWM_RIGHT 1500

/**
 * @brief Initialize the wall follower PID.
 */
void wall_follower_init(void);

/**
 * @brief Update the wall follower and apply motor speeds.
 *
 * Takes the lateral error from the ToF sensors, calculates a PD correction,
 * applies it to the BASE_PWM, and directly sets the motor speeds.
 *
 * @param lateral_error_mm The centering error (positive = too far right)
 * @param dt Time delta in seconds
 */
void wall_follower_update(float lateral_error_mm, float dt);

/**
 * @brief Get the last calculated correction value for debugging (OLED).
 */
int16_t wall_follower_get_last_correction(void);

/**
 * @brief Get current active KP for live tuning.
 */
float wall_follower_get_kp(void);

/**
 * @brief Live-tune the KP value (e.g. from a button press).
 */
void wall_follower_set_kp(float new_kp);

#endif /* WALL_FOLLOWER_H */
