/**
 * @file speed_controller.h
 * @brief Left and right wheel speed PID controllers.
 *
 * Converts target mm/s speeds for each wheel into PWM values
 * for the motor driver.
 */

#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include "pid.h"

/**
 * @brief Initialize the speed controllers.
 */
void speed_controller_init(void);

/**
 * @brief Update the speed controllers.
 * 
 * @param target_left_speed_mm_s Target left wheel speed
 * @param target_right_speed_mm_s Target right wheel speed
 * @param current_left_speed_mm_s Measured left wheel speed from encoders
 * @param current_right_speed_mm_s Measured right wheel speed from encoders
 * @param dt Time step in seconds
 * 
 * TODO: Run PID for each wheel, add feedforward, and send to motor module.
 */
void speed_controller_update(float target_left_speed_mm_s,
                             float target_right_speed_mm_s,
                             float current_left_speed_mm_s,
                             float current_right_speed_mm_s,
                             float dt);

/**
 * @brief Reset the speed controller PID states.
 */
void speed_controller_reset(void);

#endif /* SPEED_CONTROLLER_H */
