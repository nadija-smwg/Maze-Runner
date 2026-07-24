/**
 * @file speed_controller.cpp
 * @brief Speed controller implementation.
 * @see speed_controller.h
 */

#include "speed_controller.h"
#include "../hardware/motor.h"

/* TODO: Define left and right PID objects */

void speed_controller_init(void) {
    /** TODO: Initialize PIDs with tuned values */
}

void speed_controller_update(float target_left_speed_mm_s,
                             float target_right_speed_mm_s,
                             float current_left_speed_mm_s,
                             float current_right_speed_mm_s,
                             float dt) {
    /**
     * TODO:
     * 1. Compute left PID output.
     * 2. Compute right PID output.
     * 3. Apply feedforward (e.g., target_speed * kv).
     * 4. Convert to PWM.
     * 5. Call motor_set_both(left_pwm, right_pwm).
     */
}

void speed_controller_reset(void) {
    /** TODO: Reset both PIDs */
}
