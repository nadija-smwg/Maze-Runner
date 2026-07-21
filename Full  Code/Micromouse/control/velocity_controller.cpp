/**
 * @file velocity_controller.cpp
 * @brief Velocity controller implementation.
 * @see velocity_controller.h
 */

#include "velocity_controller.h"
#include "speed_controller.h"
#include "../config/robot_config.h"

void velocity_controller_init(void) {
    /** TODO: Any necessary initialization */
}

void velocity_controller_update(float linear_velocity_mm_s,
                                float angular_velocity_rad_s) {
    /**
     * TODO:
     * 1. Convert (v, ω) to (v_left, v_right):
     *    v_left  = v - (ω * WHEEL_BASE_MM / 2)
     *    v_right = v + (ω * WHEEL_BASE_MM / 2)
     * 2. Pass to speed_controller_update().
     */
}
