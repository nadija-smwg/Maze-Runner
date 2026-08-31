/**
 * @file velocity_controller.cpp
 * @brief Velocity controller — converts unicycle (v, ω) to differential wheel speeds.
 * @see velocity_controller.h
 *
 * Architecture:
 *
 *   velocity_controller_update(v, ω)
 *           │
 *   Unicycle → differential:
 *     v_left  = v - ω * WHEEL_BASE_MM / 2
 *     v_right = v + ω * WHEEL_BASE_MM / 2
 *           │
 *   speed_controller_update(v_left, v_right, measured_left, measured_right, dt)
 *           │
 *   PI + feed-forward → motor_set_speed_compensated()
 */

#include "velocity_controller.h"
#include "speed_controller.h"
#include "../hardware/encoder.h"
#include "../config/robot_config.h"

void velocity_controller_init(void) {
    speed_controller_init();
}

void velocity_controller_update(float linear_velocity_mm_s,
                                float angular_velocity_rad_s) {
    /*
     * Unicycle model → differential wheel speeds:
     *
     *   v_left  = v - (ω × L / 2)
     *   v_right = v + (ω × L / 2)
     *
     * where L = WHEEL_BASE_MM (track width in mm).
     *
     * Positive ω → turning right (right wheel slower).
     */
    float half_base = WHEEL_BASE_MM * 0.5f;

    float target_left_mm_s  = linear_velocity_mm_s
                              - angular_velocity_rad_s * half_base;

    float target_right_mm_s = linear_velocity_mm_s
                              + angular_velocity_rad_s * half_base;

    /* Read filtered wheel speeds from encoder velocity filter */
    float actual_left_mm_s  = encoder_get_speed_mms(ENCODER_LEFT);
    float actual_right_mm_s = encoder_get_speed_mms(ENCODER_RIGHT);

    /* Delegate to speed controller (PI + feed-forward + dead-zone) */
    speed_controller_update(target_left_mm_s,
                            target_right_mm_s,
                            actual_left_mm_s,
                            actual_right_mm_s,
                            CONTROL_LOOP_DT_S);
}
