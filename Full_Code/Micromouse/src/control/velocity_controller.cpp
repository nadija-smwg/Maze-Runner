/**
 * @file velocity_controller.cpp
 * @brief Velocity controller implementation.
 * @see velocity_controller.h
 */

#include "velocity_controller.h"
#include "speed_controller.h"
#include "../config/robot_config.h"
#include "../hardware/encoder.h"
#include "../utils/filters.h"

static LowPassFilter left_speed_filter(0.05f);
static LowPassFilter right_speed_filter(0.05f);

void velocity_controller_init(void) {
    speed_controller_init();
    encoder_get_delta(ENCODER_LEFT);
    encoder_get_delta(ENCODER_RIGHT);
    left_speed_filter.reset(0.0f);
    right_speed_filter.reset(0.0f);
}

void velocity_controller_update(float linear_velocity_mm_s,
                                float angular_velocity_rad_s) {
    // 1. Convert (v, ω) to (v_left, v_right):
    float w_term = angular_velocity_rad_s * (WHEEL_BASE_MM / 2.0f);
    
    float target_left_speed = linear_velocity_mm_s - w_term;
    float target_right_speed = linear_velocity_mm_s + w_term;
    
    // 2. Measure current wheel speeds (called at 1kHz, so dt=1ms)
    // Counts per 1ms * 1000 = Counts per sec. We filter this because at 1ms, counts are tiny integers!
    float raw_left = encoder_counts_to_speed(encoder_get_delta(ENCODER_LEFT) * 1000.0f);
    float raw_right = encoder_counts_to_speed(encoder_get_delta(ENCODER_RIGHT) * 1000.0f);
    
    float current_left_speed = left_speed_filter.update(raw_left);
    float current_right_speed = right_speed_filter.update(raw_right);
    
    // 3. Pass to speed controller
    speed_controller_update(target_left_speed, target_right_speed, 
                            current_left_speed, current_right_speed, 
                            CONTROL_LOOP_DT_S);
}
