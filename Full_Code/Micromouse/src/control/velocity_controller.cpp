/**
 * @file velocity_controller.cpp
 * @brief Velocity controller implementation.
 * @see velocity_controller.h
 *
 * Converts (linear_velocity, angular_velocity) into per-wheel speed targets,
 * reads encoder feedback, applies low-pass filtering, and feeds into the
 * speed controller PID.
 *
 * BUGFIX: Startup speed spike — the first call after init now resets timing
 * to prevent a huge dt from causing PID integral windup.
 */

#include "velocity_controller.h"
#include "speed_controller.h"
#include "../config/robot_config.h"
#include "../hardware/encoder.h"
#include "../utils/filters.h"
#include <Arduino.h>

// Low-pass filter alpha = 0.05 (heavy smoothing, mandatory for 1kHz with low-res encoders)
// At 1kHz with ~0.58 counts/ms, raw speed bounces between 0 and 256 mm/s.
// alpha=0.05 means each new reading contributes only 5%, giving a smooth output.
static LowPassFilter left_speed_filter(0.05f);
static LowPassFilter right_speed_filter(0.05f);
static float _current_avg_speed = 0.0f;
static float _current_left_speed = 0.0f;
static float _current_right_speed = 0.0f;
static bool _first_call = true;
static uint32_t _last_vel_tick = 0;

void velocity_controller_init(void) {
    speed_controller_init();
    // Flush stale encoder deltas so first reading starts clean
    encoder_get_delta(ENCODER_LEFT);
    encoder_get_delta(ENCODER_RIGHT);
    left_speed_filter.reset(0.0f);
    right_speed_filter.reset(0.0f);
    _current_avg_speed = 0.0f;
    _current_left_speed = 0.0f;
    _current_right_speed = 0.0f;
    _first_call = true;
}

void velocity_controller_update(float linear_velocity_mm_s,
                                float angular_velocity_rad_s) {
    // 1. Convert (v, ω) to (v_left, v_right) — differential drive equations:
    float w_term = angular_velocity_rad_s * (WHEEL_BASE_MM / 2.0f);
    
    float target_left_speed = linear_velocity_mm_s - w_term;
    float target_right_speed = linear_velocity_mm_s + w_term;
    
    // 2. Measure current wheel speeds with proper timing
    uint32_t now = micros();
    
    // BUGFIX: On the very first call after init, reset timing and flush encoders.
    // Without this, dt = (now - setup_time) = several seconds, causing:
    //   - PID integral windup: Ki * error * dt = 0.02 * 300 * 5.0 = 30 (massive!)
    //   - Initial PWM spike that jerks the robot violently
    if (_first_call) {
        _last_vel_tick = now;
        _first_call = false;
        // Flush any accumulated encoder counts from idle period
        encoder_get_delta(ENCODER_LEFT);
        encoder_get_delta(ENCODER_RIGHT);
        // Reset PID state to prevent any residual integral
        speed_controller_reset();
        // Don't drive motors on first tick — just initialize timing
        return;
    }

    float actual_dt = (now - _last_vel_tick) / 1000000.0f;
    _last_vel_tick = now;

    // Clamp dt to prevent spikes (min 0.5ms, max 10ms)
    // If OLED or I2C blocked the loop for >10ms, pretend it was only 10ms
    // to prevent the PID from seeing a huge dt and over-accumulating integral.
    if (actual_dt <= 0.0f) actual_dt = 0.001f;
    if (actual_dt > 0.01f) actual_dt = 0.01f;

    float raw_left = encoder_counts_to_speed(encoder_get_delta(ENCODER_LEFT) / actual_dt);
    float raw_right = encoder_counts_to_speed(encoder_get_delta(ENCODER_RIGHT) / actual_dt);
    
    _current_left_speed = left_speed_filter.update(raw_left);
    _current_right_speed = right_speed_filter.update(raw_right);
    
    _current_avg_speed = (_current_left_speed + _current_right_speed) / 2.0f;
    
    // 3. Pass to speed controller
    speed_controller_update(target_left_speed, target_right_speed, 
                            _current_left_speed, _current_right_speed, 
                            actual_dt);
}

float velocity_controller_get_speed(void) {
    return _current_avg_speed;
}

float velocity_controller_get_left_speed(void) {
    return _current_left_speed;
}

float velocity_controller_get_right_speed(void) {
    return _current_right_speed;
}
