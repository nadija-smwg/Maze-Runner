/**
 * @file speed_controller.cpp
 * @brief Per-wheel PI velocity controller with feed-forward and anti-windup.
 * @see speed_controller.h
 *
 * Control law per wheel:
 *
 *   PWM = Kff × target_mm_s         (feed-forward)
 *       + Kp  × error               (proportional)
 *       + Ki  × ∫error dt           (integral with anti-windup)
 *
 * Then dead-zone compensation is applied by motor_set_speed_compensated().
 *
 * Starting gains (tune with motor characterization data):
 *   Kp  = 3.0   → increase if slow to reach target; decrease if oscillating
 *   Ki  = 0.5   → increase if persistent steady-state error
 *   Kff = 3.5   → estimate from PWM/speed characterization curve
 *
 * Tuning order:
 *   1. Set Kp = 0, Ki = 0. Increase Kff until robot moves at roughly
 *      the right speed open-loop.
 *   2. Set Kff to found value. Increase Kp until response is fast
 *      without oscillation.
 *   3. Increase Ki slowly until steady-state error disappears.
 */

#include "speed_controller.h"
#include "../hardware/motor.h"
#include "../config/robot_config.h"
#include <math.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  Gain Constants — Tune These
 * ═══════════════════════════════════════════════════════════════════════════ */

/** Feed-forward gain: PWM per mm/s of target speed. */
#define KFF     3.5f

/** Proportional gain. */
#define KP      3.0f

/** Integral gain. */
#define KI      0.5f

/** Anti-windup clamp on integral accumulator (PWM units). */
#define INTEGRAL_LIMIT  1000.0f

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private State
 * ═══════════════════════════════════════════════════════════════════════════ */

static float _left_integral  = 0.0f;
static float _right_integral = 0.0f;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Helper
 * ═══════════════════════════════════════════════════════════════════════════ */

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

void speed_controller_init(void) {
    _left_integral  = 0.0f;
    _right_integral = 0.0f;
}

void speed_controller_update(float target_left_mm_s,
                             float target_right_mm_s,
                             float current_left_mm_s,
                             float current_right_mm_s,
                             float dt) {
    /* ── Left wheel ─────────────────────────────────────────────────────── */
    float err_L   = target_left_mm_s - current_left_mm_s;
    _left_integral = clampf(_left_integral + err_L * dt,
                            -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    float out_L = KFF * target_left_mm_s
                + KP  * err_L
                + KI  * _left_integral;

    out_L = clampf(out_L, -(float)PWM_MAX, (float)PWM_MAX);

    /* ── Right wheel ────────────────────────────────────────────────────── */
    float err_R    = target_right_mm_s - current_right_mm_s;
    _right_integral = clampf(_right_integral + err_R * dt,
                             -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    float out_R = KFF * target_right_mm_s
                + KP  * err_R
                + KI  * _right_integral;

    out_R = clampf(out_R, -(float)PWM_MAX, (float)PWM_MAX);

    /* ── Output with dead-zone compensation ─────────────────────────────── */
    motor_set_speed_compensated(MOTOR_LEFT,  (int16_t)out_L);
    motor_set_speed_compensated(MOTOR_RIGHT, (int16_t)out_R);
}

void speed_controller_reset(void) {
    _left_integral  = 0.0f;
    _right_integral = 0.0f;
}
