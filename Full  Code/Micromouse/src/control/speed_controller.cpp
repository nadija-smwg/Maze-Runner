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
#include "../config/robot_config.h"
#include "../hardware/motor.h"
#include <math.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  Gain Constants — Tune These
 * ═══════════════════════════════════════════════════════════════════════════
 */

/*
 * Measured characterization data (Phase 2, STBY=HIGH, battery ~7.4V):
 *   PWM 1500 → L = 178.0 mm/s,  R = 179.9 mm/s
 *
 *   KFF  = PWM / speed = 1500 / 179 ≈ 8.4
 *
 * Re-run characterization if battery voltage changes significantly
 * (a fully charged 2S at 8.4V will run faster than at 7.4V).
 *
 * Tuning order:
 *   1. Set Kp=0, Ki=0. Increase KFF until robot moves at roughly the right
 *      speed open-loop. (Use measured value above as starting point.)
 *   2. Set KFF to measured value. Increase Kp until response is fast
 *      without oscillation.
 *   3. Increase Ki slowly until steady-state error disappears.
 */

// ==========================================
// TUNING CONSTANTS — UPDATE AFTER PHASE 4 TUNING SESSION
// ==========================================
// Target: 150 mm/s | Tune using PHASE_4_TEST_MODE
//
// Step 1 — KFF only (KP=0, KI=0):
//   Press BTN_START -> 150 mm/s step. Adjust KFF until wheels
//   reach ~150 mm/s open-loop (no oscillation, may have offset).
//   Good starting range: 2.0 – 4.0. Keep incrementing by 0.2.
//
// Step 2 — Add KP (KI=0):
//   Increase KP until response is fast without oscillating.
//   Good starting range: 5.0 – 20.0. Increment by 0.5.
//
// Step 3 — Add KI:
//   Increase KI slowly until steady-state speed error disappears.
//   Good starting range: 0.5 – 3.0. Increment by 0.1.
//
// When settled: copy found values below, set PHASE_5_TEST_MODE=1.

#define SPEED_KP 5.0f
#define SPEED_KI 1.0f
#define SPEED_KD 0.0f
#define SPEED_KFF 2.6f // Integral — reset, tune after KP settled

/** Anti-windup clamp on integral accumulator (PWM units). */
#define INTEGRAL_LIMIT 1000.0f

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private State
 * ═══════════════════════════════════════════════════════════════════════════
 */

static float _left_integral = 0.0f;
static float _right_integral = 0.0f;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Helper
 * ═══════════════════════════════════════════════════════════════════════════
 */

static float clampf(float v, float lo, float hi) {
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════
 */

void speed_controller_init(void) {
  _left_integral = 0.0f;
  _right_integral = 0.0f;
}

void speed_controller_update(float target_left_mm_s, float target_right_mm_s,
                             float current_left_mm_s, float current_right_mm_s,
                             float dt) {
  /* ── Left wheel ─────────────────────────────────────────────────────── */
  float out_L = 0.0f;
  float err_L = target_left_mm_s - current_left_mm_s;

  if (target_left_mm_s == 0.0f && fabsf(current_left_mm_s) < 10.0f) {
    _left_integral = 0.0f; // Clear windup when completely stopped
    out_L = 0.0f;          // Disable motor to prevent jitter
  } else {
    _left_integral =
        clampf(_left_integral + err_L * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    out_L = SPEED_KFF * target_left_mm_s + SPEED_KP * err_L +
            SPEED_KI * _left_integral;
    out_L = clampf(out_L, -(float)PWM_MAX, (float)PWM_MAX);
  }

  /* ── Right wheel ────────────────────────────────────────────────────── */
  float out_R = 0.0f;
  float err_R = target_right_mm_s - current_right_mm_s;

  if (target_right_mm_s == 0.0f && fabsf(current_right_mm_s) < 10.0f) {
    _right_integral = 0.0f; // Clear windup when completely stopped
    out_R = 0.0f;           // Disable motor to prevent jitter
  } else {
    _right_integral =
        clampf(_right_integral + err_R * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    out_R = SPEED_KFF * target_right_mm_s + SPEED_KP * err_R +
            SPEED_KI * _right_integral;
    out_R = clampf(out_R, -(float)PWM_MAX, (float)PWM_MAX);
  }

  /* ── Output with dead-zone compensation ─────────────────────────────── */
  motor_set_speed_compensated(MOTOR_LEFT, (int16_t)out_L);
  motor_set_speed_compensated(MOTOR_RIGHT, (int16_t)out_R);
}

void speed_controller_reset(void) {
  _left_integral = 0.0f;
  _right_integral = 0.0f;
}
