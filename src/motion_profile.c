/**
 * @file motion_profile.c
 * @brief Velocity profile generator implementation.
 *
 * Implements:
 *   1. Trapezoidal profile: instant accel/decel transitions
 *   2. S-curve profile: jerk-limited smooth transitions
 *   3. Arc turn profile: constant-speed circular arc for rolling turns
 *   4. Look-ahead exit speed computation
 *
 * All pure math — no hardware dependencies.
 */

#include "motion_profile.h"
#include "maze.h"  /* for CommandType enum */
#include <math.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  Helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

static inline float fminf_safe(float a, float b) { return a < b ? a : b; }
static inline float fmaxf_safe(float a, float b) { return a > b ? a : b; }
static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Linear Profile Computation
 *
 *  Three phases:
 *    1. Accelerate from start_speed toward peak_speed
 *    2. Cruise at peak_speed
 *    3. Decelerate from peak_speed to end_speed
 *
 *  If distance is too short to reach max_speed, peak_speed is capped
 *  at what's achievable (triangular profile — no cruise phase).
 * ═══════════════════════════════════════════════════════════════════════════ */

void profile_compute_linear(LinearProfile *p,
                            float dist_mm,
                            float max_speed,
                            float accel,
                            float decel,
                            float start_speed,
                            float end_speed)
{
#if ENABLE_S_CURVE
    p->type = PROFILE_S_CURVE;
    p->jerk = JERK_LIMIT_MM_S3;
#else
    p->type = PROFILE_TRAPEZOID;
    p->jerk = 0;
#endif

    p->total_dist_mm = dist_mm;
    p->max_speed = max_speed;
    p->accel = accel;
    p->decel = decel;
    p->start_speed = fmaxf_safe(start_speed, 0.0f);
    p->end_speed = fmaxf_safe(end_speed, 0.0f);

    /* Distance needed to accelerate from start_speed to max_speed:
     *   d_accel = (v_max² - v_start²) / (2 * a)                            */
    float d_accel_full = (max_speed * max_speed - p->start_speed * p->start_speed) / (2.0f * accel);

    /* Distance needed to decelerate from max_speed to end_speed:
     *   d_decel = (v_max² - v_end²) / (2 * a)                              */
    float d_decel_full = (max_speed * max_speed - p->end_speed * p->end_speed) / (2.0f * decel);

    if (d_accel_full + d_decel_full > dist_mm) {
        /* Can't reach max_speed — compute achievable peak (triangular profile).
         *
         * The peak speed where accel_dist + decel_dist = total_dist:
         *   v_peak² = (2 * dist * a_accel * a_decel + a_decel * v_start² + a_accel * v_end²)
         *             / (a_accel + a_decel)                                 */
        float v_peak_sq = (2.0f * dist_mm * accel * decel
                           + decel * p->start_speed * p->start_speed
                           + accel * p->end_speed * p->end_speed)
                          / (accel + decel);

        if (v_peak_sq < 0) v_peak_sq = 0;
        p->peak_speed = sqrtf(v_peak_sq);
        p->accel_dist = (p->peak_speed * p->peak_speed - p->start_speed * p->start_speed) / (2.0f * accel);
        p->decel_dist = (p->peak_speed * p->peak_speed - p->end_speed * p->end_speed) / (2.0f * decel);
        p->cruise_dist = 0;  /* no cruise phase */
    } else {
        /* Full trapezoidal: accel → cruise → decel */
        p->peak_speed = max_speed;
        p->accel_dist = d_accel_full;
        p->decel_dist = d_decel_full;
        p->cruise_dist = dist_mm - p->accel_dist - p->decel_dist;
    }

    /* Clamp to non-negative (floating point imprecision guard) */
    p->accel_dist = fmaxf_safe(p->accel_dist, 0.0f);
    p->decel_dist = fmaxf_safe(p->decel_dist, 0.0f);
    p->cruise_dist = fmaxf_safe(p->cruise_dist, 0.0f);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Speed Query (called at 1kHz from the control loop)
 * ═══════════════════════════════════════════════════════════════════════════ */

float profile_get_speed(const LinearProfile *p, float dist_so_far)
{
    float d = clampf(dist_so_far, 0.0f, p->total_dist_mm);

    if (d <= p->accel_dist) {
        /* ── Acceleration phase ────────────────────────────────────────── */
        /* v² = v_start² + 2 * a * d */
        float v_sq = p->start_speed * p->start_speed + 2.0f * p->accel * d;
        float v = sqrtf(fmaxf_safe(v_sq, 0.0f));

#if ENABLE_S_CURVE
        /* S-curve modification: blend acceleration smoothly using a
         * sinusoidal ramp factor. This approximates jerk-limited behavior
         * without the full 7-phase S-curve math (which is overkill for
         * a micromouse at 1kHz update rate).
         *
         * ramp = sin²(π/2 * d/d_accel) → 0 at start, 1 at end of accel
         * v_s_curve = v_start + (v_peak - v_start) * ramp
         */
        if (p->accel_dist > 0.01f) {
            float phase = (PI / 2.0f) * (d / p->accel_dist);
            float ramp = sinf(phase);
            ramp = ramp * ramp;  /* sin² for S-shape */
            v = p->start_speed + (p->peak_speed - p->start_speed) * ramp;
        }
#endif
        return fminf_safe(v, p->peak_speed);

    } else if (d <= p->accel_dist + p->cruise_dist) {
        /* ── Cruise phase ──────────────────────────────────────────────── */
        return p->peak_speed;

    } else {
        /* ── Deceleration phase ────────────────────────────────────────── */
        float d_into_decel = d - p->accel_dist - p->cruise_dist;
        float remaining = p->decel_dist - d_into_decel;
        remaining = fmaxf_safe(remaining, 0.0f);

        /* v² = v_end² + 2 * a * remaining */
        float v_sq = p->end_speed * p->end_speed + 2.0f * p->decel * remaining;
        float v = sqrtf(fmaxf_safe(v_sq, 0.0f));

#if ENABLE_S_CURVE
        /* S-curve deceleration: mirror of acceleration ramp */
        if (p->decel_dist > 0.01f) {
            float phase = (PI / 2.0f) * (remaining / p->decel_dist);
            float ramp = sinf(phase);
            ramp = ramp * ramp;
            v = p->end_speed + (p->peak_speed - p->end_speed) * ramp;
        }
#endif
        return fmaxf_safe(v, p->end_speed);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Arc Turn Profile
 *
 *  Geometry of a rolling 90° turn:
 *
 *      ╭──────╮  arc_length = radius × angle_rad
 *     ╱        ╲        = 45mm × π/2 ≈ 70.7mm for a 90° turn
 *    ╱          ╲
 *   entry      exit
 *
 *  The robot follows this arc at constant linear speed while the
 *  angular velocity ω = v/r produces the heading change.
 *
 *  Wheel speed differential during the arc:
 *    v_left  = v - ω × wheelbase/2   (inner wheel, slower)
 *    v_right = v + ω × wheelbase/2   (outer wheel, faster)
 * ═══════════════════════════════════════════════════════════════════════════ */

void profile_compute_arc(ArcTurnProfile *t,
                         float radius_mm,
                         float angle_deg,
                         float turn_speed,
                         float cruise_speed,
                         float accel,
                         bool is_left)
{
    t->radius_mm = radius_mm;
    t->angle_rad = angle_deg * PI / 180.0f;
    t->arc_length_mm = radius_mm * t->angle_rad;
    t->linear_speed = turn_speed;
    t->angular_speed = turn_speed / radius_mm;  /* rad/s */
    t->duration_s = t->arc_length_mm / fmaxf_safe(turn_speed, 1.0f);
    t->is_left = is_left;

    /* Decel/accel distances for speed transition cruise ↔ turn_speed */
    if (cruise_speed > turn_speed) {
        t->entry_decel_dist = (cruise_speed * cruise_speed - turn_speed * turn_speed) / (2.0f * accel);
        t->exit_accel_dist = t->entry_decel_dist;  /* symmetric */
    } else {
        t->entry_decel_dist = 0;
        t->exit_accel_dist = 0;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Arc State Query
 *
 *  During the arc, linear speed is constant and angular velocity is constant.
 *  The control loop uses these to compute wheel speed differential.
 * ═══════════════════════════════════════════════════════════════════════════ */

void profile_get_arc_state(const ArcTurnProfile *t,
                           float arc_dist,
                           float *out_speed,
                           float *out_omega)
{
    /* Constant speed throughout the arc */
    *out_speed = t->linear_speed;

    /* Angular velocity: positive = left turn (counter-clockwise) */
    float omega = t->angular_speed;
    if (!t->is_left) omega = -omega;
    *out_omega = omega;

    /* If we're past the arc, stop turning */
    if (arc_dist >= t->arc_length_mm) {
        *out_omega = 0;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Look-Ahead Exit Speed
 *
 *  The key to smooth transitions: before finishing a straight segment,
 *  check what's coming next and pre-plan deceleration.
 *
 *  Next command     →  Exit speed of current straight
 *  ─────────────────────────────────────────────────────
 *  CMD_STRAIGHT     →  cruise (no decel needed)
 *  CMD_SMOOTH_*_90  →  MAX_TURN_SPEED_MM_S (decel to turn speed)
 *  CMD_SMOOTH_*_180 →  MAX_TURN_SPEED_MM_S * 0.7 (slower for hairpin)
 *  CMD_TURN_*       →  0 (full stop for in-place turn)
 *  End of path      →  0 (full stop)
 * ═══════════════════════════════════════════════════════════════════════════ */

float profile_exit_speed_for_next(int next_cmd_type, float cruise_speed)
{
    switch ((CommandType)next_cmd_type) {
        case CMD_STRAIGHT:
        case CMD_DIAGONAL:
            return cruise_speed;  /* no decel needed */

        case CMD_SMOOTH_LEFT_90:
        case CMD_SMOOTH_RIGHT_90:
        case CMD_SS_LEFT_90:
        case CMD_SS_RIGHT_90:
            return fminf_safe(MAX_TURN_SPEED_MM_S, cruise_speed);

        case CMD_SMOOTH_LEFT_180:
        case CMD_SMOOTH_RIGHT_180:
            return fminf_safe(MAX_TURN_SPEED_MM_S * 0.7f, cruise_speed);

        case CMD_TURN_LEFT_90:
        case CMD_TURN_RIGHT_90:
        case CMD_TURN_180:
        default:
            return 0.0f;  /* full stop before in-place turn */
    }
}
