/**
 * @file wall_follower.cpp
 * @brief Wall follower implementation — Step 5.5.
 *
 * ── Architecture ──────────────────────────────────────────────────────────
 *
 *   distance_get_centering_error()      (mm, from ToF sensors)
 *           │
 *   PD controller  (Kp × err + Kd × d_err/dt)
 *           │
 *   angular_correction (rad/s)  ← clamped to MAX_WALL_CORRECTION_RAD_S
 *           │
 *   velocity_controller_update(v, heading_w + wall_w)
 *
 * The wall follower outputs an angular velocity correction (rad/s) that is
 * ADDED on top of the gyro heading-correction angular velocity already
 * computed in motion_controller.cpp.  Both corrections are fed together
 * into the unicycle model.
 *
 * ── Calibration Constants (in robot_config.h) ─────────────────────────────
 *   KP_WALL   — Proportional gain (rad/s per mm of lateral error)
 *               Start at 0.01.  If robot wiggles → decrease.
 *               If robot takes too long to center → increase.
 *   KD_WALL   — Derivative gain (damps oscillation)
 *               Start at 0.005. If robot overshoots → increase.
 *   MAX_WALL_CORRECTION_RAD_S — Safety clamp on correction output.
 *   TARGET_WALL_DIST_MM       — Target side clearance (in distance_manager.cpp)
 */

#include "wall_follower.h"
#include "../config/robot_config.h"
#include <math.h>

/* ─── Private state ─────────────────────────────────────────────────────── */

/** PID object (P + D only; integral causes windup in long corridors) */
static PID _pid(KP_WALL, 0.0f, KD_WALL, -MAX_WALL_CORRECTION_RAD_S, MAX_WALL_CORRECTION_RAD_S);

/** true when wall following is active */
static bool _enabled = false;

/* ─── Public API ────────────────────────────────────────────────────────── */

void wall_follower_init(void) {
    _pid.set_gains(KP_WALL, 0.0f, KD_WALL);
    _enabled = false;
}

float wall_follower_update(float lateral_error_mm, float dt) {
    if (!_enabled) return 0.0f;

    /*
     * Compute PD correction.
     * Setpoint = 0 mm (perfectly centred, error = 0).
     * Measurement = lateral_error_mm (positive = shifted left).
     * Output = angular_velocity correction (rad/s).
     *   Positive output → robot steers RIGHT (towards centre if shifted left)
     */
    return _pid.compute(0.0f, lateral_error_mm, dt);
}

void wall_follower_enable(bool enable) {
    _enabled = enable;
    if (!enable) {
        _pid.reset();
    }
}

void wall_follower_reset(void) {
    _pid.reset();
}

bool wall_follower_is_enabled(void) {
    return _enabled;
}

void wall_follower_set_gains(float kp, float kd) {
    _pid.set_gains(kp, 0.0f, kd);
}
