/**
 * @file motion_controller.cpp
 * @brief Top-level motion controller — Phase 5 implementation.
 * @see motion_controller.h
 *
 * Phase 5 Steps implemented here:
 *
 *   5.1 — Heading-corrected straight drive
 *          heading_error = 0 − fused_heading
 *          w_correction  = KP_HEADING * heading_error  (Kp = 2.0 rad/s/rad)
 *
 *   5.2 — Trapezoidal motion profile
 *          profile: 0 → SEARCH_MAX_SPEED_MM_S → 0 over target distance
 *          handled by straight_motion.cpp / motion_profile.c
 *
 *   5.3 — Cell drive primitive (180mm)
 *          motion_drive_cell() — resets local pose, starts profile
 *          motion_controller_update() — 1kHz ISR, drives until done
 *
 * 1kHz ISR pipeline (motion_controller_update):
 *   encoder_get_delta()         -> raw ticks this ms
 *   encoder_update_velocity()   -> EMA-smoothed mm/s
 *   odometry_update()           -> X, Y, theta integration
 *   heading_estimator_update()  -> 98% gyro + 2% encoder fused heading
 *   straight_motion_update()    -> trapezoidal target speed
 *   heading correction          -> angular velocity w
 *   velocity_controller_update(v, w) -> wheel speeds -> PWM
 *   completion check            -> auto-stop at end of cell
 *
 * KFF=2.4, KP=13.0, KI=2.0 are baked into speed_controller.cpp (Phase 4).
 */

#include "motion_controller.h"
#include "../config/robot_config.h"
#include "../hardware/encoder.h"
#include "../hardware/motor.h"
#include "../localization/heading_estimator.h"
#include "../localization/odometry.h"
#include "../motion/straight_motion.h"
#include "../sensors/mpu6050.h"
#include "velocity_controller.h"

/* ── Heading-correction gain (Step 5.1) ──────────────────────────────────
 * Kp = 2.0 rad/s per rad of heading error.
 * At 1 deg error (0.0175 rad) -> w = 0.035 rad/s -> ~1.7 mm/s differential.
 * Increase if robot still curves; decrease if it oscillates left-right.   */
#define KP_HEADING  2.0f

/* ── Cell size ────────────────────────────────────────────────────────── */
#define CELL_SIZE_MM  180.0f

/* ── Module-level state ───────────────────────────────────────────────── */
static volatile bool _cell_moving = false;


/* ════════════════════════════════════════════════════════════════════════
 *  Initialisation
 * ════════════════════════════════════════════════════════════════════════ */

void motion_controller_init(void) {
  velocity_controller_init();
  odometry_init();
  heading_estimator_init();
  straight_motion_stop();
  _cell_moving = false;
}

/* ════════════════════════════════════════════════════════════════════════
 *  1kHz ISR — motion_controller_update()
 *
 *  Called by the hardware timer every 1ms.
 *  Executes the full sensor -> profile -> control -> motor pipeline.
 * ════════════════════════════════════════════════════════════════════════ */

void motion_controller_update(void) {
  /* 1. Encoder deltas (raw ticks since last ms) */
  int32_t l_ticks = encoder_get_delta(ENCODER_LEFT);
  int32_t r_ticks = encoder_get_delta(ENCODER_RIGHT);

  /* 2. Velocity LPF (EMA-smoothed mm/s for PI controller) */
  encoder_update_velocity(CONTROL_LOOP_DT_S);

  /* 3. Odometry — integrate X, Y, theta */
  float l_mm = (float)l_ticks * LEFT_MM_PER_COUNT;
  float r_mm = (float)r_ticks * RIGHT_MM_PER_COUNT;
  odometry_update(l_mm, r_mm);

  /* 4. Heading fusion (gyro + encoder complementary filter)
   *    NOTE: mpu6050_update_filter() is called from loop() at ~200Hz.
   *    Here we only READ the cached filtered value (safe in ISR).       */
  IMUScaledData imu;
  mpu6050_get_filtered(&imu);
  heading_estimator_update(imu.gyro_z_dps, odometry_get_pose().theta_rad,
                           CONTROL_LOOP_DT_S);

  /* 5. Motion control — only when a cell drive is active */
  if (!_cell_moving) {
    /* Idle — hold zero velocity (integrators stay cleared) */
    velocity_controller_update(0.0f, 0.0f);
    return;
  }

  /* 5a. Distance traveled = absolute X from odometry */
  float dist = odometry_get_pose().x_mm;
  if (dist < 0.0f) dist = -dist; /* guard against tiny reverse rollback */

  /* 5b. Trapezoidal profile -> target linear speed (mm/s) */
  straight_motion_update(dist);
  float v = straight_motion_get_target_speed();

  /* 5c. Heading correction (Step 5.1)
   *    error = 0 - fused_heading (target heading = 0 rad = straight ahead)
   *    w     = Kp * error                                                 */
  float heading_err = 0.0f - heading_estimator_get();
  float w = KP_HEADING * heading_err;

  /* 5d. Completion check */
  if (straight_motion_is_complete()) {
    velocity_controller_update(0.0f, 0.0f);
    straight_motion_stop();
    _cell_moving = false;
    return;
  }

  /* 5e. Send velocities to speed controller -> motors */
  velocity_controller_update(v, w);
}

/* ════════════════════════════════════════════════════════════════════════
 *  Phase 5.3 — Cell Drive Primitive
 * ════════════════════════════════════════════════════════════════════════ */

void motion_drive_cell(void) {
  /* Reset local pose to (0, 0, 0) so X-distance tracks from the current
   * position rather than the accumulated world position.                 */
  Pose zero = {0.0f, 0.0f, 0.0f};
  odometry_set_pose(zero);

  /* Reset fused heading to 0 so heading correction targets straight ahead
   * regardless of accumulated drift from previous moves.                 */
  heading_estimator_init();

  /* Start trapezoidal profile:
   *   distance   = CELL_SIZE_MM  = 180 mm
   *   start_speed              = 0 mm/s   (from standstill)
   *   end_speed                = 0 mm/s   (stop cleanly)
   *   max/cruise speed         = SEARCH_MAX_SPEED_MM_S = 300 mm/s
   *
   * Profile phases (accel = decel = 1500 mm/s2):
   *   Accel:  0 -> 300 mm/s over (300^2)/(2*1500) = 30 mm
   *   Cruise: 300 mm/s for 180 - 30 - 30 = 120 mm
   *   Decel:  300 -> 0 mm/s over 30 mm
   */
  straight_motion_start(CELL_SIZE_MM, 0.0f, 0.0f, SEARCH_MAX_SPEED_MM_S);

  /* Arm the ISR — motion_controller_update() takes over from here */
  _cell_moving = true;
}

bool motion_is_cell_moving(void) {
  return _cell_moving;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Generic command dispatcher (Phase 6+)
 * ════════════════════════════════════════════════════════════════════════ */

void motion_execute_command(const MotionCommand *cmd) {
  /**
   * TODO (Phase 6):
   * Switch on cmd->type and delegate to cell_controller or turn_controller.
   */
  (void)cmd; /* suppress unused-parameter warning */
}

/* ════════════════════════════════════════════════════════════════════════
 *  Status / Emergency
 * ════════════════════════════════════════════════════════════════════════ */

bool motion_is_idle(void) {
  return !_cell_moving;
}

void motion_emergency_stop(void) {
  /* Immediately disarm profile and cut motor power */
  _cell_moving = false;
  straight_motion_stop();
  motor_stop();
}
