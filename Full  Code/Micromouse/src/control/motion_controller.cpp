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
 *   5.4 — 90° turn primitive (gyro-guided)
 *          motion_turn_right_90() / motion_turn_left_90()
 *          P-controller on fused heading: w = −KP_TURN × heading_error
 *          Stops when |error| < TURN_DONE_RAD (≈2°)
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
 * KFF=2.6, KP=5.0, KI=1.0 are baked into speed_controller.cpp (Phase 4).
 */

#include "motion_controller.h"
#include "../config/robot_config.h"
#include "../hardware/encoder.h"
#include "../hardware/motor.h"
#include "../localization/heading_estimator.h"
#include "../localization/odometry.h"
#include "../motion/straight_motion.h"
#include "../sensors/mpu6050.h"
#include "../sensors/distance_manager.h"
#include "velocity_controller.h"
#include "speed_controller.h"
#include "wall_follower.h"

/* ── Heading-correction gain (Step 5.1) ──────────────────────────────────
 * Kp = 2.0 rad/s per rad of heading error.
 * At 1 deg error (0.0175 rad) -> w = 0.035 rad/s -> ~1.7 mm/s differential.
 * Increase if robot still curves; decrease if it oscillates left-right.   */
#define KP_HEADING  2.0f

/* ── Cell size ────────────────────────────────────────────────────────── */
#define CELL_SIZE_MM  180.0f

/* ── Module-level state ───────────────────────────────────────────────── */
static volatile bool _cell_moving = false;
static volatile bool _turning = false;
static volatile float _turn_target_rad = 0.0f;


/* ════════════════════════════════════════════════════════════════════════
 *  Initialisation
 * ════════════════════════════════════════════════════════════════════════ */

void motion_controller_init(void) {
  velocity_controller_init();
  odometry_init();
  wall_follower_init();
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
  /* 1 & 2 & 3. Velocity LPF & Odometry Update
   * NOTE: encoder_update_velocity internally reads encoder_get_delta()
   * and calls odometry_update(). Calling them separately clears the 
   * deltas, resulting in 0 measured speed!
   */
  encoder_update_velocity(CONTROL_LOOP_DT_S);

  /* 4. Heading fusion (gyro + encoder complementary filter)
   *    NOTE: mpu6050_update_filter() is called from loop() at ~200Hz.
   *    Here we only READ the cached filtered value (safe in ISR).       */
  IMUScaledData imu;
  mpu6050_get_filtered(&imu);
  heading_estimator_update(imu.gyro_z_dps, odometry_get_pose().theta_rad,
                           CONTROL_LOOP_DT_S);

  /* 5. Motion control state machine */

  /* ── Step 5.4: Turn state (highest priority — overrides cell drive) ──── */
  if (_turning) {
    float heading_err = heading_estimator_get() - _turn_target_rad;

    /* Normalize to [-π, π] to handle 0↔360 wrap-around */
    while (heading_err >  3.14159f) heading_err -= 6.28318f;
    while (heading_err < -3.14159f) heading_err += 6.28318f;

    if (fabsf(heading_err) > TURN_DONE_RAD) {
      /* P-controller: negative sign because +error needs -w (CW rotation) */
      float w = -KP_TURN * heading_err;
      if (w >  MAX_TURN_RAD_S) w =  MAX_TURN_RAD_S;
      if (w < -MAX_TURN_RAD_S) w = -MAX_TURN_RAD_S;

      /* Anti-stall: ensure it pushes through the last degree */
      if (w > 0.0f && w < MIN_TURN_RAD_S) w = MIN_TURN_RAD_S;
      if (w < 0.0f && w > -MIN_TURN_RAD_S) w = -MIN_TURN_RAD_S;

      velocity_controller_update(0.0f, w);
    } else {
      /* Within deadband — declare turn complete */
      velocity_controller_update(0.0f, 0.0f);
      speed_controller_reset();
      _turning = false;
    }
    return; /* skip cell drive block */
  }

  /* ── Step 5.3: Cell drive (only active when not turning) ─────────────── */
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

  /* 5c-ii. Wall following correction (Step 5.5)
   *   Compute lateral error from side ToF sensors and add PD correction.
   *   Only active when wall_follower is enabled (both walls or at least one). */
  float lateral_err = distance_get_centering_error();
  float wall_w = wall_follower_update(lateral_err, CONTROL_LOOP_DT_S);
  w += wall_w;

  /* 5d. Completion check */
  if (straight_motion_is_complete()) {
    velocity_controller_update(0.0f, 0.0f);
    straight_motion_stop();
    wall_follower_enable(false); /* disable correction when stopped */
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
  motion_drive_cells(1);
}

void motion_drive_cells(int count) {
  /* Reset local pose to (0, 0, 0) so X-distance tracks from the current
   * position rather than the accumulated world position.                 */
  Pose zero = {0.0f, 0.0f, 0.0f};
  odometry_set_pose(zero);

  /* Reset fused heading to 0 so heading correction targets straight ahead
   * regardless of accumulated drift from previous moves.                 */
  heading_estimator_init();

  /* Start trapezoidal profile:
   *   distance   = count * CELL_SIZE_MM
   *   start_speed              = MIN_SPEED_MM_S (overcome static friction deadlock)
   *   end_speed                = 0 mm/s   (stop cleanly)
   *   max/cruise speed         = SEARCH_MAX_SPEED_MM_S = 300 mm/s
   */
  straight_motion_start(count * CELL_SIZE_MM, MIN_SPEED_MM_S, 0.0f, SEARCH_MAX_SPEED_MM_S);

  /* Enable wall following — let ToF sensors centre robot in the corridor */
  wall_follower_reset();
  wall_follower_enable(true);

  /* Arm the ISR — motion_controller_update() takes over from here */
  _cell_moving = true;
}

bool motion_is_cell_moving(void) {
  return _cell_moving;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Phase 5.4 — 90° Turn Primitives
 * ════════════════════════════════════════════════════════════════════════ */

void motion_turn_right_90(void) {
  /* Target = current heading − π/2.
   * A small buffer of 0.010 rad (≈0.6°) accounts for control overshoot
   * and motor deceleration lag. TURN_DONE_RAD deadband = 0.035 rad.
   * Combined effective accuracy = ⊱2°.
   * If turns consistently undershoot: tune GYRO_MULTIPLIER_RIGHT in robot_config.h
   * (formula: new = old × actual_degrees / 90.0).
   */
  mpu6050_set_stationary(false);   /* disable drift correction during turn */
  _turn_target_rad = heading_estimator_get() - 1.5708f - 0.010f;
  speed_controller_reset();
  _turning = true;
}

void motion_turn_left_90(void) {
  /* Target = current heading + π/2.
   * Same 0.010 rad buffer as right turn.
   * If turns consistently undershoot: tune GYRO_MULTIPLIER_LEFT in robot_config.h.
   */
  mpu6050_set_stationary(false);   /* disable drift correction during turn */
  _turn_target_rad = heading_estimator_get() + 1.5708f + 0.010f;
  speed_controller_reset();
  _turning = true;
}

bool motion_is_turning(void) {
  return _turning;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Generic command dispatcher (Phase 6+)
 * ════════════════════════════════════════════════════════════════════════ */

void motion_execute_command(const MotionCommand *cmd) {
  if (!cmd) return;
  switch (cmd->type) {
    case CMD_FORWARD:
      /* Drive multiple cells in one shot (path smoother groups straights) */
      motion_drive_cells((cmd->count > 0) ? cmd->count : 1);
      break;
    case CMD_TURN_RIGHT:
      motion_turn_right_90();
      break;
    case CMD_TURN_LEFT:
      motion_turn_left_90();
      break;
    default:
      /* Unknown command — safe no-op */
      break;
  }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Status / Emergency
 * ════════════════════════════════════════════════════════════════════════ */

bool motion_is_idle(void) {
  return !_cell_moving && !_turning;
}

void motion_emergency_stop(void) {
  /* Immediately disarm both the cell drive and the turn state */
  _cell_moving = false;
  _turning     = false;
  straight_motion_stop();
  motor_stop();
}
