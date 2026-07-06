/**
 * @file motion_profile.h
 * @brief Velocity profile generator for smooth, fast micromouse motion.
 *
 * Provides three profile types:
 *   1. Trapezoidal — basic accel/cruise/decel (Phase 4 baseline)
 *   2. S-Curve    — jerk-limited smooth acceleration transitions
 *   3. Rolling Turn — arc profiles for turning while moving
 *
 * All functions are pure math (no hardware dependencies) — safe to
 * unit-test on desktop and use on STM32.
 *
 * Usage: call profile_compute() once per move, then call
 *        profile_get_speed() every control loop tick (1kHz) with
 *        the current distance traveled to get the target speed.
 */

#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include "config.h"
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PI
#define PI 3.14159265358979f
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  Profile Types
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef enum {
    PROFILE_TRAPEZOID,      /**< Standard trapezoidal velocity profile     */
    PROFILE_S_CURVE,        /**< Jerk-limited S-curve profile              */
    PROFILE_ARC_TURN        /**< Constant-speed circular arc (rolling turn)*/
} ProfileType;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Linear Profile (Straight-line moves)
 *
 *  Computes target speed given distance traveled for a straight segment.
 *  Handles: acceleration → cruise → deceleration.
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef struct {
    ProfileType type;

    /* Input parameters */
    float total_dist_mm;    /**< Total distance to travel                   */
    float max_speed;        /**< Maximum cruise speed (mm/s)                */
    float accel;            /**< Acceleration rate (mm/s²)                  */
    float decel;            /**< Deceleration rate (mm/s²)                  */
    float start_speed;      /**< Speed at profile start (mm/s), 0 or entry */
    float end_speed;        /**< Speed at profile end (mm/s), 0 or exit    */

    /* Computed phase boundaries (set by profile_compute) */
    float accel_dist;       /**< Distance spent accelerating               */
    float decel_dist;       /**< Distance spent decelerating               */
    float cruise_dist;      /**< Distance at cruise speed                  */
    float peak_speed;       /**< Actual peak speed reached (may be < max)  */

    /* S-curve specific */
    float jerk;             /**< Jerk limit (mm/s³)                        */
} LinearProfile;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Arc Turn Profile (Rolling turn)
 *
 *  The robot traces a circular arc of specified radius and angle.
 *  Linear speed along the arc is maintained (angular velocity = v/r).
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef struct {
    float radius_mm;        /**< Turn radius (from config)                 */
    float angle_rad;        /**< Turn angle: π/2 for 90°, π for 180°      */
    float arc_length_mm;    /**< = radius × angle (total distance along arc) */
    float linear_speed;     /**< Constant linear speed during arc (mm/s)   */
    float angular_speed;    /**< = linear_speed / radius (rad/s)           */
    float duration_s;       /**< = arc_length / linear_speed               */
    float entry_decel_dist; /**< Distance to decel from cruise to turn speed */
    float exit_accel_dist;  /**< Distance to accel from turn speed to cruise */
    bool  is_left;          /**< true = left turn, false = right turn      */
} ArcTurnProfile;

/* ═══════════════════════════════════════════════════════════════════════════
 *  API — Profile Computation
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * Compute a linear (straight-line) velocity profile.
 * After calling this, use profile_get_speed() to query target speed.
 *
 * @param p           The profile to initialize
 * @param dist_mm     Total distance to travel
 * @param max_speed   Maximum cruise speed (mm/s)
 * @param accel       Acceleration (mm/s²)
 * @param decel       Deceleration (mm/s²)
 * @param start_speed Initial speed (0 for start-from-stop, or entry speed)
 * @param end_speed   Final speed (0 for stop, or exit speed for next segment)
 */
void profile_compute_linear(LinearProfile *p,
                            float dist_mm,
                            float max_speed,
                            float accel,
                            float decel,
                            float start_speed,
                            float end_speed);

/**
 * Get the target speed at a given distance along a linear profile.
 *
 * @param p           The computed profile
 * @param dist_so_far Distance traveled so far (mm)
 * @return            Target speed at this point (mm/s)
 */
float profile_get_speed(const LinearProfile *p, float dist_so_far);

/**
 * Compute a rolling (arc) turn profile.
 *
 * @param t            The arc profile to initialize
 * @param radius_mm    Turn radius (mm)
 * @param angle_deg    Turn angle in degrees (90 or 180)
 * @param turn_speed   Linear speed to maintain during the arc (mm/s)
 * @param cruise_speed Current cruise speed (mm/s) — for computing decel/accel ramps
 * @param accel        Accel/decel rate for speed transition (mm/s²)
 * @param is_left      true = turn left, false = turn right
 */
void profile_compute_arc(ArcTurnProfile *t,
                         float radius_mm,
                         float angle_deg,
                         float turn_speed,
                         float cruise_speed,
                         float accel,
                         bool is_left);

/**
 * Get target linear speed and angular velocity at a given arc distance.
 *
 * @param t            The computed arc profile
 * @param arc_dist     Distance traveled along the arc (mm)
 * @param out_speed    Output: target linear speed (mm/s)
 * @param out_omega    Output: target angular velocity (rad/s, + = left)
 */
void profile_get_arc_state(const ArcTurnProfile *t,
                           float arc_dist,
                           float *out_speed,
                           float *out_omega);

/* ═══════════════════════════════════════════════════════════════════════════
 *  API — Differential Drive Motor Commands
 *
 *  Converts (linear_speed, angular_velocity) into left/right wheel speeds.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * Compute individual wheel speeds from unicycle model.
 *
 * @param linear_speed  Forward speed (mm/s)
 * @param omega         Angular velocity (rad/s, positive = counter-clockwise/left)
 * @param wheel_base_mm Distance between wheel centers (mm)
 * @param out_left      Output: left wheel speed (mm/s)
 * @param out_right     Output: right wheel speed (mm/s)
 */
static inline void profile_to_wheel_speeds(float linear_speed,
                                           float omega,
                                           float wheel_base_mm,
                                           float *out_left,
                                           float *out_right)
{
    *out_left  = linear_speed - (omega * wheel_base_mm / 2.0f);
    *out_right = linear_speed + (omega * wheel_base_mm / 2.0f);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  API — Look-Ahead Speed Planning
 *
 *  Given the next N commands, compute the maximum safe exit speed from
 *  the current segment so we can decelerate smoothly into a turn.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * Compute the maximum exit speed for the current straight segment,
 * given that the next command is a turn.
 *
 * If next command is a smooth turn: exit speed = turn's linear speed.
 * If next command is an in-place turn: exit speed = 0.
 * If next command is another straight: exit speed = cruise speed.
 *
 * @param next_cmd_type  The CommandType of the next command
 * @param cruise_speed   Current cruise speed (mm/s)
 * @return               Safe exit speed (mm/s)
 */
float profile_exit_speed_for_next(int next_cmd_type, float cruise_speed);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_PROFILE_H */
