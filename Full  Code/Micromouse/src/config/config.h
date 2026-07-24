/**
 * @file config.h
 * @brief Centralized tuning constants for the micromouse maze-solving suite.
 *
 * All competition-critical parameters live here so you can retune at the venue
 * without hunting through multiple files. Group: maze geometry, algorithm costs,
 * motion limits.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/* ─── Maze Geometry ────────────────────────────────────────────────────── */
#define MAZE_SIZE          16       /* standard micromouse: 16×16 cells       */
#define CELL_SIZE_MM       180      /* physical cell edge length in mm        */

/* Goal center block (0-indexed). Standard 16×16 goal = center 2×2 block.    */
static const uint8_t GOAL_CELLS[][2] = {
    {7, 7}, {7, 8}, {8, 7}, {8, 8}
};
#define NUM_GOAL_CELLS     4

/* Start cell — bottom-left corner, standard competition placement.          */
#define START_X            0
#define START_Y            0

/* ─── Flood Fill ───────────────────────────────────────────────────────── */
#define FLOOD_INFINITY     0xFFFF   /* unreachable / uninitialized sentinel   */

/* ─── Weighted Dijkstra Cost Model ─────────────────────────────────────── */
/*
 * These are relative cost units (not physical time, but proportional to it).
 * Tune them to match your robot's actual speed/turn characteristics:
 *
 *   A straight cell traversal at cruise speed might take ~100ms.
 *   A 90° in-place turn (decel + rotate + accel) might take ~150ms.
 *   A 180° turn is roughly 2× a 90° turn.
 *
 * The ratio matters more than absolute values. 10:12 straight:turn means
 * the algorithm will accept a path 1 cell longer if it avoids a turn.
 */
#define COST_STRAIGHT      10      /* cost to traverse one cell straight     */
#define COST_TURN_90       12      /* additional cost for a 90° turn         */
#define COST_TURN_180      30      /* additional cost for a 180° turn        */

/* Diagonal traversal cost (sqrt(2) × straight ≈ 14, but diagonals are
 * mechanically harder, so add a small penalty).                             */
#define COST_DIAGONAL      15      /* cost for one diagonal cell traverse    */
#define ENABLE_DIAGONALS   0       /* set to 1 when your robot can do 45°   */

/* ─── Path / Command Limits ────────────────────────────────────────────── */
#define MAX_PATH_LENGTH    256     /* max waypoints in reconstructed path    */
#define MAX_COMMANDS       128     /* max motion commands after smoothing    */

/* ─── Dijkstra State Space ─────────────────────────────────────────────── */
/* State = (x, y, heading), total states = 16×16×4 = 1024                   */
#define NUM_DIRECTIONS     4
#define DIJKSTRA_STATES    (MAZE_SIZE * MAZE_SIZE * NUM_DIRECTIONS)

/* ─── Algorithm Tuning ─────────────────────────────────────────────────── */
/* When choosing next direction in flood fill, prefer continuing straight
 * over turning to reduce unnecessary turns during search run.               */
#define PREFER_STRAIGHT    1

/* ─── Motion Profile — Speed & Acceleration ────────────────────────── */
/*
 * These are physical units. Tune to YOUR robot's measured capabilities.
 * Start conservative, increase after PID tuning is solid.
 */

/* Search run (conservative — prioritize sensing accuracy) */
#define SEARCH_MAX_SPEED_MM_S    300.0f   /* mm/s max forward speed        */
#define SEARCH_ACCEL_MM_S2       800.0f   /* mm/s² acceleration            */
#define SEARCH_DECEL_MM_S2       800.0f   /* mm/s² deceleration            */

/* Fast run (aggressive — push for time after maze is known) */
#define FAST_MAX_SPEED_MM_S      800.0f   /* mm/s — push higher as tuning improves */
#define FAST_ACCEL_MM_S2        2000.0f   /* mm/s² */
#define FAST_DECEL_MM_S2        2500.0f   /* mm/s² — decel can be harder than accel */

/* ─── Smooth Turn Geometry ─────────────────────────────────────────── */
/*
 * Rolling turn = arc at constant radius, no stop.
 * The robot traces a quarter-circle (90°) or half-circle (180°) through
 * the cell junction. The arc radius determines how much the robot cuts
 * the corner vs. hugging the center line.
 *
 *              ┌─────────┐
 *              │         │
 *              │    ╭──→ │  ← rolling 90° turn arc
 *              │    │    │     radius = TURN_RADIUS_MM
 *  ──entry───→ │    ╯    │
 *              │         │
 *              └─────────┘
 *
 * A radius of CELL_SIZE_MM/2 (90mm) traces the arc exactly through
 * the cell center. Smaller = tighter turn, needs more grip.
 */
#define TURN_RADIUS_90_MM       45.0f    /* radius for 90° rolling turn    */
#define TURN_RADIUS_180_MM      40.0f    /* radius for 180° rolling turn   */

/* Maximum angular velocity during turns (rad/s).
 * v_angular = v_linear / turn_radius
 * At 300mm/s, radius 45mm → 6.67 rad/s (≈ 382°/s) — fast but doable. */
#define MAX_TURN_SPEED_MM_S     300.0f   /* linear speed maintained during rolling turn */

/* Entry/exit speed for in-place turns (when rolling turn isn't possible,
 * e.g. dead end requiring 180°). */
#define INPLACE_TURN_SPEED_DPS  360.0f   /* degrees per second for in-place rotation */

/* ─── S-Curve Profile ──────────────────────────────────────────────── */
/*
 * S-curve adds a jerk limit on top of trapezoidal profile, giving
 * smoother acceleration transitions (reduces wheel slip and mechanical
 * shock). Set to 0 to disable S-curve (falls back to trapezoidal).
 *
 *   Trapezoidal:  speed jumps instantly to max accel
 *   S-curve:      acceleration itself ramps up/down smoothly
 *
 *   velocity
 *     │      ╭────────╮        S-curve (smooth)
 *     │     ╱          ╲
 *     │    ╱            ╲
 *     │   ╱              ╲
 *     └──╱────────────────╲── time
 *
 *     │    ┌──────────┐        Trapezoidal (sharp corners)
 *     │   ╱            ╲
 *     │  ╱              ╲
 *     └─╱────────────────╲── time
 */
#define JERK_LIMIT_MM_S3       8000.0f   /* mm/s³ — higher = snappier, lower = smoother */
#define ENABLE_S_CURVE         1         /* set to 0 for basic trapezoidal              */

/* ─── Turn Tolerance ───────────────────────────────────────────────── */
#define TURN_TOLERANCE_DEG     2.0f      /* acceptable heading error after a turn (°) */
#define CELL_ENTRY_OFFSET_MM   10.0f     /* distance past cell boundary before turning */

/* ─── Look-Ahead ───────────────────────────────────────────────────── */
/* How many commands ahead the motion controller examines to pre-plan
 * deceleration and smooth turn entry. More = smoother but more compute. */
#define LOOK_AHEAD_COMMANDS    3

#endif /* CONFIG_H */
