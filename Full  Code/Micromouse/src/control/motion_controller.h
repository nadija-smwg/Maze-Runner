/**
 * @file motion_controller.h
 * @brief Top-level controller coordinating all motion sub-controllers.
 *
 * This is the entry point for the 1kHz control loop. It reads sensors,
 * updates localization, and cascades down through trajectory, heading,
 * and speed controllers.
 *
 * Phase 5 additions:
 *   - motion_drive_cell()     : drive exactly one maze cell (180mm) with
 *                               trapezoidal profile + heading correction
 *   - motion_is_cell_moving() : poll from loop() to detect completion
 */

#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "../maze/maze.h"
#include <stdbool.h>


/**
 * @brief Initialize the motion controller and all sub-controllers.
 */
void motion_controller_init(void);

/**
 * @brief The main 1kHz control loop update function.
 *
 * MUST be called exactly every 1ms from the hardware timer ISR.
 *
 * Handles:
 *   1. Encoder deltas → odometry → heading fusion
 *   2. Trapezoidal motion profile update
 *   3. Heading correction (Kp = 2.0)
 *   4. velocity_controller_update(v, w) → speed controller → motors
 *   5. Cell completion detection + auto-stop
 */
void motion_controller_update(void);

/**
 * @brief Drive a specified number of cells straight.
 *
 * @param count Number of cells to drive (e.g. 5 for test 5.6)
 */
void motion_drive_cells(int count);

/**
 * @brief Execute a single motion command.
 *
 * @param cmd MotionCommand from the path smoother
 */
void motion_execute_command(const MotionCommand *cmd);

/**
 * @brief Check if the motion controller is currently idle.
 *
 * @return true if no active move or turn is in progress
 */
bool motion_is_idle(void);

/**
 * @brief Emergency stop all motion.
 */
void motion_emergency_stop(void);

/* ── Phase 5.3 — Cell Drive Primitive ─────────────────────────────────────── */

/**
 * @brief Drive one maze cell (180mm) from a standstill to a standstill.
 *
 * Resets local odometry and heading to zero, then starts a trapezoidal
 * profile (0 → SEARCH_MAX_SPEED_MM_S → 0 over CELL_SIZE_MM = 180mm).
 * Heading correction (Kp = 2.0) keeps the robot driving straight.
 *
 * Call from loop() or a command handler ONLY (not from ISR).
 * motion_controller_update() drives the ISR side autonomously.
 */
void motion_drive_cell(void);

/**
 * @brief Returns true while a cell drive is in progress.
 *
 * Poll this from loop() to detect when the robot has finished driving
 * one cell and is ready for the next command.
 *
 * @return true = still driving, false = stopped / idle
 */
bool motion_is_cell_moving(void);

/* ── Phase 5.4 — 90° Turn Primitives ──────────────────────────────────────── */

/**
 * @brief Spin right (clockwise) exactly 90° using closed-loop heading control.
 *
 * Uses the fused gyro+encoder heading as feedback. Runs entirely in the
 * 1kHz ISR — no blocking delay. Poll motion_is_turning() to detect completion.
 *
 * Tuning: KP_TURN, MAX_TURN_RAD_S, TURN_DONE_RAD in robot_config.h.
 */
void motion_turn_right_90(void);

/**
 * @brief Spin left (counter-clockwise) exactly 90° using closed-loop heading control.
 *
 * Mirror of motion_turn_right_90(). Same tuning constants apply.
 */
void motion_turn_left_90(void);

/**
 * @brief Returns true while an in-place turn is in progress.
 *
 * Poll this from loop() to detect when the robot has finished turning
 * and is ready for the next command (e.g., motion_drive_cell).
 *
 * @return true = still turning, false = stopped / idle
 */
bool motion_is_turning(void);

#endif /* MOTION_CONTROLLER_H */
