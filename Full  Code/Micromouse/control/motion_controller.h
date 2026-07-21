/**
 * @file motion_controller.h
 * @brief Top-level controller coordinating all motion sub-controllers.
 *
 * This is the entry point for the 1kHz control loop. It reads sensors,
 * updates localization, and cascades down through trajectory, heading,
 * and speed controllers.
 */

#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <stdbool.h>
#include "../maze/maze.h"

/**
 * @brief Initialize the motion controller and all sub-controllers.
 */
void motion_controller_init(void);

/**
 * @brief The main 1kHz control loop update function.
 *
 * MUST be called exactly every 1ms.
 */
void motion_controller_update(void);

/**
 * @brief Execute a single motion command.
 *
 * @param cmd MotionCommand from the path smoother
 */
void motion_execute_command(const MotionCommand* cmd);

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

#endif /* MOTION_CONTROLLER_H */
