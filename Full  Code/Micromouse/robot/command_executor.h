/**
 * @file command_executor.h
 * @brief Executes sequences of MotionCommands.
 */

#ifndef COMMAND_EXECUTOR_H
#define COMMAND_EXECUTOR_H

#include <stdbool.h>
#include "../maze/maze.h"

/**
 * @brief Load a queue of commands.
 */
void executor_load_commands(const MotionCommand* commands, int count);

/**
 * @brief Update the executor.
 *
 * Dispatches the next command when the current one finishes.
 */
void executor_step(void);

/**
 * @brief Check if all commands are finished.
 */
bool executor_is_done(void);

/**
 * @brief Force stop execution.
 */
void executor_stop(void);

#endif /* COMMAND_EXECUTOR_H */
