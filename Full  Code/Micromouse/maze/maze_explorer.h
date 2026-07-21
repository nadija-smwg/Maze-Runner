/**
 * @file maze_explorer.h
 * @brief Exploration strategy management.
 *
 * Decides when to switch from exploring the maze to returning to the start,
 * based on whether the goal has been found and sufficient paths explored.
 */

#ifndef MAZE_EXPLORER_H
#define MAZE_EXPLORER_H

#include <stdbool.h>
#include "solver.h"

/**
 * @brief Initialize the explorer.
 */
void explorer_init(void);

/**
 * @brief Evaluate the current state of exploration.
 *
 * @param current_solver_state Pointer to the active solver state
 * @return true if the robot should stop searching and return to start
 */
bool explorer_should_return(const Solver *current_solver_state);

/**
 * @brief Check if exploration is considered 100% complete.
 *
 * @param current_solver_state Pointer to the active solver state
 * @return true if no more optimal paths can possibly be found
 */
bool explorer_is_complete(const Solver *current_solver_state);

#endif /* MAZE_EXPLORER_H */
