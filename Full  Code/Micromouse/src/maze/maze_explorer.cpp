/**
 * @file maze_explorer.cpp
 * @brief Maze explorer implementation.
 * @see maze_explorer.h
 */

#include "maze_explorer.h"

void explorer_init(void) {
    /* Nothing specific to init for basic exploration; solver manages its state */
}

bool explorer_should_return(const Solver *current_solver_state) {
    /* Basic strategy: return immediately once the goal is found.
     * Advanced strategy: continue exploring unvisited cells that might offer a shorter path. */
    return solver_at_goal(current_solver_state);
}

bool explorer_is_complete(const Solver *current_solver_state) {
    /* Basic strategy: if we reached the goal, we consider exploration 'complete' enough for a fast run. */
    return solver_at_goal(current_solver_state);
}
