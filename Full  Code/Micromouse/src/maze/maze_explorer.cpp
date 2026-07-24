/**
 * @file maze_explorer.cpp
 * @brief Maze explorer implementation.
 * @see maze_explorer.h
 */

#include "maze_explorer.h"

void explorer_init(void) {
    /** TODO: Implementation */
}

bool explorer_should_return(const Solver *current_solver_state) {
    /**
     * TODO:
     * Logic: If we reached the goal, we can return.
     * Advanced: Continue exploring unvisited cells that might offer a shorter path.
     */
    return solver_at_goal(current_solver_state);
}

bool explorer_is_complete(const Solver *current_solver_state) {
    /** TODO: Implementation */
    return false;
}
