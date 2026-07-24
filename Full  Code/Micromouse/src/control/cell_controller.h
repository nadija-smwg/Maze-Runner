/**
 * @file cell_controller.h
 * @brief Manages cell-to-cell movement execution.
 *
 * Drives the robot forward by exactly one cell distance (180mm),
 * using the motion profile generator and trajectory tracking.
 */

#ifndef CELL_CONTROLLER_H
#define CELL_CONTROLLER_H

#include <stdbool.h>

/**
 * @brief Start moving forward by a given number of cells.
 *
 * @param num_cells Number of cells to move forward.
 */
void cell_start_move(int num_cells);

/**
 * @brief Check if the current cell move is complete.
 *
 * @return true if complete
 */
bool cell_is_complete(void);

/**
 * @brief Get the progress of the current move.
 *
 * @return Progress from 0.0 to 1.0
 */
float cell_get_progress(void);

/**
 * @brief Force stop the current cell move.
 */
void cell_stop(void);

#endif /* CELL_CONTROLLER_H */
