/**
 * @file cell_controller.cpp
 * @brief Cell controller implementation.
 * @see cell_controller.h
 */

#include "cell_controller.h"

void cell_start_move(int num_cells) {
    /**
     * TODO:
     * 1. Calculate distance: num_cells * CELL_SIZE_MM.
     * 2. Configure straight_motion module with distance and motion profile limits.
     * 3. Start straight_motion.
     */
}

bool cell_is_complete(void) {
    /**
     * TODO: Return straight_motion_is_complete().
     */
    return true;
}

float cell_get_progress(void) {
    /**
     * TODO: Return current distance / total distance.
     */
    return 1.0f;
}

void cell_stop(void) {
    /**
     * TODO: Call straight_motion_stop().
     */
}
