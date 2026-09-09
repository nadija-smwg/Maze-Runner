/**
 * @file search_mode.cpp
 * @brief Search mode — thin delegate to mission_manager.
 *
 * The full search run logic (SENSE → FLOOD → TURN → DRIVE state machine,
 * all 5-sensor wall detection, flood fill, and solver_advance) lives in
 * mission_manager.cpp. This module is the named entry point called by
 * any legacy code that references search_mode_init / search_mode_update.
 */

#include "search_mode.h"
#include "mission_manager.h"

void search_mode_init(void)
{
    /* mission_manager_init() already called in setup() — nothing extra needed */
}

void search_mode_update(void)
{
    /* Delegate entirely to mission_manager which runs the full
     * SENSE → FLOOD → TURN → DRIVE state machine each loop() call. */
    mission_manager_update();
}
