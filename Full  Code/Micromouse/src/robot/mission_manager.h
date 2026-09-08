/**
 * @file mission_manager.h
 * @brief High-level mission coordinator — Phase 6/7.
 *
 * Orchestrates: Search Run (flood fill exploration) → Return to Start
 *               → Fast Run (Dijkstra optimal path).
 */

#ifndef MISSION_MANAGER_H
#define MISSION_MANAGER_H

/** Initialize solver and mission state. Call once in setup(). */
void mission_manager_init(void);

/**
 * Non-blocking update — call every loop() iteration from fsm_update().
 * Drives the search-run state machine one step at a time.
 */
void mission_manager_update(void);

/**
 * Trigger the fast run. Call after search run is complete and robot
 * has returned to start. Uses the Dijkstra path computed at the goal.
 */
void mission_start_fast_run(void);

#endif /* MISSION_MANAGER_H */
