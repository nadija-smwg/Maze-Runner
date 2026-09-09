/**
 * @file fast_run_mode.cpp
 * @brief Fast run mode — replays Dijkstra-smoothed path via command_executor.
 *
 * After the search run completes and robot is back at start:
 *   1. fast_run_mode_init() loads the smooth path from the solver into
 *      command_executor.
 *   2. fast_run_mode_update() calls executor_step() every loop() iteration.
 *      executor_step() dispatches the next MotionCommand when motion_is_idle().
 *   3. When executor_is_done(), mission_manager sets STATE_IDLE.
 *
 * Speed used during fast run = FAST_MAX_SPEED_MM_S from config.h.
 * Tune this value after the search run is stable.
 */

#include "fast_run_mode.h"
#include "command_executor.h"
#include "mission_manager.h"
#include "../maze/solver.h"
#include "../control/motion_controller.h"
#include "../hardware/motor.h"
#include <Arduino.h>

/* Extern reference to the global solver owned by mission_manager.cpp.
 * Declared here as extern so we can access the smooth path without
 * duplicating the struct. */
extern Solver g_solver;

/* Track fast run start time for competition_record_time() */
static uint32_t _fast_run_start_ms = 0;

void fast_run_mode_init(void)
{
    /* Load the Dijkstra smooth path into the command executor */
    if (g_solver.smooth_path.num_commands == 0) {
        Serial.println(F("[FastRun] ERROR: No path computed. Run search first."));
        return;
    }

    executor_load_commands(
        g_solver.smooth_path.commands,
        (int)g_solver.smooth_path.num_commands
    );

    _fast_run_start_ms = millis();

    Serial.print(F("[FastRun] Loaded "));
    Serial.print(g_solver.smooth_path.num_commands);
    Serial.println(F(" commands. Starting fast run."));
}

void fast_run_mode_update(void)
{
    if (executor_is_done()) {
        /* Fast run finished — record time and return to IDLE */
        uint32_t elapsed = millis() - _fast_run_start_ms;
        Serial.print(F("[FastRun] DONE! Time: "));
        Serial.print(elapsed);
        Serial.println(F(" ms"));
        motor_stop();
        /* mission_manager_update() will handle state transition to STATE_IDLE */
        return;
    }

    /* Dispatch next command when previous finishes (non-blocking) */
    executor_step();
}
