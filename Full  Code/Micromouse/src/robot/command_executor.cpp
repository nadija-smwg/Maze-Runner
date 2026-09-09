/**
 * @file command_executor.cpp
 * @brief Command executor — dispatches MotionCommand sequences to
 *        motion_controller one command at a time, non-blocking.
 *
 * Used by fast_run_mode to replay the Dijkstra-smoothed path.
 * Also used by motion_execute_command() in motion_controller.cpp.
 *
 * Call executor_step() every loop() iteration.
 * Poll executor_is_done() to detect sequence completion.
 */

#include "command_executor.h"
#include "../control/motion_controller.h"
#include "../hardware/motor.h"

/* ─── Private state ──────────────────────────────────────────────────── */
static const MotionCommand *_cmds      = (const MotionCommand*)0;
static int                  _count     = 0;
static int                  _index     = 0;
static bool                 _running   = false;

/* ─── Public API ─────────────────────────────────────────────────────── */

void executor_load_commands(const MotionCommand *commands, int count)
{
    _cmds    = commands;
    _count   = count;
    _index   = 0;
    _running = (commands != (const MotionCommand*)0 && count > 0);
}

void executor_step(void)
{
    if (!_running) return;
    if (_index >= _count) {
        _running = false;
        return;
    }

    /* Wait for any in-progress move to finish */
    if (!motion_is_idle()) return;

    /* Dispatch the current command */
    const MotionCommand *cmd = &_cmds[_index];
    switch (cmd->type) {
        case CMD_FORWARD:
            motion_drive_cells(cmd->count);
            break;
        case CMD_TURN_RIGHT:
            motion_turn_right_90();
            break;
        case CMD_TURN_LEFT:
            motion_turn_left_90();
            break;
        default:
            break;
    }

    /* Advance to next command — completion is detected next executor_step()
     * call when motion_is_idle() returns true again.                      */
    _index++;
}

bool executor_is_done(void)
{
    return !_running || _index >= _count;
}

void executor_stop(void)
{
    _running = false;
    _index   = _count;  /* skip remaining commands */
    motor_stop();
}
