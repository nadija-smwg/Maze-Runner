/**
 * @file command_executor.cpp
 * @brief Command executor implementation.
 * @see command_executor.h
 */

#include "command_executor.h"

void executor_load_commands(const MotionCommand* commands, int count) {
    /** TODO: Store pointer and count */
}

void executor_step(void) {
    /** TODO: If current command is done, advance pointer and execute next */
}

bool executor_is_done(void) {
    /** TODO: Return true if all commands processed */
    return true;
}

void executor_stop(void) {
    /** TODO: Implementation */
}
