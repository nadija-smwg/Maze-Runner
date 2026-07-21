/**
 * @file robot_state_machine.cpp
 * @brief Robot FSM implementation.
 * @see robot_state_machine.h
 */

#include "robot_state_machine.h"

static RobotState _current_state = STATE_BOOT;

void fsm_init(void) {
    _current_state = STATE_BOOT;
}

void fsm_update(void) {
    /**
     * TODO: Switch statement for _current_state.
     * Call appropriate mode update function based on state.
     */
    switch (_current_state) {
        case STATE_BOOT:
            break;
        case STATE_IDLE:
            break;
        // ...
        default:
            break;
    }
}

RobotState fsm_get_state(void) {
    return _current_state;
}

void fsm_set_state(RobotState new_state) {
    _current_state = new_state;
}
