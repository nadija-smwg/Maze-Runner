/**
 * @file robot_state_machine.h
 * @brief Top-level robot finite state machine (FSM).
 *
 * Manages the high-level states: Boot, Idle, Calibrating, Searching,
 * Fast Run, Return to Start, Error.
 */

#ifndef ROBOT_STATE_MACHINE_H
#define ROBOT_STATE_MACHINE_H

/**
 * @brief Top-level robot states.
 */
typedef enum {
    STATE_BOOT,
    STATE_IDLE,
    STATE_CALIBRATING,
    STATE_SEARCH_RUN,
    STATE_RETURN_START,
    STATE_FAST_RUN,
    STATE_ERROR
} RobotState;

/**
 * @brief Initialize the FSM.
 */
void fsm_init(void);

/**
 * @brief Update the FSM logic. Call in main loop.
 */
void fsm_update(void);

/**
 * @brief Get current state.
 */
RobotState fsm_get_state(void);

/**
 * @brief Transition to a new state.
 */
void fsm_set_state(RobotState new_state);

#endif /* ROBOT_STATE_MACHINE_H */
