/**
 * @file robot_state_machine.cpp
 * @brief Robot FSM — Phase 6/7 full implementation.
 *
 *  STATE_BOOT        → auto-transition to IDLE after setup()
 *  STATE_IDLE        → wait for BTN_START → SEARCH_RUN
 *                      (or BTN_MODE after return → FAST_RUN)
 *  STATE_SEARCH_RUN  → mission_manager_update() drives search loop
 *  STATE_RETURN_START→ mission_manager_update() drives return
 *  STATE_FAST_RUN    → mission_start_fast_run() + mission_manager_update()
 *  STATE_ERROR       → motor stop, LED blink
 */

#include "robot_state_machine.h"
#include "mission_manager.h"
#include "../hardware/button.h"
#include "../hardware/motor.h"
#include "../hardware/led.h"
#include "../utils/logger.h"

static RobotState _current_state = STATE_BOOT;

void fsm_init(void) {
    _current_state = STATE_BOOT;
}

void fsm_update(void) {
    switch (_current_state) {

    case STATE_BOOT:
        /* setup() already ran all inits. Transition immediately to IDLE. */
        fsm_set_state(STATE_IDLE);
        break;

    case STATE_IDLE:
        /*
         * BTN_START (short press) → begin search run
         * BTN_MODE  (short press) → begin fast run (only if path computed)
         */
        if (button_just_pressed(BUTTON_START)) {
            LOG_INFO("[FSM] BTN_START → STATE_SEARCH_RUN");
            fsm_set_state(STATE_SEARCH_RUN);
        }
        if (button_just_pressed(BUTTON_MODE)) {
            LOG_INFO("[FSM] BTN_MODE → STATE_FAST_RUN");
            mission_start_fast_run();
            fsm_set_state(STATE_FAST_RUN);
        }
        break;

    case STATE_SEARCH_RUN:
        /*
         * mission_manager_update() runs the search loop:
         *   SENSE → FLOOD → TURN → DRIVE → SENSE ...
         * It sets the state to STATE_RETURN_START when goal is reached.
         */
        mission_manager_update();

        /* Emergency stop: hold BTN_MODE → abort search */
        if (button_is_pressed(BUTTON_MODE)) {
            motor_stop();
            LOG_INFO("[FSM] Search aborted by BTN_MODE.");
            fsm_set_state(STATE_IDLE);
        }
        break;

    case STATE_RETURN_START:
        /*
         * Re-uses mission_manager_update() — the STEP_RETURN branch
         * uses reverse flood fill to navigate back to (0,0).
         * When back at start, mission_manager sets state to STATE_IDLE.
         */
        mission_manager_update();

        if (button_is_pressed(BUTTON_MODE)) {
            motor_stop();
            LOG_INFO("[FSM] Return aborted by BTN_MODE.");
            fsm_set_state(STATE_IDLE);
        }
        break;

    case STATE_FAST_RUN:
        /*
         * mission_manager_update() runs the fast run loop:
         *   solver_get_next_command() → motion_drive_cells / turn
         * When all commands consumed, sets STATE_IDLE.
         */
        mission_manager_update();

        /* Emergency stop */
        if (button_is_pressed(BUTTON_MODE)) {
            motor_stop();
            LOG_INFO("[FSM] Fast run aborted by BTN_MODE.");
            fsm_set_state(STATE_IDLE);
        }
        break;

    case STATE_ERROR:
        motor_stop();
        led_blink(LED_STATUS, 200);  /* slow blink = error */
        /* Press BTN_START to acknowledge and return to IDLE */
        if (button_just_pressed(BUTTON_START)) {
            fsm_set_state(STATE_IDLE);
        }
        break;

    case STATE_CALIBRATING:
        /* Handled in setup() — should never reach here in normal run */
        break;

    default:
        fsm_set_state(STATE_ERROR);
        break;
    }
}

RobotState fsm_get_state(void) {
    return _current_state;
}

void fsm_set_state(RobotState new_state) {
    if (new_state != _current_state) {
        const char *names[] = {
            "BOOT","IDLE","CALIBRATING","SEARCH_RUN",
            "RETURN_START","FAST_RUN","ERROR"
        };
        Serial.print(F("[FSM] "));
        Serial.print(names[(int)_current_state]);
        Serial.print(F(" → "));
        Serial.println(names[(int)new_state]);
    }
    _current_state = new_state;
}
