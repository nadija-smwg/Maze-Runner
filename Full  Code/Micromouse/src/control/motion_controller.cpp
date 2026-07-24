/**
 * @file motion_controller.cpp
 * @brief Top-level motion controller implementation.
 * @see motion_controller.h
 */

#include "motion_controller.h"
#include "cell_controller.h"
#include "turn_controller.h"
#include "speed_controller.h"
#include "heading_controller.h"
#include "wall_follower.h"
#include "velocity_controller.h"
#include "trajectory_controller.h"
#include "../sensors/sensor_fusion.h"
#include "../hardware/motor.h"

void motion_controller_init(void) {
    /**
     * TODO: Initialize all sub-controllers.
     */
}

void motion_controller_update(void) {
    /**
     * TODO:
     * 1. Check if idle, if so just hold position or coast.
     * 2. fusion_update(dt).
     * 3. trajectory_controller_update(dt) -> gets target v and target heading.
     * 4. heading_controller_update() -> gets angular velocity ω.
     * 5. wall_follower_update() -> adjust ω if applicable.
     * 6. velocity_controller_update(v, ω) -> gets target wheel speeds.
     * 7. speed_controller_update() -> sets PWMs.
     */
}

void motion_execute_command(const MotionCommand* cmd) {
    /**
     * TODO:
     * Switch on cmd->type and delegate to cell_controller or turn_controller.
     */
}

bool motion_is_idle(void) {
    /** TODO: Return whether all sub-controllers are idle. */
    return true;
}

void motion_emergency_stop(void) {
    /** TODO: Stop motors and reset all active controller states. */
    motor_stop();
}
