/**
 * @file heading_controller.cpp
 * @brief Heading controller implementation.
 * @see heading_controller.h
 */

#include "heading_controller.h"
#include "../utils/math_utils.h"

/* TODO: Define heading PID object */

void heading_controller_init(void) {
    /** TODO: Initialize PID with tuned values */
}

float heading_controller_update(float target_heading_deg,
                                float current_heading_deg,
                                float dt) {
    /**
     * TODO:
     * 1. Calculate error = target - current.
     * 2. Wrap error to [-180, 180].
     * 3. Compute PID output.
     * 4. Return angular velocity correction.
     */
    return 0.0f;
}

void heading_controller_reset(void) {
    /** TODO: Reset PID state */
}
