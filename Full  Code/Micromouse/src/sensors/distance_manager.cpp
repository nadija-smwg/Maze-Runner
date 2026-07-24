/**
 * @file distance_manager.cpp
 * @brief Distance manager implementation.
 * @see distance_manager.h
 */

#include "distance_manager.h"
#include "vl53l0x.h"

/* TODO: Define array of VL53L0X_Sensor objects */
/* TODO: Store latest readings internally */

void distance_manager_init(void) {
    /**
     * TODO:
     * 1. Populate sensor array with XSHUT pins and I2C addresses.
     * 2. Call vl53l0x_init_all().
     */
}

void distance_manager_update(void) {
    /**
     * TODO: Loop through all initialized sensors and update readings
     * using vl53l0x_read_distance_mm().
     */
}

uint16_t distance_get_mm(DistanceSensorID id) {
    /** TODO: Return latest reading for the requested sensor ID */
    return 8190;
}

bool distance_has_wall_left(void) {
    /** TODO: Return (distance_get_mm(TOF_LEFT) < WALL_THRESHOLD_SIDE_MM) */
    return false;
}

bool distance_has_wall_right(void) {
    /** TODO: Return (distance_get_mm(TOF_RIGHT) < WALL_THRESHOLD_SIDE_MM) */
    return false;
}

bool distance_has_wall_front(void) {
    /** TODO: Evaluate front, front-left, and front-right sensors */
    return false;
}

float distance_get_centering_error(void) {
    /**
     * TODO:
     * 1. Check if both walls are present.
     * 2. If both: error = (left_dist - right_dist) / 2.
     * 3. If only left: error = target_left - left_dist.
     * 4. If only right: error = right_dist - target_right.
     * 5. Apply low-pass filter to error to prevent jitter.
     */
    return 0.0f;
}

float distance_get_front_alignment_error(void) {
    /**
     * TODO:
     * If front-left and front-right both see a wall close by,
     * calculate angular deviation based on their difference and separation distance.
     */
    return 0.0f;
}
