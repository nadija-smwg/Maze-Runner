/**
 * @file distance_manager.h
 * @brief High-level management of all 5 ToF distance sensors.
 *
 * Converts raw millimeter distances from VL53L0X sensors into boolean
 * wall presence flags and centering error values for the wall follower.
 */

#ifndef DISTANCE_MANAGER_H
#define DISTANCE_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief ToF sensor identifiers.
 */
typedef enum {
    TOF_FRONT       = 0,
    TOF_FRONT_LEFT  = 1,
    TOF_FRONT_RIGHT = 2,
    TOF_LEFT        = 3,
    TOF_RIGHT       = 4,
    TOF_COUNT       = 5
} DistanceSensorID;

/**
 * @brief Wall presence thresholds (mm).
 * TODO: Tune these values based on actual sensor placement and maze size.
 */
#define WALL_THRESHOLD_SIDE_MM      120.0f
#define WALL_THRESHOLD_FRONT_MM     150.0f

/**
 * @brief Initialize all distance sensors.
 *
 * Calls vl53l0x_init_all() internally.
 */
void distance_manager_init(void);

/**
 * @brief Update distance readings.
 *
 * Should be called periodically to read new values from all sensors.
 */
void distance_manager_update(void);

/**
 * @brief Get the raw distance from a specific sensor.
 *
 * @param id Sensor identifier
 * @return   Distance in mm
 */
uint16_t distance_get_mm(DistanceSensorID id);

/**
 * @brief Check if a wall is present to the left.
 */
bool distance_has_wall_left(void);

/**
 * @brief Check if a wall is present to the right.
 */
bool distance_has_wall_right(void);

/**
 * @brief Check if a wall is present in front.
 *
 * Fuses reading from front, front-left, and front-right if necessary.
 */
bool distance_has_wall_front(void);

/**
 * @brief Calculate the lateral centering error for wall following.
 *
 * Uses left and right distance sensors to find deviation from center.
 *
 * @return Error in mm (positive = too far right, negative = too far left).
 *         Returns 0 if no walls or if centering is unreliable.
 */
float distance_get_centering_error(void);

/**
 * @brief Calculate front alignment error.
 *
 * Uses front-left and front-right sensors when facing a wall to square up.
 *
 * @return Angle error in degrees.
 */
float distance_get_front_alignment_error(void);

#endif /* DISTANCE_MANAGER_H */
