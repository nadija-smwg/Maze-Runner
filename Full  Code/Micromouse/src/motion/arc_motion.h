/**
 * @file arc_motion.h
 * @brief Circular arc motion execution interface.
 */

#ifndef ARC_MOTION_H
#define ARC_MOTION_H

#include <stdbool.h>

/**
 * @brief Start a circular arc move.
 *
 * @param radius_mm Turn radius
 * @param angle_deg Turn angle (degrees)
 * @param turn_speed Constant linear speed during arc
 * @param is_left true = left turn, false = right turn
 */
void arc_motion_start(float radius_mm, float angle_deg, float turn_speed, bool is_left);

/**
 * @brief Update the arc motion state.
 *
 * @param distance_traveled_mm Current distance along the arc
 */
void arc_motion_update(float distance_traveled_mm);

/**
 * @brief Check if arc is complete.
 */
bool arc_motion_is_complete(void);

#endif /* ARC_MOTION_H */
