/**
 * @file distance_manager.h
 * @brief High-level management of all 5 ToF distance sensors.
 *
 * Converts raw millimeter distances from VL53L0X sensors through a
 * multi-stage filtering pipeline (validity → calibration → Median-3 →
 * jump rejection → EMA) into stable filtered distances, boolean wall
 * presence flags with hysteresis, and PID error values.
 *
 * Filtering is handled by tof_filter.h / tof_filter.cpp — each sensor
 * has its own independent ToF_FilterState.
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

/* ═══════════════════════════════════════════════════════════════════════════
 *  Wall Detection Thresholds — Hysteresis Bands
 *
 *  Two-threshold hysteresis prevents flickering at the wall boundary:
 *
 *    NO_WALL ──── distance drops below ENTER ───► WALL state
 *    WALL    ──── distance rises above EXIT  ───► NO_WALL state
 *
 *  Adjust based on actual sensor mounting position and maze cell geometry.
 *  Current values assume a standard 180 mm cell with ~90 mm target distance.
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @defgroup WallThresholds Wall Detection Hysteresis
 *  TODO: Tune these based on sensor placement and maze geometry.
 *  @{
 */

/** Side sensor: distance below this → wall detected (enter WALL state) */
#define WALL_ENTER_SIDE_MM      110.0f

/** Side sensor: distance above this → wall cleared (enter NO_WALL state) */
#define WALL_EXIT_SIDE_MM       130.0f

/** Front sensor: distance below this → front wall detected */
#define WALL_ENTER_FRONT_MM     140.0f

/** Front sensor: distance above this → front wall cleared */
#define WALL_EXIT_FRONT_MM      160.0f

/** @} */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialize all distance sensors and their filter states.
 *
 * Calls vl53l0x_init_all() for hardware setup, then initialises one
 * ToF_FilterState per sensor with default offset = 0.
 */
void distance_manager_init(void);

/**
 * @brief Update all sensor readings and run the filter pipeline.
 *
 * Reads raw distances from all 5 sensors and passes each through
 * tof_filter_process(). Should be called at a fixed rate (e.g. 100 Hz).
 * Also updates hysteresis wall-state flags.
 */
void distance_manager_update(void);

/**
 * @brief Get the filtered distance from a specific sensor.
 *
 * Returns the EMA-smoothed, calibrated, median-filtered output.
 * Equivalent to distance_get_mm() — provided as an explicit alias.
 *
 * @param id Sensor identifier
 * @return   Filtered distance in mm (0–500), or last good value if invalid
 */
uint16_t distance_get_filtered_mm(DistanceSensorID id);

/**
 * @brief Get the filtered distance from a specific sensor.
 *
 * Kept for backward compatibility with existing callers (Phase 3 test mode,
 * sensor_manager_debug_print, etc.). Returns the same value as
 * distance_get_filtered_mm().
 *
 * @param id Sensor identifier
 * @return   Filtered distance in mm
 */
uint16_t distance_get_mm(DistanceSensorID id);

/**
 * @brief Check if a wall is present to the left (hysteresis).
 *
 * Uses two-threshold hysteresis to prevent flickering at the boundary.
 * State changes: NO_WALL→WALL when dist < WALL_ENTER_SIDE_MM,
 *                WALL→NO_WALL when dist > WALL_EXIT_SIDE_MM.
 */
bool distance_has_wall_left(void);

/**
 * @brief Check if a wall is present to the right (hysteresis).
 */
bool distance_has_wall_right(void);

/**
 * @brief Check if a wall is present in front (hysteresis).
 *
 * Uses the TOF_FRONT sensor with front-specific thresholds.
 */
bool distance_has_wall_front(void);

/**
 * @brief Calculate the lateral centering error for wall following.
 *
 * Uses filtered left and right distances to find deviation from center.
 *
 * @return Error in mm (positive = robot shifted left, need to correct right).
 *         Returns 0 if no walls or centering is unreliable.
 */
float distance_get_centering_error(void);

/**
 * @brief Calculate front alignment error for wall squaring.
 *
 * Uses front-left and front-right sensors when facing a wall.
 *
 * @return Angle error in mm difference (positive = angled right).
 *         Returns 0 if front sensors don't both see a wall.
 */
float distance_get_front_alignment_error(void);

/**
 * @brief Set the calibration offset for a specific sensor.
 *
 * Passes the offset to tof_filter_set_offset().
 * offset > 0: sensor reads low (add to compensate).
 * offset < 0: sensor reads high (subtract to compensate).
 *
 * @param id     Sensor identifier
 * @param offset Offset in mm
 */
void distance_set_sensor_offset(DistanceSensorID id, int16_t offset);

#endif /* DISTANCE_MANAGER_H */
