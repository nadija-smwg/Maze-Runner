/**
 * @file calibration.h
 * @brief Routines for calibrating various sensors.
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdbool.h>

/**
 * @brief Run full startup calibration sequence.
 *
 * @return true if successful
 */
bool calibrate_all(void);

/**
 * @brief Calibrate IMU gyro zero-offsets.
 */
void calibrate_gyro(void);

/**
 * @brief Calibrate ToF distance sensors against walls.
 *
 * Placed in a known position in a cell, measure and store offsets
 * for the distance sensors.
 */
void calibrate_distance_sensors(void);

/**
 * @brief Tune encoder wheel size offsets (advanced).
 */
void calibrate_encoders(void);

#endif /* CALIBRATION_H */
