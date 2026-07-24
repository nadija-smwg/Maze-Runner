/**
 * @file sensor_manager.h
 * @brief Orchestrates initialization and updating of all sensors.
 *
 * Provides a single entry point in the main loop to update all sensor
 * data simultaneously (IMU, distance, battery, encoders).
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

/**
 * @brief Initialize all system sensors.
 *
 * Calls initialization for MPU6050, VL53L0X (via distance_manager),
 * Battery monitor, and Encoders.
 */
void sensor_manager_init(void);

/**
 * @brief Update all sensors.
 *
 * Should be called periodically. Reads IMU and Distance sensors.
 * (Encoders update automatically via hardware timers, but their delta
 * calculations can be triggered here or in the control loop).
 */
void sensor_manager_update(void);

/**
 * @brief Print all sensor readings for debugging.
 *
 * TODO: Output current sensor states over serial (if debug enabled).
 */
void sensor_manager_debug_print(void);

#endif /* SENSOR_MANAGER_H */
