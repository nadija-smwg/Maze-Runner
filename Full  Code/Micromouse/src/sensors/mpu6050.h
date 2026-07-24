/**
 * @file mpu6050.h
 * @brief I2C driver for the MPU6050 6-axis IMU.
 *
 * Provides raw data reading and offset calibration for the MPU6050.
 * In a Micromouse, only the Z-axis gyro is strictly necessary for
 * heading tracking, but this provides access to all axes.
 *
 * Dependencies: pin_config (for I2C address)
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "../config/pin_config.h"

/**
 * @brief Structure containing raw IMU readings.
 */
typedef struct {
    int16_t accel_x;    /**< Raw X-axis accelerometer data */
    int16_t accel_y;    /**< Raw Y-axis accelerometer data */
    int16_t accel_z;    /**< Raw Z-axis accelerometer data */
    int16_t gyro_x;     /**< Raw X-axis gyroscope data     */
    int16_t gyro_y;     /**< Raw Y-axis gyroscope data     */
    int16_t gyro_z;     /**< Raw Z-axis gyroscope data     */
    int16_t temp;       /**< Raw temperature data          */
} IMURawData;

/**
 * @brief Structure containing scaled IMU readings.
 */
typedef struct {
    float accel_x_g;    /**< X-axis acceleration in g's    */
    float accel_y_g;    /**< Y-axis acceleration in g's    */
    float accel_z_g;    /**< Z-axis acceleration in g's    */
    float gyro_x_dps;   /**< X-axis rotation in deg/sec    */
    float gyro_y_dps;   /**< Y-axis rotation in deg/sec    */
    float gyro_z_dps;   /**< Z-axis rotation in deg/sec    */
} IMUScaledData;

/**
 * @brief Initialize the MPU6050 over I2C.
 *
 * Wakes up the sensor, sets ranges (e.g., ±1000 deg/s for gyro,
 * ±2g for accel), and configures the low-pass filter.
 *
 * @return true if initialization successful, false if sensor not found
 *
 * TODO: Implement I2C communication (Wire.beginTransmission, etc.)
 */
bool mpu6050_init(void);

/**
 * @brief Read raw data from the MPU6050 registers.
 *
 * @param data Pointer to IMURawData struct to populate
 *
 * TODO: Implement burst read of 14 bytes starting at register 0x3B.
 */
void mpu6050_read_raw(IMURawData *data);

/**
 * @brief Read scaled data (in g's and deg/s).
 *
 * Includes zero-offset subtraction for the gyro.
 *
 * @param data Pointer to IMUScaledData struct to populate
 *
 * TODO: Implement scaling and offset application.
 */
void mpu6050_read_scaled(IMUScaledData *data);

/**
 * @brief Calibrate the gyro by calculating the zero-rate offsets.
 *
 * Should be called when the robot is completely stationary.
 * Takes multiple samples and averages them.
 *
 * @param samples Number of samples to take (e.g., 500)
 *
 * TODO: Implement sampling loop and store offsets internally.
 */
void mpu6050_calibrate_gyro(uint16_t samples);

/**
 * @brief Check if calibration is complete.
 *
 * @return true if calibrated
 */
bool mpu6050_is_calibrated(void);

#endif /* MPU6050_H */
