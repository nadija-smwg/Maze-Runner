/**
 * @file mpu6050.h
 * @brief I2C driver for the MPU6050 6-axis IMU.
 *
 * Provides raw data reading, full 6-axis calibration, EMA filtering on all
 * axes, yaw integration support, and stationary gyro-drift correction.
 *
 * For a Micromouse, the primary output is the filtered Gz (deg/s) used by
 * sensor_fusion.cpp to integrate robot heading. Accelerometer is available
 * for tilt detection if needed.
 *
 * Filtering architecture:
 *
 *   mpu6050_read_raw()
 *         │
 *   Apply bias offsets (calibrated)
 *         │
 *   Convert to physical units (g, deg/s)
 *         │
 *   EMA on accel (α = ACC_ALPHA  = 0.90)
 *   EMA on gyro  (α = GYRO_ALPHA = 0.80)
 *         │
 *   mpu6050_get_filtered()
 *         │
 *   [optional] stationary drift correction on Gz bias
 *
 * Dependencies: pin_config (for I2C address MPU6050_I2C_ADDR)
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "../config/pin_config.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Data Structures (unchanged — no breaking changes to existing callers)
 * ═══════════════════════════════════════════════════════════════════════════ */

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

/* ═══════════════════════════════════════════════════════════════════════════
 *  Existing API  (all existing callers unchanged)
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialize the MPU6050 over I2C.
 *
 * Checks WHO_AM_I, wakes up device, sets:
 *   Sample rate = 125 Hz (SMPLRT_DIV = 7)
 *   DLPF        = ~44 Hz  (CONFIG = 3)
 *   Gyro range  = ±500 °/s (65.5 LSB/°/s)
 *   Accel range = ±2 g     (16384 LSB/g)
 *
 * @return true if initialization successful, false if sensor not found
 */
bool mpu6050_init(void);

/**
 * @brief Read raw data from the MPU6050 registers.
 *
 * Burst-reads 14 bytes from register 0x3B (ACCEL_XOUT_H) over I2C.
 *
 * @param data Pointer to IMURawData struct to populate
 */
void mpu6050_read_raw(IMURawData *data);

/**
 * @brief Read bias-corrected, scaled data (g's and deg/s).
 *
 * Applies calibration offsets and converts to physical units.
 * Does NOT apply EMA filtering — use mpu6050_get_filtered() for filtered values.
 *
 * @param data Pointer to IMUScaledData struct to populate
 */
void mpu6050_read_scaled(IMUScaledData *data);

/**
 * @brief Calibrate all 6 axes by averaging samples while stationary.
 *
 * Discards 50 warmup samples, then averages `samples` readings.
 * Stores accel bias (X,Y,Z) and gyro bias (X,Y,Z) internally.
 * Accel Z bias is corrected for 1g (assumes flat surface).
 *
 * @param samples Number of samples to average (recommended: 1000)
 */
void mpu6050_calibrate_gyro(uint16_t samples);

/**
 * @brief Check if calibration is complete.
 * @return true if calibrated
 */
bool mpu6050_is_calibrated(void);

/**
 * @brief Get the currently calibrated gyro Z bias (raw LSB units).
 * @return Gyro Z bias in raw LSB
 */
float mpu6050_get_gyro_bias_z(void);

/* ═══════════════════════════════════════════════════════════════════════════
 *  New API — EMA Filter + Stationary Drift Correction
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Read sensor, apply bias, convert, and run EMA filter on all 6 axes.
 *
 * Must be called at a fixed rate (e.g. every 8 ms → 125 Hz matches hardware).
 * Internally reads raw data, applies calibration offsets, converts to physical
 * units, and updates 6 EMA accumulators:
 *
 *   accel EMA: filtered = 0.90 * filtered + 0.10 * new
 *   gyro  EMA: filtered = 0.80 * filtered + 0.20 * new
 *
 * If mpu6050_set_stationary(true) was called and |filtered_gz| < threshold,
 * the gyro Z bias is slowly corrected to reduce drift.
 *
 * @param dt  Time step in seconds since last call (used for drift correction)
 */
void mpu6050_update_filter(float dt);

/**
 * @brief Get the latest EMA-filtered sensor values.
 *
 * Returns the result of the most recent mpu6050_update_filter() call.
 * Call mpu6050_update_filter() before calling this.
 *
 * @param out Pointer to IMUScaledData struct to fill with filtered values
 */
void mpu6050_get_filtered(IMUScaledData *out);

/**
 * @brief Hint from motion controller: robot is stationary (wheels not moving).
 *
 * When true, mpu6050_update_filter() will slowly nudge the gyro Z bias
 * toward the current filtered Gz reading to compensate for drift.
 * Only corrects if |filtered_gz| < STATIONARY_GZ_THRESH_DPS (1.0 °/s).
 *
 * @param is_stationary true = robot is not moving, false = robot is moving
 */
void mpu6050_set_stationary(bool is_stationary);

#endif /* MPU6050_H */
