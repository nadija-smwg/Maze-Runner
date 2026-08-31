/**
 * @file mpu6050.cpp
 * @brief MPU6050 driver implementation with EMA filtering and drift correction.
 * @see mpu6050.h
 *
 * Filter constants (from 1.MPU6050.ino test sketch):
 *   ACC_ALPHA  = 0.90  → 90% old, 10% new (smooth accelerometer)
 *   GYRO_ALPHA = 0.80  → 80% old, 20% new (responsive gyro for turns)
 *
 * Stationary drift correction:
 *   When the robot is still and |gz_filtered| < 1.0 °/s, the raw gyro_z
 *   bias is slowly pulled toward the current raw reading at rate 0.0005.
 *   This corrects temperature-induced drift without affecting turns.
 */

#include "mpu6050.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  Filter Constants
 * ═══════════════════════════════════════════════════════════════════════════ */

/** EMA coefficient for accelerometer (0=max smooth, 1=no filter). */
#define ACC_ALPHA               0.90f

/** EMA coefficient for gyroscope. Keep 0.75–0.85 for fast turn response. */
#define GYRO_ALPHA              0.80f

/**
 * Gyro Z threshold below which drift correction is allowed (°/s).
 * Must be larger than the worst-case post-calibration thermal drift.
 * MPU6050 cheap chips can drift up to ±3 °/s as they warm up, so
 * set this above that. It must still be smaller than a real slow turn.
 *
 * 3.5 °/s → catches thermal drift after calibration; a real turn is
 * typically 50–500 °/s, so this will not trigger during motion.
 */
#define STATIONARY_GZ_THRESH    3.5f

/**
 * Rate at which gyro Z bias is corrected when stationary.
 *
 * At 1 kHz control loop:  0.002 → time constant ≈ 500 ms  (fast, good)
 * At  10 Hz Phase 3 test: 0.002 → time constant ≈ 50 s    (visible)
 *
 * Keep below 0.01 to avoid correcting during real turns.
 */
#define DRIFT_CORRECT_RATE      0.002f

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private State — Calibration Biases
 * ═══════════════════════════════════════════════════════════════════════════ */

static float _accel_bias_x = 0.0f;
static float _accel_bias_y = 0.0f;
static float _accel_bias_z = 0.0f;  /* Corrected for 1g at calibration */

static float _gyro_bias_x  = 0.0f;
static float _gyro_bias_y  = 0.0f;
static float _gyro_bias_z  = 0.0f;

static bool  _calibrated   = false;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private State — EMA Filter
 * ═══════════════════════════════════════════════════════════════════════════ */

static float _filt_ax = 0.0f;
static float _filt_ay = 0.0f;
static float _filt_az = 0.0f;

static float _filt_gx = 0.0f;
static float _filt_gy = 0.0f;
static float _filt_gz = 0.0f;

/** true after first valid mpu6050_update_filter() call */
static bool  _filter_initialized = false;

/** Set by mpu6050_set_stationary() — enables drift correction */
static bool  _stationary         = false;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private Helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

static void write_register(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialization
 * ═══════════════════════════════════════════════════════════════════════════ */

bool mpu6050_init(void)
{
    /* 1. Check WHO_AM_I register (0x75) */
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(0x75);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)MPU6050_I2C_ADDR, (uint8_t)1);
    if (!Wire.available())
        return false;

    uint8_t whoami = Wire.read();
    if (whoami != 0x68 && whoami != 0x71 && whoami != 0x73)
        return false;

    /* 2. Wake up (clear sleep bit in PWR_MGMT_1) */
    write_register(0x6B, 0x00);
    delay(100);

    /* 3. Sample rate = 1000 / (1+7) = 125 Hz */
    write_register(0x19, 0x07);

    /* 4. DLPF ≈ 44 Hz bandwidth */
    write_register(0x1A, 0x03);

    /* 5. Gyroscope ±500 °/s  → sensitivity 65.5 LSB/(°/s) */
    write_register(0x1B, 0x08);

    /* 6. Accelerometer ±2 g  → sensitivity 16384 LSB/g */
    write_register(0x1C, 0x00);

    delay(100); /* Let filters settle */

    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Raw Read
 * ═══════════════════════════════════════════════════════════════════════════ */

void mpu6050_read_raw(IMURawData *data)
{
    if (!data) return;

    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(0x3B); /* ACCEL_XOUT_H — start of 14-byte block */
    Wire.endTransmission();

    Wire.requestFrom((uint8_t)MPU6050_I2C_ADDR, (uint8_t)14);

    int available = Wire.available();
    if (available < 14)
    {
        Serial.print("[MPU6050] I2C read error: got ");
        Serial.print(available);
        Serial.println(" bytes (expected 14)");
        return;
    }

    uint8_t buf[14];
    for (int i = 0; i < 14; i++)
        buf[i] = Wire.read();

    data->accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    data->accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    data->accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
    data->temp    = (int16_t)((buf[6]  << 8) | buf[7]);
    data->gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
    data->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Scaled Read (bias-corrected, no filter — kept for backward compatibility)
 * ═══════════════════════════════════════════════════════════════════════════ */

void mpu6050_read_scaled(IMUScaledData *data)
{
    if (!data) return;

    IMURawData raw;
    mpu6050_read_raw(&raw);

    /* Accel: ±2g → 16384 LSB/g */
    data->accel_x_g = (float)(raw.accel_x - _accel_bias_x) / 16384.0f;
    data->accel_y_g = (float)(raw.accel_y - _accel_bias_y) / 16384.0f;
    data->accel_z_g = (float)(raw.accel_z - _accel_bias_z) / 16384.0f;

    /* Gyro: ±500 °/s → 65.5 LSB/(°/s)
     * NOTE: The +2.1f empirical correction has been removed.
     * Proper calibration with all hardware powered absorbs this offset. */
    data->gyro_x_dps = (float)(raw.gyro_x - _gyro_bias_x) / 65.5f;
    data->gyro_y_dps = (float)(raw.gyro_y - _gyro_bias_y) / 65.5f;
    data->gyro_z_dps = (float)(raw.gyro_z - _gyro_bias_z) / 65.5f;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Calibration
 * ═══════════════════════════════════════════════════════════════════════════ */

void mpu6050_calibrate_gyro(uint16_t samples)
{
    IMURawData raw;

    /* Discard first 50 readings — let DLPF settle */
    for (int i = 0; i < 50; i++)
    {
        mpu6050_read_raw(&raw);
        delay(2);
    }

    /* Accumulate using int64 to avoid overflow on 1000 × int16 values */
    int64_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int64_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    uint16_t valid = 0;

    for (uint16_t i = 0; i < samples; i++)
    {
        mpu6050_read_raw(&raw);

        ax_sum += raw.accel_x;
        ay_sum += raw.accel_y;
        az_sum += raw.accel_z;
        gx_sum += raw.gyro_x;
        gy_sum += raw.gyro_y;
        gz_sum += raw.gyro_z;

        valid++;
        delay(2);
    }

    if (valid == 0) return;

    /* Accelerometer bias — accel Z corrected for 1g (robot on flat surface) */
    _accel_bias_x = (float)ax_sum / valid;
    _accel_bias_y = (float)ay_sum / valid;
    _accel_bias_z = ((float)az_sum / valid) - 16384.0f;

    /* Gyro bias */
    _gyro_bias_x = (float)gx_sum / valid;
    _gyro_bias_y = (float)gy_sum / valid;
    _gyro_bias_z = (float)gz_sum / valid;

    _calibrated = true;

    /* Reset filter state so it re-seeds from the new biases */
    _filter_initialized = false;

    Serial.println("[MPU6050] Calibration complete:");
    Serial.print("  Accel Bias X: "); Serial.print(_accel_bias_x, 1);
    Serial.print("  Y: ");            Serial.print(_accel_bias_y, 1);
    Serial.print("  Z: ");            Serial.println(_accel_bias_z, 1);
    Serial.print("  Gyro  Bias X: "); Serial.print(_gyro_bias_x, 2);
    Serial.print("  Y: ");            Serial.print(_gyro_bias_y, 2);
    Serial.print("  Z: ");            Serial.println(_gyro_bias_z, 2);
}

bool mpu6050_is_calibrated(void)
{
    return _calibrated;
}

float mpu6050_get_gyro_bias_z(void)
{
    return _gyro_bias_z;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  EMA Filter Update
 * ═══════════════════════════════════════════════════════════════════════════ */

void mpu6050_update_filter(float dt)
{
    IMURawData raw;
    mpu6050_read_raw(&raw);

    /* Convert to physical units with calibration offsets */
    float ax = (float)(raw.accel_x - _accel_bias_x) / 16384.0f;
    float ay = (float)(raw.accel_y - _accel_bias_y) / 16384.0f;
    float az = (float)(raw.accel_z - _accel_bias_z) / 16384.0f;

    float gx = (float)(raw.gyro_x - _gyro_bias_x) / 65.5f;
    float gy = (float)(raw.gyro_y - _gyro_bias_y) / 65.5f;
    float gz = (float)(raw.gyro_z - _gyro_bias_z) / 65.5f;

    if (!_filter_initialized)
    {
        /* First call: seed EMA with first reading so we don't ramp from 0 */
        _filt_ax = ax;  _filt_ay = ay;  _filt_az = az;
        _filt_gx = gx;  _filt_gy = gy;  _filt_gz = gz;
        _filter_initialized = true;
        return;
    }

    /* EMA on accelerometer — matches test sketch alphaAcc = 0.95 pattern
     * but inverted convention: filtered = alpha*old + (1-alpha)*new
     * Note: ACC_ALPHA = 0.90 means 10% new data per step */
    _filt_ax = ACC_ALPHA * _filt_ax + (1.0f - ACC_ALPHA) * ax;
    _filt_ay = ACC_ALPHA * _filt_ay + (1.0f - ACC_ALPHA) * ay;
    _filt_az = ACC_ALPHA * _filt_az + (1.0f - ACC_ALPHA) * az;

    /* EMA on gyroscope — GYRO_ALPHA = 0.80 keeps fast turn response */
    _filt_gx = GYRO_ALPHA * _filt_gx + (1.0f - GYRO_ALPHA) * gx;
    _filt_gy = GYRO_ALPHA * _filt_gy + (1.0f - GYRO_ALPHA) * gy;
    _filt_gz = GYRO_ALPHA * _filt_gz + (1.0f - GYRO_ALPHA) * gz;

    /*
     * Stationary gyro drift correction.
     * Only runs when: robot is stationary AND filtered Gz is very small.
     * Slowly pulls _gyro_bias_z toward the current raw reading.
     * Does NOT run during turns — motion controller must call
     * mpu6050_set_stationary(false) whenever wheels are commanded.
     */
    if (_stationary && fabsf(_filt_gz) < STATIONARY_GZ_THRESH)
    {
        _gyro_bias_z = (1.0f - DRIFT_CORRECT_RATE) * _gyro_bias_z
                     + DRIFT_CORRECT_RATE           * (float)raw.gyro_z;
    }

    (void)dt; /* dt reserved for future use (e.g. Madgwick filter) */
}

void mpu6050_get_filtered(IMUScaledData *out)
{
    if (!out) return;

    out->accel_x_g   = _filt_ax;
    out->accel_y_g   = _filt_ay;
    out->accel_z_g   = _filt_az;
    out->gyro_x_dps  = _filt_gx;
    out->gyro_y_dps  = _filt_gy;
    out->gyro_z_dps  = _filt_gz;
}

void mpu6050_set_stationary(bool is_stationary)
{
    _stationary = is_stationary;
}
