/**
 * @file mpu6050.cpp
 * @brief MPU6050 driver implementation.
 * @see mpu6050.h
 */

#include "mpu6050.h"
#include <Arduino.h>
#include <Wire.h>

/* Internal State */
static float _accel_bias_x = 0.0f;
static float _accel_bias_y = 0.0f;
static float _accel_bias_z = 0.0f;

static float _gyro_bias_x = 0.0f;
static float _gyro_bias_y = 0.0f;
static float _gyro_bias_z = 0.0f;

static bool _calibrated = false;

/* Helper functions for I2C communication */
static void write_register(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

bool mpu6050_init(void) {
    // 1. Check WHO_AM_I register (0x75)
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(0x75);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)MPU6050_I2C_ADDR, (uint8_t)1);
    if (!Wire.available()) {
        return false;
    }
    uint8_t whoami = Wire.read();
    if (whoami != 0x68 && whoami != 0x71 && whoami != 0x73) {
        // 0x68 is standard MPU6050, others are variants (like MPU9250)
        return false;
    }

    // 2. Wake up (PWR_MGMT_1)
    write_register(0x6B, 0x00);
    delay(10); // Wait for sensor to stabilize

    // 3. Sample Rate = 1000Hz (SMPLRT_DIV = 0)
    write_register(0x19, 0x00);

    // 4. DLPF = 44Hz (CONFIG = 3)
    write_register(0x1A, 0x03);

    // 5. Gyro config = ±500 deg/s (GYRO_CONFIG = 8)
    write_register(0x1B, 0x08);

    // 6. Accel config = ±2g (ACCEL_CONFIG = 0)
    write_register(0x1C, 0x00);

    delay(100); // Wait for registers to populate and filters to settle
    
    return true;
}

void mpu6050_read_raw(IMURawData *data) {
    if (!data) return;

    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(0x3B); // Starting register: ACCEL_XOUT_H
    Wire.endTransmission(false); // MUST use Repeated Start to prevent register tearing!
    
    Wire.requestFrom((uint8_t)MPU6050_I2C_ADDR, (uint8_t)14);

    int available = Wire.available();
    if (available < 14) {
        Serial.print("[MPU6050] I2C Read Error! Expected 14, got: ");
        Serial.println(available);
    }

    uint8_t buffer[14];
    for (int i = 0; i < 14; i++) {
        if (Wire.available()) {
            buffer[i] = Wire.read();
        } else {
            buffer[i] = 0;
        }
    }

    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    data->temp    = (int16_t)((buffer[6] << 8) | buffer[7]);
    
    data->gyro_x  = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y  = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z  = (int16_t)((buffer[12] << 8) | buffer[13]);
}

void mpu6050_read_scaled(IMUScaledData *data) {
    if (!data) return;

    IMURawData raw;
    mpu6050_read_raw(&raw);

    // Scale factors based on configurations:
    // Accel: ±2g -> 16384 LSB/g
    // Gyro: ±500 deg/s -> 65.5 LSB/(deg/s)

    data->accel_x_g = (float)(raw.accel_x - _accel_bias_x) / 16384.0f;
    data->accel_y_g = (float)(raw.accel_y - _accel_bias_y) / 16384.0f;
    data->accel_z_g = (float)(raw.accel_z - _accel_bias_z) / 16384.0f;

    data->gyro_x_dps = (float)(raw.gyro_x - _gyro_bias_x) / 65.5f;
    data->gyro_y_dps = (float)(raw.gyro_y - _gyro_bias_y) / 65.5f;
    
    // CUSTOM HARDWARE CALIBRATION:
    // 1. The VL53L0X lasers pull current on the 3.3V line, creating a constant 
    //    hardware noise shift on the MPU6050 gyro. We apply an empirical +2.1 deg/s 
    //    correction here to cancel it out so the robot tracks perfectly straight.
    float base_gz = ((float)(raw.gyro_z - _gyro_bias_z) / 65.5f) + 2.1f;
    
    // 2. A 90-degree physical turn was reading as ~172.5 degrees.
    //    We apply a multiplier of (90.0 / 172.5) = 0.5217 to scale the turns perfectly!
    float raw_gz = base_gz * 0.5217f;
    
    // EMA Low-Pass Filter (matches test code alpha=0.85)
    // Smooths out electrical noise spikes from ToF sensors and motors
    static float gz_filtered = 0.0f;
    gz_filtered = 0.85f * gz_filtered + 0.15f * raw_gz;
    data->gyro_z_dps = gz_filtered;
}

void mpu6050_calibrate_gyro(uint16_t samples) {
    long ax_sum = 0, ay_sum = 0, az_sum = 0;
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;

    IMURawData raw;
    
    // Discard first few readings to let filter settle
    for (int i = 0; i < 50; i++) {
        mpu6050_read_raw(&raw);
        delay(2);
    }

    for (uint16_t i = 0; i < samples; i++) {
        mpu6050_read_raw(&raw);

        ax_sum += raw.accel_x;
        ay_sum += raw.accel_y;
        az_sum += raw.accel_z;

        gx_sum += raw.gyro_x;
        gy_sum += raw.gyro_y;
        gz_sum += raw.gyro_z;

        delay(2);
    }

    _accel_bias_x = (float)ax_sum / samples;
    _accel_bias_y = (float)ay_sum / samples;
    _accel_bias_z = ((float)az_sum / samples) - 16384.0f; // Subtract 1g

    _gyro_bias_x = (float)gx_sum / samples;
    _gyro_bias_y = (float)gy_sum / samples;
    _gyro_bias_z = (float)gz_sum / samples;

    _calibrated = true;

    Serial.print("[MPU6050] Calibrated! gz_sum: ");
    Serial.print(gz_sum);
    Serial.print(" | _gyro_bias_z: ");
    Serial.println(_gyro_bias_z);
}

bool mpu6050_is_calibrated(void) {
    return _calibrated;
}

float mpu6050_get_gyro_bias_z(void) {
    return _gyro_bias_z;
}
