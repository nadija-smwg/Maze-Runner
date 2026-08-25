/**
 * @file distance_manager.cpp
 * @brief Distance manager implementation.
 * @see distance_manager.h
 */

#include "distance_manager.h"
#include "vl53l0x.h"
#include "../config/pin_config.h"
#include <Arduino.h>

static VL53L0X_Sensor _sensors[TOF_COUNT];
static uint16_t _distances_mm[TOF_COUNT] = {8190, 8190, 8190, 8190, 8190};

void distance_manager_init(void) {
    // 1. Populate sensor array with XSHUT pins and I2C addresses
    _sensors[TOF_FRONT].xshut_pin = PIN_TOF_XSHUT_FRONT;
    _sensors[TOF_FRONT].i2c_address = TOF_ADDR_FRONT;

    _sensors[TOF_FRONT_LEFT].xshut_pin = PIN_TOF_XSHUT_FRONT_LEFT;
    _sensors[TOF_FRONT_LEFT].i2c_address = TOF_ADDR_FRONT_LEFT;

    _sensors[TOF_FRONT_RIGHT].xshut_pin = PIN_TOF_XSHUT_FRONT_RIGHT;
    _sensors[TOF_FRONT_RIGHT].i2c_address = TOF_ADDR_FRONT_RIGHT;

    _sensors[TOF_LEFT].xshut_pin = PIN_TOF_XSHUT_LEFT;
    _sensors[TOF_LEFT].i2c_address = TOF_ADDR_LEFT;

    _sensors[TOF_RIGHT].xshut_pin = PIN_TOF_XSHUT_RIGHT;
    _sensors[TOF_RIGHT].i2c_address = TOF_ADDR_RIGHT;

    // 2. Initialize the multiplexed bus
    uint8_t count = vl53l0x_init_all(_sensors, TOF_COUNT);
    Serial.print("[Distance] Init success count: ");
    Serial.print(count);
    Serial.println("/5");
}

void distance_manager_update(void) {
    // CONTINUOUS POLLING:
    // With the Pololu library in continuous mode, reading all 5 sensors 
    // takes very little time. We can read them all in one pass.
    for (uint8_t i = 0; i < TOF_COUNT; i++) {
        uint16_t raw_mm = vl53l0x_read_distance_mm(&_sensors[i]);
        
        // Simple Exponential Moving Average (EMA) filter to reduce noise
        if (raw_mm != 8190 && raw_mm < 8000) {
            if (_distances_mm[i] == 8190) {
                _distances_mm[i] = raw_mm; // Initialize on first valid read
            } else {
                _distances_mm[i] = (uint16_t)(0.7f * _distances_mm[i] + 0.3f * raw_mm);
            }
        } else {
            _distances_mm[i] = 8190;
        }
    }
}

uint16_t distance_get_mm(DistanceSensorID id) {
    if (id < TOF_COUNT) {
        return _distances_mm[id];
    }
    return 8190;
}

bool distance_has_wall_left(void) {
    return _distances_mm[TOF_LEFT] < WALL_THRESHOLD_SIDE_MM;
}

bool distance_has_wall_right(void) {
    return _distances_mm[TOF_RIGHT] < WALL_THRESHOLD_SIDE_MM;
}

bool distance_has_wall_front(void) {
    // If any of the front-facing sensors report a wall very close
    return _distances_mm[TOF_FRONT] < WALL_THRESHOLD_FRONT_MM;
}

float distance_get_centering_error(void) {
    bool wall_l = distance_has_wall_left();
    bool wall_r = distance_has_wall_right();

    // Standard maze cell is 180mm. 
    // Assuming robot width is ~120mm, target distance to wall is ~30mm.
    const float TARGET_WALL_DIST_MM = 30.0f;

    if (wall_l && wall_r) {
        // Both walls: Error is simply the difference
        // Positive error = robot is too far LEFT (L is small, R is large)
        // Wait, if L is small and R is large, L - R is negative.
        // Let's define Positive Error = Robot is shifted LEFT. 
        // L=40, R=60 -> we are 10mm shifted Left. error = 40 - 60 = -20 (Negative)
        // Let's use (Right - Left). R=60, L=40 -> error = +20 (we are left, need to correct right).
        return (float)_distances_mm[TOF_RIGHT] - (float)_distances_mm[TOF_LEFT];
    } else if (wall_l) {
        // Only left wall present. 
        // Target is 30mm. If L=20, we are too far left. error = 30 - 20 = +10.
        return TARGET_WALL_DIST_MM - (float)_distances_mm[TOF_LEFT];
    } else if (wall_r) {
        // Only right wall present.
        // Target is 30mm. If R=20, we are too far right. error = 20 - 30 = -10.
        return (float)_distances_mm[TOF_RIGHT] - TARGET_WALL_DIST_MM;
    }

    return 0.0f;
}

float distance_get_front_alignment_error(void) {
    // If front-left and front-right both see a wall close by,
    // calculate angular deviation based on their difference.
    if (_distances_mm[TOF_FRONT_LEFT] < 150 && _distances_mm[TOF_FRONT_RIGHT] < 150) {
        return (float)_distances_mm[TOF_FRONT_RIGHT] - (float)_distances_mm[TOF_FRONT_LEFT];
    }
    return 0.0f;
}
