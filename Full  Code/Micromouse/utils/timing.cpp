/**
 * @file timing.cpp
 * @brief Timing utilities implementation.
 * @see timing.h
 */

#include "timing.h"
#include <Arduino.h>

bool time_is_expired(uint32_t start_time, uint32_t duration) {
    return (millis() - start_time) >= duration;
}
