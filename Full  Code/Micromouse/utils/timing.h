/**
 * @file timing.h
 * @brief General non-blocking timing utilities.
 */

#ifndef TIMING_H
#define TIMING_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Check if a timeout has expired based on millis().
 */
bool time_is_expired(uint32_t start_time, uint32_t duration);

#endif /* TIMING_H */
