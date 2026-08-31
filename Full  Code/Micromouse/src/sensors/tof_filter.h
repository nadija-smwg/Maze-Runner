/**
 * @file tof_filter.h
 * @brief Per-sensor filtering pipeline for VL53L0X Time-of-Flight sensors.
 *
 * Implements a multi-stage filtering pipeline for each of the 5 ToF sensors:
 *
 *   Raw reading
 *       │
 *   [1] Range validity check  (< TOF_MIN_DIST or > TOF_MAX_DIST → reject)
 *       │
 *   [2] Calibration offset    (corrected = raw + offset)
 *       │
 *   [3] Median-3 filter       (ring buffer of 3, returns middle value)
 *       │
 *   [4] Outlier jump reject   (|median − prev| > TOF_MAX_JUMP → reject)
 *       │
 *   [5] EMA smoothing         (α × median + (1−α) × ema_prev)
 *       │
 *   Filtered distance (mm)
 *
 * Each sensor has its own independent ToF_FilterState — no shared state.
 *
 * Dependencies: stdint, stdbool
 */

#ifndef TOF_FILTER_H
#define TOF_FILTER_H

#include <stdint.h>
#include <stdbool.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  Filter Tuning Constants
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @defgroup ToFFilterConstants ToF Filter Tuning Parameters
 *  Adjust these based on your measured sensor noise and robot geometry.
 *  @{
 */

/** Minimum plausible distance reading (mm). Below this = sensor error. */
#define TOF_MIN_DIST_MM         20u

/** Maximum plausible distance reading (mm). Above this = out of range. */
#define TOF_MAX_DIST_MM         500u

/**
 * Maximum plausible change between two consecutive filtered samples (mm).
 * A jump larger than this is treated as an outlier and the previous value
 * is held. Tune based on max speed: at 500 mm/s with 10 ms sample period,
 * max real jump is ~5 mm — 50 mm gives comfortable margin.
 */
#define TOF_MAX_JUMP_MM         50u

/**
 * EMA smoothing coefficient.
 * α = 0.30 → 30% new sample, 70% previous filtered value.
 * Increase toward 1.0 for faster response, decrease for more smoothing.
 * Recommended range for micromouse: 0.20 – 0.40
 */
#define TOF_EMA_ALPHA           0.30f

/** Number of samples in the median ring buffer. Must be 3 (fixed). */
#define TOF_MEDIAN_SIZE         3u

/** @} */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Per-Sensor State
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Internal state for one ToF sensor's filter pipeline.
 *
 * Instantiate one of these per sensor. Zero-initialize before calling
 * tof_filter_init().
 */
typedef struct {
    /* --- Median ring buffer --- */
    uint16_t median_buf[TOF_MEDIAN_SIZE];   /**< Circular sample buffer       */
    uint8_t  median_index;                  /**< Next write position (0–2)    */

    /* --- EMA state --- */
    float    ema;                           /**< Current EMA accumulator (mm) */

    /* --- Jump-rejection state --- */
    uint16_t previous;                      /**< Last accepted median output  */

    /* --- Calibration --- */
    int16_t  offset;                        /**< Per-sensor offset (mm)       */

    /* --- Output --- */
    uint16_t filtered;                      /**< Latest filtered distance (mm)*/

    /* --- Flags --- */
    bool     valid;                         /**< Last reading was in range     */
    bool     initialized;                   /**< Filter has been seeded       */
} ToF_FilterState;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialize (or re-initialize) a filter state.
 *
 * Clears all history and resets the filter to an un-seeded state.
 * The calibration offset is preserved across calls to allow setting the
 * offset before calling init.
 *
 * @param s  Pointer to the ToF_FilterState to initialize
 */
void tof_filter_init(ToF_FilterState *s);

/**
 * @brief Process a new raw distance reading through the full pipeline.
 *
 * Runs all 5 filter stages in order and stores the result in s->filtered.
 * On any rejection stage, the previous s->filtered value is preserved.
 *
 * @param s    Pointer to the sensor's filter state
 * @param raw  Raw distance in mm from vl53l0x_read_distance_mm()
 * @return     Filtered distance in mm
 */
uint16_t tof_filter_process(ToF_FilterState *s, uint16_t raw);

/**
 * @brief Set the calibration offset for a sensor.
 *
 * The offset is added to each raw reading after validity check:
 *   corrected = raw + offset
 *
 * To apply a negative offset (sensor reads high): set offset = -N.
 * To apply a positive offset (sensor reads low):  set offset = +N.
 *
 * @param s       Pointer to the sensor's filter state
 * @param offset  Offset in mm (positive or negative)
 */
void tof_filter_set_offset(ToF_FilterState *s, int16_t offset);

/**
 * @brief Force-seed the filter with a known good value.
 *
 * Useful during initialization if the first reading is known reliable.
 * Seeds the median buffer and EMA with the given value so the filter
 * does not start from zero.
 *
 * @param s     Pointer to the sensor's filter state
 * @param value Starting value in mm
 */
void tof_filter_seed(ToF_FilterState *s, uint16_t value);

#endif /* TOF_FILTER_H */
