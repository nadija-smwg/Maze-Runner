/**
 * @file tof_filter.cpp
 * @brief ToF sensor filtering pipeline implementation.
 * @see tof_filter.h
 *
 * Pipeline stages per call to tof_filter_process():
 *
 *   [1] Range validity  — rejects < TOF_MIN_DIST_MM or > TOF_MAX_DIST_MM
 *   [2] Calibration     — adds per-sensor offset
 *   [3] Median-3        — ring buffer of 3, output is middle value
 *   [4] Jump rejection  — rejects sudden changes > TOF_MAX_JUMP_MM
 *   [5] EMA             — exponential moving average with TOF_EMA_ALPHA
 */

#include "tof_filter.h"
#include <string.h>   /* memset */
#include <stdlib.h>   /* abs    */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Sort three uint16_t values and return the median (middle value).
 *
 * Uses a 3-element sorting network — no branches on the comparison path.
 * Guaranteed to produce the middle of {a, b, c} in exactly 3 comparisons.
 */
static uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t tmp;

    /* Pass 1 */
    if (a > b) { tmp = a; a = b; b = tmp; }
    /* Pass 2 */
    if (b > c) { tmp = b; b = c; c = tmp; }
    /* Pass 3 */
    if (a > b) { tmp = a; a = b; b = tmp; }

    (void)a; (void)c; /* suppress unused-variable warnings */
    return b;
}

/**
 * @brief Push a new sample into the median ring buffer and return median.
 *
 * On the very first call (s->initialized == false), the ring buffer is
 * pre-filled with the first sample so the filter starts at a sensible
 * value rather than blending in two zeros.
 *
 * @param s      Pointer to filter state
 * @param sample New raw (calibrated) sample
 * @return       Median of the three most recent samples
 */
static uint16_t median_update(ToF_FilterState *s, uint16_t sample)
{
    if (!s->initialized)
    {
        /* Pre-fill buffer with first reading to avoid 0,0,sample → median=0 */
        s->median_buf[0] = sample;
        s->median_buf[1] = sample;
        s->median_buf[2] = sample;
        s->median_index  = 0;
        return sample;
    }

    /* Overwrite oldest sample */
    s->median_buf[s->median_index] = sample;
    s->median_index = (s->median_index + 1u) % TOF_MEDIAN_SIZE;

    return median3(s->median_buf[0],
                   s->median_buf[1],
                   s->median_buf[2]);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

void tof_filter_init(ToF_FilterState *s)
{
    if (!s) return;

    int16_t saved_offset = s->offset; /* preserve calibration across resets */

    memset(s, 0, sizeof(ToF_FilterState));

    s->offset = saved_offset;
    /* All other fields are zero — valid=false, initialized=false, etc. */
}

void tof_filter_set_offset(ToF_FilterState *s, int16_t offset)
{
    if (!s) return;
    s->offset = offset;
}

void tof_filter_seed(ToF_FilterState *s, uint16_t value)
{
    if (!s) return;

    s->median_buf[0] = value;
    s->median_buf[1] = value;
    s->median_buf[2] = value;
    s->median_index  = 0;

    s->ema           = (float)value;
    s->previous      = value;
    s->filtered      = value;
    s->initialized   = true;
    s->valid         = true;
}

uint16_t tof_filter_process(ToF_FilterState *s, uint16_t raw)
{
    if (!s) return 0u;

    /* ── Stage 1: Range validity ──────────────────────────────────────────── */
    if (raw < TOF_MIN_DIST_MM || raw > TOF_MAX_DIST_MM)
    {
        s->valid = false;
        /* Hold previous filtered output unchanged */
        return s->filtered;
    }

    s->valid = true;

    /* ── Stage 2: Calibration offset ─────────────────────────────────────── */
    int32_t corrected = (int32_t)raw + (int32_t)s->offset;

    /* Clamp to valid range after offset */
    if (corrected < (int32_t)TOF_MIN_DIST_MM)
        corrected = (int32_t)TOF_MIN_DIST_MM;

    if (corrected > (int32_t)TOF_MAX_DIST_MM)
        corrected = (int32_t)TOF_MAX_DIST_MM;

    uint16_t cal = (uint16_t)corrected;

    /* ── Stage 3: Median-3 filter ─────────────────────────────────────────── */
    uint16_t med = median_update(s, cal);

    /* ── Stage 4: Outlier jump rejection ─────────────────────────────────── */
    if (s->initialized)
    {
        uint16_t jump = (med > s->previous) ?
                        (med - s->previous) :
                        (s->previous - med);

        if (jump > TOF_MAX_JUMP_MM)
        {
            /* Reject — hold last filtered output */
            return s->filtered;
        }
    }

    s->previous = med;

    /* ── Stage 5: EMA smoothing ───────────────────────────────────────────── */
    if (!s->initialized)
    {
        /* Seed EMA with first valid median so we don't ramp from zero */
        s->ema         = (float)med;
        s->filtered    = med;
        s->initialized = true;
    }
    else
    {
        s->ema = TOF_EMA_ALPHA * (float)med +
                 (1.0f - TOF_EMA_ALPHA) * s->ema;

        s->filtered = (uint16_t)s->ema;
    }

    return s->filtered;
}
