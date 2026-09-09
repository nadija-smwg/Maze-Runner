/**
 * @file competition_state.cpp
 * @brief Competition state — tracks runs, times, and best result.
 *
 * Usage:
 *   competition_state_init()      — call once in setup()
 *   competition_record_run()      — call at start of each fast run
 *   competition_record_time(ms)   — call when fast run finishes
 *   competition_get_best_time()   — read best time for OLED display
 *
 * Serial output format per run:
 *   [Comp] Run #3  Time: 4521 ms  Best: 4312 ms
 */

#include "competition_state.h"
#include <Arduino.h>

/* ─── Private state ──────────────────────────────────────────────────── */
static uint8_t  _run_count  = 0;
static uint32_t _best_time  = 0xFFFFFFFFUL;  /* max = "no best yet" */
static uint32_t _last_time  = 0;

/* ─── Public API ─────────────────────────────────────────────────────── */

void competition_state_init(void)
{
    _run_count = 0;
    _best_time = 0xFFFFFFFFUL;
    _last_time = 0;
    Serial.println(F("[Comp] State initialized. Ready."));
}

void competition_record_run(void)
{
    _run_count++;
    Serial.print(F("[Comp] Starting run #"));
    Serial.println(_run_count);
}

void competition_record_time(uint32_t time_ms)
{
    _last_time = time_ms;

    if (time_ms < _best_time) {
        _best_time = time_ms;
    }

    Serial.print(F("[Comp] Run #"));
    Serial.print(_run_count);
    Serial.print(F("  Time: "));
    Serial.print(time_ms);
    Serial.print(F(" ms  Best: "));
    if (_best_time == 0xFFFFFFFFUL) {
        Serial.println(F("--"));
    } else {
        Serial.print(_best_time);
        Serial.println(F(" ms"));
    }
}

uint32_t competition_get_best_time(void)
{
    return (_best_time == 0xFFFFFFFFUL) ? 0 : _best_time;
}
