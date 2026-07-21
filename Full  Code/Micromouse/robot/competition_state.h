/**
 * @file competition_state.h
 * @brief Competition state tracking.
 *
 * Tracks number of runs, best time, and crash status.
 */

#ifndef COMPETITION_STATE_H
#define COMPETITION_STATE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize competition state.
 */
void competition_state_init(void);

/**
 * @brief Increment the run counter.
 */
void competition_record_run(void);

/**
 * @brief Record the time of the latest fast run.
 */
void competition_record_time(uint32_t time_ms);

/**
 * @brief Get the best time so far.
 */
uint32_t competition_get_best_time(void);

#endif /* COMPETITION_STATE_H */
