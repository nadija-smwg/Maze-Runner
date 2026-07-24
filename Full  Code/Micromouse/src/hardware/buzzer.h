/**
 * @file buzzer.h
 * @brief Buzzer control for audio feedback.
 *
 * Provides tone generation and predefined beep patterns for status indication.
 *
 * Dependencies: pin_config
 */

#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include "../config/pin_config.h"

/** @brief Initialize the buzzer pin. TODO: Implement. */
void buzzer_init(void);

/** @brief Play a tone at the given frequency for the given duration.
 *  @param freq_hz Frequency in Hz
 *  @param duration_ms Duration in milliseconds
 *  TODO: Implement using tone() or timer-based PWM. */
void buzzer_tone(uint16_t freq_hz, uint16_t duration_ms);

/** @brief Short confirmation beep. TODO: Implement. */
void buzzer_beep(void);

/** @brief Double beep for warnings. TODO: Implement. */
void buzzer_beep_warning(void);

/** @brief Triple beep for errors. TODO: Implement. */
void buzzer_beep_error(void);

/** @brief Victory melody (maze solved). TODO: Implement. */
void buzzer_play_victory(void);

/** @brief Startup melody. TODO: Implement. */
void buzzer_play_startup(void);

/** @brief Stop any currently playing tone. TODO: Implement. */
void buzzer_stop(void);

#endif /* BUZZER_H */
