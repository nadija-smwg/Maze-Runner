/**
 * @file timer.h
 * @brief System timer utilities and control loop timer setup.
 *
 * Provides a 1kHz control loop timer using a hardware timer interrupt.
 * The control loop callback is called from the ISR context at precisely
 * 1kHz (1ms period) to ensure deterministic control timing.
 *
 * Uses TIM4 (or another free timer) for the control loop tick.
 *
 * Dependencies: robot_config
 */

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include <stdbool.h>
#include "../config/robot_config.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Control Loop Callback Type
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Control loop callback function pointer type.
 *
 * This function will be called from a timer ISR at CONTROL_LOOP_FREQ_HZ.
 * It must be fast (< 500µs) and ISR-safe (no blocking, no Serial).
 */
typedef void (*ControlLoopCallback)(void);

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialize the control loop timer.
 *
 * Sets up a hardware timer to fire an interrupt at CONTROL_LOOP_FREQ_HZ.
 * The callback function is called from the ISR.
 *
 * @param callback Function to call at each control tick (1kHz)
 *
 * TODO: Implement using TIM4 or HardwareTimer from Arduino STM32:
 *       - Configure timer for 1kHz interrupt
 *       - Register callback in the ISR
 *       - Start the timer
 *
 * TODO: Alternative: Use register-level setup for TIM4:
 *       - Enable TIM4 clock
 *       - Set PSC and ARR for 1kHz
 *       - Enable update interrupt (DIER)
 *       - Set NVIC priority
 *       - Start timer
 */
void timer_init(ControlLoopCallback callback);

/**
 * @brief Start the control loop timer.
 *
 * Enables the timer interrupt to begin calling the callback.
 *
 * TODO: Enable the timer (TIMx->CR1 |= CEN) and NVIC interrupt.
 */
void timer_start(void);

/**
 * @brief Stop the control loop timer.
 *
 * Disables the timer interrupt. Useful during calibration or menu mode.
 *
 * TODO: Disable the timer and NVIC interrupt.
 */
void timer_stop(void);

/**
 * @brief Check if the control loop tick flag is set.
 *
 * Alternative to callback-based approach: poll this flag in the main loop.
 *
 * @return true if a tick has occurred since last call
 *
 * TODO: Implement using a volatile flag set in the ISR.
 */
bool timer_tick_pending(void);

/**
 * @brief Clear the tick pending flag.
 *
 * TODO: Reset the volatile tick flag.
 */
void timer_tick_clear(void);

/**
 * @brief Get high-resolution microsecond timestamp.
 *
 * @return Microseconds since boot
 *
 * TODO: Use micros() from Arduino or implement via DWT cycle counter
 *       for sub-microsecond precision.
 */
uint32_t timer_micros(void);

#endif /* TIMER_H */
