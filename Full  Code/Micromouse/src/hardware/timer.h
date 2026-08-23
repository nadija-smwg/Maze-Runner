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

#include "../config/robot_config.h"
#include <stdbool.h>
#include <stdint.h>


/* ═══════════════════════════════════════════════════════════════════════════
 *  Control Loop Callback Type
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief Control loop callback function pointer type.
 *
 * This function will be called from a timer ISR at CONTROL_LOOP_FREQ_HZ.
 * It must be fast (< 500µs) and ISR-safe (no blocking, no Serial).
 */
typedef void (*ControlLoopCallback)(void);

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief Initialize the control loop timer.
 *
 * Sets up a hardware timer to fire an interrupt at CONTROL_LOOP_FREQ_HZ.
 * The callback function is called from the ISR.
 *
 * @param callback Function to call at each control tick (1kHz)
 *
 * Implementation: Uses TIM4 HardwareTimer with setOverflow(1000, HERTZ_FORMAT)
 * and attachInterrupt() to fire a 1kHz ISR that invokes the callback.
 */
void timer_init(ControlLoopCallback callback);

/**
 * @brief Start the control loop timer.
 *
 * Enables the timer interrupt to begin calling the callback.
 *
 * Calls HardwareTimer::resume() to start the TIM4 clock and enable interrupts.
 */
void timer_start(void);

/**
 * @brief Stop the control loop timer.
 *
 * Disables the timer interrupt. Useful during calibration or menu mode.
 *
 * Calls HardwareTimer::pause() to stop the TIM4 clock.
 */
void timer_stop(void);

/**
 * @brief Check if the control loop tick flag is set.
 *
 * Alternative to callback-based approach: poll this flag in the main loop.
 *
 * @return true if a tick has occurred since last call
 *
 * Uses a volatile bool flag set in the TIM4 ISR.
 */
bool timer_tick_pending(void);

/**
 * @brief Clear the tick pending flag.
 *
 * Resets the volatile tick flag to false.
 */
void timer_tick_clear(void);

/**
 * @brief Get high-resolution microsecond timestamp.
 *
 * @return Microseconds since boot
 *
 * Uses Arduino STM32 core micros() which leverages the DWT cycle counter.
 */
uint32_t timer_micros(void);

#endif /* TIMER_H */
