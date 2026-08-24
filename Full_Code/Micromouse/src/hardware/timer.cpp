/**
 * @file timer.cpp
 * @brief System timer implementation for 1kHz control loop.
 *
 * @see timer.h for public API documentation
 */

#include "timer.h"
#include <Arduino.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private State
 * ═══════════════════════════════════════════════════════════════════════════ */

static ControlLoopCallback _control_callback = nullptr;
static volatile bool _tick_pending = false;
static HardwareTimer *MyTim = nullptr;

/* ═══════════════════════════════════════════════════════════════════════════
 *  ISR
 * ═══════════════════════════════════════════════════════════════════════════ */

static void _timer_isr(void) {
    // 1. Mark that a 1ms control loop tick has occurred
    _tick_pending = true;

    // 2. Invoke the registered 1kHz control loop callback if valid
    if (_control_callback) {
        _control_callback();
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Implementation
 * ═══════════════════════════════════════════════════════════════════════════ */

void timer_init(ControlLoopCallback callback) {
    _control_callback = callback;

    // Instantiate HardwareTimer on TIM4 (TIM1=PWM, TIM2=LeftEnc, TIM3=RightEnc)
    if (MyTim == nullptr) {
        MyTim = new HardwareTimer(TIM4);
    }

    MyTim->pause();
    
    // Configure TIM4 for 1000 Hz (1 ms period) overflow interrupt
    MyTim->setOverflow(CONTROL_LOOP_FREQ_HZ, HERTZ_FORMAT);
    MyTim->attachInterrupt(_timer_isr);
}

void timer_start(void) {
    if (MyTim) {
        MyTim->resume(); // Starts the timer clock and enables interrupts
    }
}

void timer_stop(void) {
    if (MyTim) {
        MyTim->pause();  // Pauses the timer clock
    }
}

bool timer_tick_pending(void) {
    return _tick_pending;
}

void timer_tick_clear(void) {
    _tick_pending = false;
}

uint32_t timer_micros(void) {
    // Arduino STM32 core micros() uses DWT cycle counter for 1µs precision
    return micros();
}
