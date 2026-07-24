/**
 * @file encoder.cpp
 * @brief Quadrature encoder implementation using STM32 timer encoder mode.
 *
 * Register-level configuration for TIM2 (left, 32-bit) and TIM3 (right, 16-bit)
 * in Encoder Mode 3. Matches the proven patterns from the testing code.
 *
 * @see encoder.h for public API documentation
 */

#include "encoder.h"
#include <Arduino.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private State
 * ═══════════════════════════════════════════════════════════════════════════ */

/** Previous count values for delta calculation. */
static int32_t _last_left_count  = 0;
static int32_t _last_right_count = 0;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialization
 * ═══════════════════════════════════════════════════════════════════════════ */

void encoder_init(void) {
    /**
     * TODO: Setup Left Encoder (TIM2, 32-bit timer)
     *
     * 1. Enable clocks:
     *    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
     *    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
     *
     * 2. Configure PA0, PA1 as Alternate Function:
     *    GPIOA->MODER: set bits for AF mode (0b10) on pins 0 and 1
     *    GPIOA->AFR[0]: set AF1 for TIM2 on pins 0 and 1
     *
     * 3. Configure TIM2:
     *    - Reset: CR1=0, CR2=0, SMCR=0, CCMR1=0, CCER=0
     *    - PSC=0, ARR=0xFFFFFFFF (full 32-bit range)
     *    - Encoder Mode 3: SMCR |= SMS_0 | SMS_1
     *    - CH1/CH2 as input: CCMR1 |= CC1S_0 | CC2S_0
     *    - Input filter: CCMR1 |= (3<<4) | (3<<12)
     *    - CNT=0, enable: CR1 |= CEN
     */

    /**
     * TODO: Setup Right Encoder (TIM3, 16-bit timer)
     *
     * 1. Enable clocks:
     *    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
     *    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
     *
     * 2. Configure PA6, PA7 as Alternate Function:
     *    GPIOA->MODER: set bits for AF mode on pins 6 and 7
     *    GPIOA->AFR[0]: set AF2 for TIM3 on pins 6 and 7
     *
     * 3. Configure TIM3:
     *    - Reset all registers
     *    - PSC=0, ARR=0xFFFF (16-bit full range)
     *    - Encoder Mode 3
     *    - CH1/CH2 input with filter
     *    - CNT=0, enable
     */

    _last_left_count  = 0;
    _last_right_count = 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Count Access
 * ═══════════════════════════════════════════════════════════════════════════ */

int32_t encoder_get_count(EncoderID enc) {
    /**
     * TODO: For ENCODER_LEFT:  return (int32_t)TIM2->CNT;
     * TODO: For ENCODER_RIGHT: return (int32_t)(int16_t)TIM3->CNT;
     *       (int16_t cast handles 16-bit wrap-around / sign extension)
     */
    return 0;
}

int32_t encoder_get_delta(EncoderID enc) {
    /**
     * TODO: For ENCODER_LEFT:
     *   int32_t current = (int32_t)TIM2->CNT;
     *   int32_t delta = current - _last_left_count;
     *   _last_left_count = current;
     *   return delta;
     *
     * TODO: For ENCODER_RIGHT:
     *   int16_t current = (int16_t)TIM3->CNT;
     *   int16_t delta = current - (int16_t)_last_right_count;
     *   _last_right_count = current;
     *   return (int32_t)delta;
     */
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Reset
 * ═══════════════════════════════════════════════════════════════════════════ */

void encoder_reset(EncoderID enc) {
    /**
     * TODO: Set TIMx->CNT = 0 for the given encoder.
     * TODO: Reset the corresponding _last_xxx_count to 0.
     */
}

void encoder_reset_all(void) {
    /**
     * TODO: Reset both TIM2->CNT and TIM3->CNT to 0.
     * TODO: Reset _last_left_count and _last_right_count to 0.
     */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Conversions
 * ═══════════════════════════════════════════════════════════════════════════ */

float encoder_counts_to_mm(int32_t counts) {
    /**
     * TODO: return counts * MM_PER_COUNT;
     */
    return 0.0f;
}

float encoder_counts_to_speed(float counts_per_sec) {
    /**
     * TODO: return counts_per_sec * MM_PER_COUNT;
     */
    return 0.0f;
}

float encoder_counts_to_rpm(float counts_per_sec) {
    /**
     * TODO: return (counts_per_sec * 60.0f) / ENCODER_CPR;
     */
    return 0.0f;
}
