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
    // ═══════════════════════════════════════════════════════════════════════
    //  Left Encoder — TIM2 (32-bit timer, PA0 = CH1, PA1 = CH2, AF1)
    // ═══════════════════════════════════════════════════════════════════════
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Set PA0 and PA1 to Alternate Function mode (binary 10 = 2)
    GPIOA->MODER &= ~(3 << (0 * 2));
    GPIOA->MODER &= ~(3 << (1 * 2));
    GPIOA->MODER |= (2 << (0 * 2));
    GPIOA->MODER |= (2 << (1 * 2));

    // Set AF1 (TIM2) in AFR[0] for pin 0 and pin 1
    GPIOA->AFR[0] &= ~(0xF << (0 * 4));
    GPIOA->AFR[0] &= ~(0xF << (1 * 4));
    GPIOA->AFR[0] |= (1 << (0 * 4));
    GPIOA->AFR[0] |= (1 << (1 * 4));

    // Reset TIM2 registers
    TIM2->CR1   = 0;
    TIM2->CR2   = 0;
    TIM2->SMCR  = 0;
    TIM2->CCMR1 = 0;
    TIM2->CCER  = 0;

    TIM2->PSC = 0;
    TIM2->ARR = 0xFFFFFFFF; // Full 32-bit timer range

    // Encoder Mode 3: counts on both edges of both channels (4x quadrature)
    TIM2->SMCR |= TIM_SMCR_SMS_0;
    TIM2->SMCR |= TIM_SMCR_SMS_1;

    // Connect CH1 and CH2 to TI1 and TI2 inputs
    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0;
    TIM2->CCMR1 |= TIM_CCMR1_CC2S_0;

    // Add digital input filter (fSAMPLING = fDTS/8, N=6)
    TIM2->CCMR1 |= (3 << 4);
    TIM2->CCMR1 |= (3 << 12);

    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;

    // ═══════════════════════════════════════════════════════════════════════
    //  Right Encoder — TIM3 (16-bit timer, PA6 = CH1, PA7 = CH2, AF2)
    // ═══════════════════════════════════════════════════════════════════════
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Set PA6 and PA7 to Alternate Function mode (binary 10 = 2)
    GPIOA->MODER &= ~(3 << (6 * 2));
    GPIOA->MODER &= ~(3 << (7 * 2));
    GPIOA->MODER |= (2 << (6 * 2));
    GPIOA->MODER |= (2 << (7 * 2));

    // Set AF2 (TIM3) in AFR[0] for pin 6 and pin 7
    GPIOA->AFR[0] &= ~(0xF << (6 * 4));
    GPIOA->AFR[0] &= ~(0xF << (7 * 4));
    GPIOA->AFR[0] |= (2 << (6 * 4));
    GPIOA->AFR[0] |= (2 << (7 * 4));

    // Reset TIM3 registers
    TIM3->CR1   = 0;
    TIM3->CR2   = 0;
    TIM3->SMCR  = 0;
    TIM3->CCMR1 = 0;
    TIM3->CCER  = 0;

    TIM3->PSC = 0;
    TIM3->ARR = 0xFFFF; // Full 16-bit timer range

    // Encoder Mode 3 (4x quadrature decoding)
    TIM3->SMCR |= TIM_SMCR_SMS_0;
    TIM3->SMCR |= TIM_SMCR_SMS_1;

    // Connect CH1 and CH2 inputs
    TIM3->CCMR1 |= TIM_CCMR1_CC1S_0;
    TIM3->CCMR1 |= TIM_CCMR1_CC2S_0;

    // Add digital input filter
    TIM3->CCMR1 |= (3 << 4);
    TIM3->CCMR1 |= (3 << 12);

    TIM3->CNT = 0;
    TIM3->CR1 |= TIM_CR1_CEN;

    _last_left_count  = 0;
    _last_right_count = 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Count Access
 * ═══════════════════════════════════════════════════════════════════════════ */

int32_t encoder_get_count(EncoderID enc) {
    if (enc == ENCODER_LEFT) {
        // TIM2 is a 32-bit timer, direct cast is safe and accurate
        return (int32_t)TIM2->CNT;
    } else {
        // TIM3 is a 16-bit timer: cast to int16_t first for sign extension,
        // then upgrade to int32_t so it matches our signed 32-bit API
        return (int32_t)(int16_t)TIM3->CNT;
    }
}

int32_t encoder_get_delta(EncoderID enc) {
    if (enc == ENCODER_LEFT) {
        int32_t current = (int32_t)TIM2->CNT;
        int32_t delta = current - _last_left_count;
        _last_left_count = current;
        return delta;
    } else {
        // For 16-bit TIM3, perform subtraction in 16-bit signed math so overflows/underflows
        // wrap around correctly, then cast to 32-bit signed integer
        int16_t current = (int16_t)TIM3->CNT;
        int16_t delta = current - (int16_t)_last_right_count;
        _last_right_count = current;
        return (int32_t)delta;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Reset
 * ═══════════════════════════════════════════════════════════════════════════ */

void encoder_reset(EncoderID enc) {
    if (enc == ENCODER_LEFT) {
        TIM2->CNT = 0;
        _last_left_count = 0;
    } else {
        TIM3->CNT = 0;
        _last_right_count = 0;
    }
}

void encoder_reset_all(void) {
    TIM2->CNT = 0;
    TIM3->CNT = 0;
    _last_left_count  = 0;
    _last_right_count = 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Conversions
 * ═══════════════════════════════════════════════════════════════════════════ */

float encoder_counts_to_mm(int32_t counts) {
    return (float)counts * MM_PER_COUNT;
}

float encoder_counts_to_speed(float counts_per_sec) {
    return counts_per_sec * MM_PER_COUNT;
}

float encoder_counts_to_rpm(float counts_per_sec) {
    return (counts_per_sec * 60.0f) / ENCODER_CPR;
}
