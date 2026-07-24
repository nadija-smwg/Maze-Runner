/**
 * @file encoder.h
 * @brief Quadrature encoder interface using hardware timer encoder mode.
 *
 * Reads N20 motor encoders via STM32 timer encoder mode (no interrupts
 * needed — the timer hardware counts automatically).
 *
 * Hardware configuration:
 *   - Left encoder:  TIM2 (32-bit), PA0 (CH1), PA1 (CH2)
 *   - Right encoder: TIM3 (16-bit), PA6 (CH1), PA7 (CH2)
 *
 * Encoder Mode 3: counts on both rising and falling edges of both
 * channels → 4× resolution (quadrature decoding).
 *
 * Motor specs:
 *   - 7 PPR × 65:1 gear ratio × 4 (quadrature) = 1820 counts/revolution
 *   - Wheel circumference ≈ 106.81 mm
 *   - Resolution ≈ 0.0587 mm/count
 *
 * Dependencies: pin_config, robot_config
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "../config/pin_config.h"
#include "../config/robot_config.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Encoder Identification
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Encoder identifier enum.
 */
typedef enum {
    ENCODER_LEFT  = 0,  /**< Left encoder (TIM2)  */
    ENCODER_RIGHT = 1   /**< Right encoder (TIM3) */
} EncoderID;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialize both encoder timers in quadrature encoder mode.
 *
 * Configures TIM2 and TIM3 in Encoder Mode 3 (counts on both edges of
 * both channels) using direct register access.
 *
 * TODO: Implement register-level setup for TIM2 (32-bit, PA0/PA1):
 *       - Enable GPIOA and TIM2 clocks
 *       - Set PA0, PA1 to Alternate Function (AF1 for TIM2)
 *       - Configure Encoder Mode 3 (SMS=011)
 *       - Set input capture with filter
 *       - Set ARR to 0xFFFFFFFF (32-bit full range)
 *       - Start timer
 *
 * TODO: Implement register-level setup for TIM3 (16-bit, PA6/PA7):
 *       - Enable GPIOA and TIM3 clocks
 *       - Set PA6, PA7 to Alternate Function (AF2 for TIM3)
 *       - Configure Encoder Mode 3
 *       - Set input capture with filter
 *       - Set ARR to 0xFFFF (16-bit full range)
 *       - Start timer
 */
void encoder_init(void);

/**
 * @brief Get the raw count value of an encoder.
 *
 * For the left encoder (TIM2, 32-bit): reads TIM2->CNT directly.
 * For the right encoder (TIM3, 16-bit): reads TIM3->CNT with sign extension.
 *
 * @param enc Encoder identifier
 * @return    Raw encoder count (signed, handles wrap-around)
 *
 * TODO: Implement TIM2->CNT read (32-bit, direct cast).
 * TODO: Implement TIM3->CNT read (16-bit, cast through int16_t for sign).
 */
int32_t encoder_get_count(EncoderID enc);

/**
 * @brief Get the count change since the last call to this function.
 *
 * Tracks the previous count internally and returns the delta.
 * Handles timer overflow/underflow correctly.
 *
 * @param enc Encoder identifier
 * @return    Count change since last call (positive = forward)
 *
 * TODO: Implement delta calculation with proper overflow handling.
 * TODO: For TIM3 (16-bit), use int16_t arithmetic for correct wrap-around.
 */
int32_t encoder_get_delta(EncoderID enc);

/**
 * @brief Reset an encoder count to zero.
 *
 * @param enc Encoder identifier
 *
 * TODO: Set TIMx->CNT = 0 and reset internal tracking variables.
 */
void encoder_reset(EncoderID enc);

/**
 * @brief Reset both encoders to zero.
 *
 * TODO: Reset TIM2->CNT and TIM3->CNT and all tracking state.
 */
void encoder_reset_all(void);

/**
 * @brief Convert encoder counts to distance in millimeters.
 *
 * @param counts Raw encoder count value
 * @return       Distance in mm
 *
 * TODO: Implement as: counts * MM_PER_COUNT
 */
float encoder_counts_to_mm(int32_t counts);

/**
 * @brief Convert encoder counts per second to speed in mm/s.
 *
 * @param counts_per_sec Encoder count rate
 * @return               Speed in mm/s
 *
 * TODO: Implement as: counts_per_sec * MM_PER_COUNT
 */
float encoder_counts_to_speed(float counts_per_sec);

/**
 * @brief Convert encoder counts per second to RPM.
 *
 * @param counts_per_sec Encoder count rate
 * @return               Motor shaft RPM
 *
 * TODO: Implement as: (counts_per_sec * 60.0f) / ENCODER_CPR
 */
float encoder_counts_to_rpm(float counts_per_sec);

#endif /* ENCODER_H */
