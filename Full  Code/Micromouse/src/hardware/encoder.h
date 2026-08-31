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
 * Implementation uses register-level access for both timers:
 *
 * TIM2 (32-bit, PA0/PA1, AF1):
 *   GPIOA+TIM2 clocks, AF1, Encoder Mode 3 (SMS=011),
 *   IC filter, ARR=0xFFFFFFFF, CEN.
 *
 * TIM3 (16-bit, PA6/PA7, AF2):
 *   GPIOA+TIM3 clocks, AF2, Encoder Mode 3 (SMS=011),
 *   IC filter, ARR=0xFFFF, CEN.
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
 * TIM2: direct (int32_t) cast of CNT.
 * TIM3: cast CNT through (int16_t) for sign extension, then to int32_t.
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
 * Uses int16_t subtraction for TIM3 to handle 16-bit wrap-around correctly.
 */
int32_t encoder_get_delta(EncoderID enc);

/**
 * @brief Reset an encoder count to zero.
 *
 * @param enc Encoder identifier
 *
 * Sets TIMx->CNT = 0 and resets internal tracking variables.
 */
void encoder_reset(EncoderID enc);

/**
 * @brief Reset both encoders to zero.
 *
 * Resets TIM2->CNT and TIM3->CNT to 0 and clears all tracking state.
 */
void encoder_reset_all(void);

/**
 * @brief Convert encoder counts to distance in millimeters.
 *
 * @param counts Raw encoder count value
 * @return       Distance in mm
 *
 * Computes: counts * MM_PER_COUNT
 */
float encoder_counts_to_mm(int32_t counts);

/**
 * @brief Convert encoder counts per second to speed in mm/s.
 *
 * @param counts_per_sec Encoder count rate
 * @return               Speed in mm/s
 *
 * Computes: counts_per_sec * MM_PER_COUNT
 */
float encoder_counts_to_speed(float counts_per_sec);

/**
 * @brief Convert encoder counts per second to RPM.
 *
 * @param counts_per_sec Encoder count rate
 * @return               Motor shaft RPM
 *
 * Computes: (counts_per_sec * 60.0f) / ENCODER_CPR
 */
float encoder_counts_to_rpm(float counts_per_sec);

/* ═══════════════════════════════════════════════════════════════════════════
 *  Velocity Filter API
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Update encoder velocity filters for both wheels.
 *
 * Reads encoder deltas, computes instantaneous counts-per-second, applies
 * the LPF (VELOCITY_LPF_ALPHA from robot_config.h), and accumulates
 * total distance for each wheel.
 *
 * Call this at a fixed rate (e.g. every 1 ms at the 1 kHz control loop).
 *
 * @param dt  Time step in seconds (e.g., 0.001f for 1 kHz)
 */
void encoder_update_velocity(float dt);

/**
 * @brief Get the filtered wheel speed in mm/s.
 *
 * Returns the LPF-smoothed speed from the last encoder_update_velocity() call.
 * Positive = forward, negative = reverse.
 *
 * @param enc  ENCODER_LEFT or ENCODER_RIGHT
 * @return     Filtered speed in mm/s
 */
float encoder_get_speed_mms(EncoderID enc);

/**
 * @brief Get the filtered wheel speed in RPM.
 *
 * @param enc  ENCODER_LEFT or ENCODER_RIGHT
 * @return     Filtered motor shaft RPM
 */
float encoder_get_rpm(EncoderID enc);

/**
 * @brief Get total distance accumulated since last encoder_reset().
 *
 * @param enc  ENCODER_LEFT or ENCODER_RIGHT
 * @return     Distance traveled in mm (signed, positive = forward)
 */
float encoder_get_distance_mm(EncoderID enc);

#endif /* ENCODER_H */
