/**
 * @file pwm.h
 * @brief TIM1 PWM driver for motor speed control.
 *
 * Configures TIM1 channels 1 and 2 for 20kHz PWM output on PA8 and PA9
 * using direct register-level programming. TIM1 is an advanced timer
 * with MOE (Main Output Enable) that must be set for output to work.
 *
 * PWM frequency: 84MHz / (PSC+1) / (ARR+1) = 84MHz / 1 / 4200 = 20kHz
 *
 * Dependencies: pin_config, robot_config
 */

#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include "../config/pin_config.h"
#include "../config/robot_config.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialize TIM1 for dual-channel PWM output.
 *
 * Configures PA8 (TIM1_CH1) and PA9 (TIM1_CH2) for PWM Mode 1
 * at 20kHz using register-level access.
 *
 * Implementation uses register-level access:
 * GPIOA clock, TIM1 clock, PA8/PA9 AF1, PSC=0, ARR=4199,
 * PWM Mode 1 on CH1/CH2, preload, BDTR MOE, ARPE, CEN.
 */
void pwm_init(void);

/**
 * @brief Set PWM duty cycle for the left motor (TIM1_CH1).
 *
 * @param duty Duty cycle value (0 to PWM_MAX)
 *
 * Clamps duty to [0, PWM_MAX] and writes to TIM1->CCR1.
 */
void pwm_set_left(uint16_t duty);

/**
 * @brief Set PWM duty cycle for the right motor (TIM1_CH2).
 *
 * @param duty Duty cycle value (0 to PWM_MAX)
 *
 * Clamps duty to [0, PWM_MAX] and writes to TIM1->CCR2.
 */
void pwm_set_right(uint16_t duty);

/**
 * @brief Set PWM duty cycle for both motors.
 *
 * @param left_duty  Left motor duty (0 to PWM_MAX)
 * @param right_duty Right motor duty (0 to PWM_MAX)
 *
 * Calls pwm_set_left() and pwm_set_right().
 */
void pwm_set_both(uint16_t left_duty, uint16_t right_duty);

#endif /* PWM_H */
