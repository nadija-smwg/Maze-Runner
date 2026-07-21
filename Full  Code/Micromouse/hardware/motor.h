/**
 * @file motor.h
 * @brief Motor driver interface for the TB6612FNG dual H-bridge.
 *
 * Provides high-level control of left and right N20 metal gear motors
 * through the TB6612FNG motor driver IC. Handles direction pin control
 * and PWM duty cycle setting.
 *
 * Hardware connections:
 *   - Left Motor (A):  AIN1=PB12, AIN2=PB13, PWM=PA8 (TIM1_CH1)
 *   - Right Motor (B): BIN1=PB15, BIN2=PA10, PWM=PA9 (TIM1_CH2)
 *   - Standby:         STBY=PB14
 *
 * Dependencies: pwm, gpio, pin_config, robot_config
 *
 * @see pwm.h for TIM1 PWM initialization
 * @see gpio.h for direction pin abstraction
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <stdint.h>
#include "../config/pin_config.h"
#include "../config/robot_config.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Motor Identification
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Motor identifier enum.
 */
typedef enum {
    MOTOR_LEFT  = 0,    /**< Left motor (Motor A on TB6612FNG)  */
    MOTOR_RIGHT = 1     /**< Right motor (Motor B on TB6612FNG) */
} MotorID;

/**
 * @brief Motor direction enum.
 */
typedef enum {
    MOTOR_DIR_FORWARD = 0,  /**< Forward rotation   */
    MOTOR_DIR_REVERSE = 1,  /**< Reverse rotation   */
    MOTOR_DIR_BRAKE   = 2,  /**< Active braking     */
    MOTOR_DIR_COAST   = 3   /**< Free-wheeling      */
} MotorDirection;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialize the motor driver hardware.
 *
 * Sets up direction pins as outputs and activates the STBY pin.
 * Must be called after pwm_init() since motors need PWM to function.
 *
 * TODO: Implement direction pin initialization via pinMode().
 * TODO: Set STBY pin HIGH to enable the motor driver.
 */
void motor_init(void);

/**
 * @brief Set the speed of a specific motor.
 *
 * Positive values = forward, negative values = reverse, zero = brake.
 * The PWM value is clamped to [-PWM_MAX, PWM_MAX].
 *
 * @param motor Motor identifier (MOTOR_LEFT or MOTOR_RIGHT)
 * @param pwm   Signed PWM value (-PWM_MAX to PWM_MAX)
 *
 * TODO: Implement direction setting based on sign of pwm.
 * TODO: Implement PWM duty cycle setting via TIM1 CCR registers.
 */
void motor_set_speed(MotorID motor, int16_t pwm);

/**
 * @brief Set the direction of a specific motor.
 *
 * @param motor Motor identifier
 * @param dir   Desired direction
 *
 * TODO: Implement direction pin logic for TB6612FNG:
 *       FORWARD: IN1=HIGH, IN2=LOW
 *       REVERSE: IN1=LOW,  IN2=HIGH
 *       BRAKE:   IN1=HIGH, IN2=HIGH
 *       COAST:   IN1=LOW,  IN2=LOW
 */
void motor_set_direction(MotorID motor, MotorDirection dir);

/**
 * @brief Set both motors simultaneously.
 *
 * Positive = forward, negative = reverse.
 *
 * @param left_pwm  Left motor PWM  (-PWM_MAX to PWM_MAX)
 * @param right_pwm Right motor PWM (-PWM_MAX to PWM_MAX)
 *
 * TODO: Implement as calls to motor_set_speed() for each motor.
 */
void motor_set_both(int16_t left_pwm, int16_t right_pwm);

/**
 * @brief Stop both motors with active braking.
 *
 * TODO: Implement by setting both motors to BRAKE and PWM to 0.
 */
void motor_stop(void);

/**
 * @brief Enable or disable the motor driver (STBY pin).
 *
 * @param enable true = motors active, false = motors in standby
 *
 * TODO: Implement STBY pin control.
 */
void motor_enable(bool enable);

/**
 * @brief Drive the robot forward at given PWM.
 *
 * @param pwm PWM value (0 to PWM_MAX)
 *
 * TODO: Implement as motor_set_both(pwm, pwm).
 */
void motor_forward(uint16_t pwm);

/**
 * @brief Drive the robot in reverse at given PWM.
 *
 * @param pwm PWM value (0 to PWM_MAX)
 *
 * TODO: Implement as motor_set_both(-pwm, -pwm).
 */
void motor_reverse(uint16_t pwm);

/**
 * @brief Spin the robot left (in-place turn).
 *
 * @param pwm PWM value (0 to PWM_MAX)
 *
 * TODO: Implement as motor_set_both(-pwm, pwm).
 */
void motor_turn_left(uint16_t pwm);

/**
 * @brief Spin the robot right (in-place turn).
 *
 * @param pwm PWM value (0 to PWM_MAX)
 *
 * TODO: Implement as motor_set_both(pwm, -pwm).
 */
void motor_turn_right(uint16_t pwm);

#endif /* MOTOR_H */
