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
 * @see gpio.h for pin abstraction
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
 * Calls gpio_init_motor_pins(), gpio_set_standby(true), and motor_stop().
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
 * Sets direction via gpio and duty via TIM1 CCR based on sign of pwm.
 */
void motor_set_speed(MotorID motor, int16_t pwm);

/**
 * @brief Set the direction of a specific motor.
 *
 * @param motor Motor identifier
 * @param dir   Desired direction
 *
 * Direction pin truth table for TB6612FNG:
 *   FORWARD: IN1=HIGH, IN2=LOW
 *   REVERSE: IN1=LOW,  IN2=HIGH
 *   BRAKE:   IN1=HIGH, IN2=HIGH
 *   COAST:   IN1=LOW,  IN2=LOW
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
 * Delegates to motor_set_speed() for each motor.
 */
void motor_set_both(int16_t left_pwm, int16_t right_pwm);

/**
 * @brief Stop both motors with active braking.
 *
 * Sets both motors to BRAKE direction with PWM=0.
 */
void motor_stop(void);

/**
 * @brief Enable or disable the motor driver (STBY pin).
 *
 * @param enable true = motors active, false = motors in standby
 *
 * Controls STBY pin via gpio_set_standby().
 */
void motor_enable(bool enable);

/**
 * @brief Drive the robot forward at given PWM.
 *
 * @param pwm PWM value (0 to PWM_MAX)
 *
 * Calls motor_set_both(pwm, pwm).
 */
void motor_forward(uint16_t pwm);

/**
 * @brief Drive the robot in reverse at given PWM.
 *
 * @param pwm PWM value (0 to PWM_MAX)
 *
 * Calls motor_set_both(-pwm, -pwm).
 */
void motor_reverse(uint16_t pwm);

/**
 * @brief Spin the robot left (in-place turn).
 *
 * @param pwm PWM value (0 to PWM_MAX)
 *
 * Calls motor_set_both(-pwm, pwm).
 */
void motor_turn_left(uint16_t pwm);

/**
 * @brief Spin the robot right (in-place turn).
 *
 * @param pwm PWM value (0 to PWM_MAX)
 *
 * Calls motor_set_both(pwm, -pwm).
 */
void motor_turn_right(uint16_t pwm);

/* ═══════════════════════════════════════════════════════════════════════════
 *  Dead-zone Compensation & Diagnostics
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Set motor speed with dead-zone compensation.
 *
 * Adds LEFT_MOTOR_DEAD_PWM / RIGHT_MOTOR_DEAD_PWM (from robot_config.h)
 * in the direction of motion so the motor overcomes static friction
 * immediately. Zero input still brakes cleanly.
 *
 * Use this instead of motor_set_speed() when the velocity controller
 * is providing a corrective output on top of feed-forward.
 *
 * @param motor Motor identifier
 * @param pwm   Signed corrective PWM (-PWM_MAX to PWM_MAX). 0 = brake.
 */
void motor_set_speed_compensated(MotorID motor, int16_t pwm);

/**
 * @brief Check if a motor is stalled.
 *
 * Returns true when the commanded PWM is high but the measured speed
 * is near zero — indicating a blocked wheel or disconnected encoder.
 *
 * @param motor   Motor identifier
 * @param cmd_pwm Absolute magnitude of currently commanded PWM
 * @return        true if stall detected
 *
 * Thresholds:
 *   PWM must exceed 800 (STALL_PWM_THRESHOLD)
 *   Speed must be below 20 mm/s (STALL_SPEED_THRESHOLD_MMS)
 */
bool motor_is_stalled(MotorID motor, int16_t cmd_pwm);

#endif /* MOTOR_H */
