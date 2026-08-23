/**
 * @file motor.cpp
 * @brief Motor driver implementation for the TB6612FNG.
 *
 * Implements motor control functions using GPIO direction pins and
 * TIM1 PWM for speed control. Matches register-level patterns from
 * the existing test code.
 *
 * @see motor.h for public API documentation
 */

#include "motor.h"
#include "pwm.h"
#include "gpio.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private State
 * ═══════════════════════════════════════════════════════════════════════════ */

/* No additional private state needed — direction is set via GPIO on each call */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialization
 * ═══════════════════════════════════════════════════════════════════════════ */

void motor_init(void) {
    gpio_init_motor_pins();
    gpio_set_standby(true);
    motor_stop();
}

void motor_set_direction(MotorID motor, MotorDirection dir) {
    uint8_t in1 = LOW;
    uint8_t in2 = LOW;

    switch (dir) {
        case MOTOR_DIR_FORWARD:
            in1 = HIGH; in2 = LOW;
            break;
        case MOTOR_DIR_REVERSE:
            in1 = LOW;  in2 = HIGH;
            break;
        case MOTOR_DIR_BRAKE:
            in1 = HIGH; in2 = HIGH;
            break;
        case MOTOR_DIR_COAST:
            in1 = LOW;  in2 = LOW;
            break;
    }

    if (motor == MOTOR_LEFT) {
        gpio_set_left_direction(in1, in2);
    } else {
        gpio_set_right_direction(in1, in2);
    }
}

void motor_set_speed(MotorID motor, int16_t pwm) {
    // 1. Clamp PWM to safe maximum limits [-PWM_MAX, PWM_MAX]
    if (pwm > (int16_t)PWM_MAX)  pwm = PWM_MAX;
    if (pwm < -(int16_t)PWM_MAX) pwm = -((int16_t)PWM_MAX);

    // 2. Set H-bridge direction and apply unsigned PWM duty cycle
    if (pwm > 0) {
        motor_set_direction(motor, MOTOR_DIR_FORWARD);
        if (motor == MOTOR_LEFT) pwm_set_left((uint16_t)pwm);
        else                     pwm_set_right((uint16_t)pwm);
    } else if (pwm < 0) {
        motor_set_direction(motor, MOTOR_DIR_REVERSE);
        if (motor == MOTOR_LEFT) pwm_set_left((uint16_t)(-pwm));
        else                     pwm_set_right((uint16_t)(-pwm));
    } else {
        motor_set_direction(motor, MOTOR_DIR_BRAKE);
        if (motor == MOTOR_LEFT) pwm_set_left(0);
        else                     pwm_set_right(0);
    }
}

void motor_set_both(int16_t left_pwm, int16_t right_pwm) {
    motor_set_speed(MOTOR_LEFT, left_pwm);
    motor_set_speed(MOTOR_RIGHT, right_pwm);
}

void motor_stop(void) {
    motor_set_speed(MOTOR_LEFT, 0);
    motor_set_speed(MOTOR_RIGHT, 0);
}

void motor_enable(bool enable) {
    gpio_set_standby(enable);
}

void motor_forward(uint16_t pwm) {
    motor_set_both((int16_t)pwm, (int16_t)pwm);
}

void motor_reverse(uint16_t pwm) {
    motor_set_both(-((int16_t)pwm), -((int16_t)pwm));
}

void motor_turn_left(uint16_t pwm) {
    motor_set_both(-((int16_t)pwm), (int16_t)pwm);
}

void motor_turn_right(uint16_t pwm) {
    motor_set_both((int16_t)pwm, -((int16_t)pwm));
}
