/**
 * @file gpio.cpp
 * @brief GPIO implementation for motor direction pin control.
 *
 * @see gpio.h for public API documentation
 */

#include "gpio.h"

void gpio_init_motor_pins(void) {
    /**
     * TODO: Configure all motor direction pins as OUTPUT:
     *   pinMode(PIN_MOTOR_LEFT_IN1, OUTPUT);
     *   pinMode(PIN_MOTOR_LEFT_IN2, OUTPUT);
     *   pinMode(PIN_MOTOR_RIGHT_IN1, OUTPUT);
     *   pinMode(PIN_MOTOR_RIGHT_IN2, OUTPUT);
     *   pinMode(PIN_MOTOR_STBY, OUTPUT);
     *
     * TODO: Set STBY HIGH to enable the driver.
     * TODO: Set all direction pins LOW initially (coast mode).
     */
}

void gpio_set_standby(bool enable) {
    /**
     * TODO: digitalWrite(PIN_MOTOR_STBY, enable ? HIGH : LOW);
     */
}

void gpio_set_left_direction(uint8_t in1, uint8_t in2) {
    /**
     * TODO: digitalWrite(PIN_MOTOR_LEFT_IN1, in1);
     * TODO: digitalWrite(PIN_MOTOR_LEFT_IN2, in2);
     */
}

void gpio_set_right_direction(uint8_t in1, uint8_t in2) {
    /**
     * TODO: digitalWrite(PIN_MOTOR_RIGHT_IN1, in1);
     * TODO: digitalWrite(PIN_MOTOR_RIGHT_IN2, in2);
     */
}
