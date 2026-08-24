/**
 * @file gpio.cpp
 * @brief GPIO implementation for motor direction pin control.
 *
 * @see gpio.h for public API documentation
 */

#include "gpio.h"

void gpio_init_motor_pins(void) {
    // 1. Set all motor direction and standby pins as digital outputs
    pinMode(PIN_MOTOR_LEFT_IN1, OUTPUT);
    pinMode(PIN_MOTOR_LEFT_IN2, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT_IN1, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT_IN2, OUTPUT);
    pinMode(PIN_MOTOR_STBY, OUTPUT);

    // 2. Enable the TB6612FNG motor driver by pulling STBY HIGH
    gpio_set_standby(true);

    // 3. Set both motors to coast mode initially (IN1 = LOW, IN2 = LOW)
    gpio_set_left_direction(LOW, LOW);
    gpio_set_right_direction(LOW, LOW);
}

void gpio_set_standby(bool enable) {
    // HIGH enables the H-bridges, LOW puts them in low-power standby mode
    digitalWrite(PIN_MOTOR_STBY, enable ? HIGH : LOW);
}

void gpio_set_left_direction(uint8_t in1, uint8_t in2) {
    digitalWrite(PIN_MOTOR_LEFT_IN1, in1);
    digitalWrite(PIN_MOTOR_LEFT_IN2, in2);
}

void gpio_set_right_direction(uint8_t in1, uint8_t in2) {
    digitalWrite(PIN_MOTOR_RIGHT_IN1, in1);
    digitalWrite(PIN_MOTOR_RIGHT_IN2, in2);
}
