/**
 * @file Micromouse.ino
 * @brief Main Arduino sketch for the Micromouse.
 *
 * This is the main entry point for the Arduino framework. It initializes
 * all hardware and software modules, then hands over execution to the
 * robot state machine in the main loop.
 */

#include <Arduino.h>
#include <Wire.h>

#include "config/pin_config.h"
#include "config/robot_config.h"

// Hardware
#include "hardware/timer.h"
#include "hardware/motor.h"
#include "hardware/encoder.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/battery.h"
#include "hardware/button.h"
#include "hardware/buzzer.h"
#include "hardware/led.h"

// Sensors
#include "sensors/sensor_manager.h"
#include "sensors/sensor_fusion.h"
#include "sensors/calibration.h"

// Localization
#include "localization/odometry.h"
#include "localization/position_estimator.h"

// Control
#include "control/motion_controller.h"

// Robot
#include "robot/robot_state_machine.h"
#include "robot/mission_manager.h"

// Display
#include "display/oled_driver.h"
#include "display/menu.h"
#include "display/status_screen.h"

// Utils
#include "utils/serial_debug.h"
#include "utils/logger.h"

void setup() {
    Serial.begin(115200);
    delay(100);
    LOG_INFO("Micromouse Booting...");

    // 1. Hardware Initialization
    gpio_init_motor_pins();
    pwm_init();
    encoder_init();
    motor_init();
    button_init();
    led_init();
    buzzer_init();
    battery_init();

    // 2. Display Initialization
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C
    if (oled_init()) {
        oled_clear();
        oled_print(0, 0, "Micromouse v1.0");
        oled_update();
    } else {
        LOG_ERROR("OLED init failed!");
    }

    // 3. Sensor Initialization
    sensor_manager_init();

    // 4. Software Modules Initialization
    odometry_init();
    fusion_init();
    motion_controller_init();
    mission_manager_init();
    menu_init();
    fsm_init();

    // 5. Start Control Loop Interrupt
    timer_init(motion_controller_update);
    timer_start();

    // 6. Enter Idle State
    fsm_set_state(STATE_IDLE);
    buzzer_play_startup();
    LOG_INFO("Boot Complete. Entering Idle.");
}

void loop() {
    // Read user inputs
    button_update();
    serial_debug_update();

    // Update slow sensors (I2C ToF)
    sensor_manager_update();

    // Run high-level state machine
    fsm_update();

    // Update Display based on state
    if (fsm_get_state() == STATE_IDLE) {
        if (!menu_update()) {
            status_screen_draw_main();
        }
    }

    // Small delay to prevent tight loop lockup if needed
    delay(1);
}
