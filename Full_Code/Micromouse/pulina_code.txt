/**
 * @file pulina_code.ino
 * @brief Simple forward movement and sensor reading script.
 */

#include <Arduino.h>
#include <Wire.h>

#include "src/config/pin_config.h"
#include "src/config/robot_config.h"

// Hardware
#include "src/hardware/battery.h"
#include "src/hardware/button.h"
#include "src/hardware/encoder.h"
#include "src/hardware/gpio.h"
#include "src/hardware/led.h"
#include "src/hardware/motor.h"
#include "src/hardware/pwm.h"
#include "src/hardware/timer.h"

// Sensors
#include "src/sensors/calibration.h"
#include "src/sensors/distance_manager.h"
#include "src/sensors/mpu6050.h"
#include "src/sensors/sensor_manager.h"
// #include "src/sensors/sensor_fusion.h" // Commented out sensor fusion

// Localization (Commented out)
// #include "src/localization/odometry.h"
// #include "src/localization/position_estimator.h"

// Control (Commented out - PID Parts)
// #include "src/control/motion_controller.h"
// #include "src/control/velocity_controller.h"
// #include "src/control/heading_controller.h"
// #include "src/control/speed_controller.h"

// Robot (Commented out)
// #include "src/robot/mission_manager.h"
// #include "src/robot/robot_state_machine.h"

// Display
#include "src/display/menu.h"
#include "src/display/oled_driver.h"
#include "src/display/status_screen.h"

// Utils
#include "src/utils/logger.h"
#include "src/utils/serial_debug.h"

void setup() {
  Serial.begin(115200);
  delay(100);
  LOG_INFO("Pulina Code Booting...");

  // 1. Hardware Initialization
  gpio_init_motor_pins();
  pwm_init();
  encoder_init();
  motor_init();
  button_init();
  led_init();
  battery_init();

  // 2. Display Initialization
  Wire.setSCL(PIN_I2C_SCL);
  Wire.setSDA(PIN_I2C_SDA);
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C
  
  if (oled_init()) {
    oled_clear();
    oled_print(0, 0, "Pulina Code");
    oled_update();
  } else {
    LOG_ERROR("OLED init failed!");
  }

  // 3. Sensor Initialization (VL53L0X & MPU6050)
  sensor_manager_init();

  LOG_INFO("Calibrating Gyro - Keep Robot Still");
  calibrate_all();

  // 4. Software Modules Initialization (PID and FSM commented out)
  // odometry_init();
  // fusion_init();
  // motion_controller_init();
  // mission_manager_init();
  // menu_init();
  // fsm_init();

  // 5. Start Control Loop Interrupt (Commented out PID Loop)
  // timer_init(motion_controller_update);
  // timer_start();

  // 7. Wait for Start Button (from plan)
  LOG_INFO("Press BTN_START to begin exploration.");
  // Wait until button is pressed before engaging exploration
  // while(button_read() == false) { delay(10); }
  // delay(500);
  
  LOG_INFO("Moving Forward!");
  
  // Move forward at a favorable speed (PWM 1500)
  motor_forward(1500);
}

void loop() {
  // Read user inputs
  button_update();
  serial_debug_update();

  // Update slow sensors (I2C ToF)
  sensor_manager_update();
  
  // Get Distance Readings
  uint16_t dist_f = distance_get_mm(TOF_FRONT);
  uint16_t dist_fl = distance_get_mm(TOF_FRONT_LEFT);
  uint16_t dist_fr = distance_get_mm(TOF_FRONT_RIGHT);
  uint16_t dist_l = distance_get_mm(TOF_LEFT);
  uint16_t dist_r = distance_get_mm(TOF_RIGHT);

  // Stop if an obstacle is within 15cm (150mm)
  if (dist_f <= 150 && dist_f > 0) { // dist_f > 0 to ignore false 0 readings
    motor_stop();
  }

  // Get IMU Readings
  IMUScaledData imu;
  mpu6050_read_scaled(&imu);
  
  // Print sensor readings to Serial and OLED
  static uint32_t last_print = 0;
  if (millis() - last_print >= 100) { // Print at 10Hz
    last_print = millis();
    
    // Calculate speed (runs every 100ms, so multiply delta by 10 for counts per second)
    int32_t l_delta = encoder_get_delta(ENCODER_LEFT);
    int32_t r_delta = encoder_get_delta(ENCODER_RIGHT);
    float l_speed = encoder_counts_to_speed(l_delta * 10.0f);
    float r_speed = encoder_counts_to_speed(r_delta * 10.0f);
    
    Serial.print("Dist_F:"); Serial.print(dist_f);
    Serial.print(" | L_Spd:"); Serial.print(l_speed, 1);
    Serial.print(" | R_Spd:"); Serial.println(r_speed, 1);
    
    // Update OLED Display
    char buf[32];
    oled_clear();
    oled_print(0, 0, "Dist:");
    
    sprintf(buf, "%u mm", dist_f);
    oled_print(40, 0, buf);
    
    sprintf(buf, "Spd L: %.1f mm/s", l_speed);
    oled_print(0, 20, buf);
    
    sprintf(buf, "Spd R: %.1f mm/s", r_speed);
    oled_print(0, 40, buf);
    
    oled_update();
  }

  // PID and High-level state machine (Commented out)
  // fsm_update();
  // if (fsm_get_state() == STATE_IDLE) {
  //   if (!menu_update()) {
  //     status_screen_draw_main();
  //   }
  // }

  delay(5);
}
