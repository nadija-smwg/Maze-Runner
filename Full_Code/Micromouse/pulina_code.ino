/**
 * @file pulina_code.ino
 * @brief Wall-Following PD Control Script (No Encoders/IMU)
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

// Control
#include "src/control/wall_follower.h"

// Display
#include "src/display/oled_driver.h"

// Utils
#include "src/utils/logger.h"
#include "src/utils/serial_debug.h"

// State variables
static bool is_driving = false;
static uint32_t last_loop_time = 0;
static uint32_t mode_press_time = 0;

void setup() {
  Serial.begin(115200);
  delay(100);
  LOG_INFO("Pulina Code - Wall Follower Booting...");

  // 1. Hardware Initialization
  gpio_init_motor_pins();
  pwm_init();
  encoder_init(); // Still init encoders just for debug reading
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
    oled_print(0, 0, "Wall Follower");
    oled_print(0, 20, "Booting...");
    oled_update();
  } else {
    LOG_ERROR("OLED init failed!");
  }

  // 3. Sensor & Control Initialization
  sensor_manager_init();
  wall_follower_init();
  
  // Optional: Calibrate IMU if you plan to use it for turning later
  LOG_INFO("Calibrating Gyro - Keep Robot Still");
  calibrate_all();

  // Reset timing
  last_loop_time = micros();

  LOG_INFO("Ready! Press BTN_START to drive.");
  
  oled_clear();
  oled_print(0, 0, "READY");
  oled_print(0, 20, "Press START");
  oled_update();
}

void loop() {
  // Read user inputs
  button_update();
  led_update();
  
  uint32_t now = micros();
  float dt = (now - last_loop_time) / 1000000.0f;
  last_loop_time = now;
  if (dt <= 0.0f) dt = 0.01f;
  if (dt > 0.1f) dt = 0.1f; // Clamp to 100ms max

  // ────────────────────────────────────────────────────────────
  //  Button Logic
  // ────────────────────────────────────────────────────────────
  if (button_just_pressed(BUTTON_START)) {
    is_driving = !is_driving;
    if (is_driving) {
      wall_follower_init(); // Reset errors
      Serial.println("\n[STATE] DRIVE");
    } else {
      motor_stop();
      Serial.println("\n[STATE] IDLE (Stopped)");
    }
  }

  // Use MODE button to cycle KP for live tuning
  if (button_just_pressed(BUTTON_MODE)) {
    if (!is_driving) {
      float kp = wall_follower_get_kp();
      kp += 1.0f;
      if (kp > 15.0f) kp = 1.0f;
      wall_follower_set_kp(kp);
      Serial.print("[TUNE] Wall Follow KP -> ");
      Serial.println(kp);
    }
  }

  // ────────────────────────────────────────────────────────────
  //  Sensor Update (~100Hz)
  // ────────────────────────────────────────────────────────────
  static uint32_t last_sensor_tick = 0;
  if (millis() - last_sensor_tick >= 10) {
    last_sensor_tick = millis();
    sensor_manager_update(); // Updates all ToF distances
  }

  // Get current readings
  uint16_t dist_f = distance_get_mm(TOF_FRONT);
  uint16_t dist_l = distance_get_mm(TOF_LEFT);
  uint16_t dist_r = distance_get_mm(TOF_RIGHT);
  float lat_error = distance_get_centering_error();

  // ────────────────────────────────────────────────────────────
  //  Motion Control
  // ────────────────────────────────────────────────────────────
  if (is_driving) {
    // 1. Safety / Maze logic: Stop if wall is in front
    if (dist_f <= 150 && dist_f > 0 && dist_f != 8190) { 
      motor_stop();
      is_driving = false;
      Serial.println("[STATE] FRONT WALL HIT -> STOPPED");
    } 
    else {
      // 2. Drive forward and stay centered
      wall_follower_update(lat_error, dt);
    }
  } else {
    motor_stop();
  }

  // ────────────────────────────────────────────────────────────
  //  OLED & Serial Debug Output (10Hz)
  // ────────────────────────────────────────────────────────────
  static uint32_t last_print = 0;
  if (millis() - last_print >= 100) { 
    last_print = millis();
    
    int16_t correction = wall_follower_get_last_correction();
    int16_t pwm_l = is_driving ? WALL_FOLLOW_BASE_PWM_LEFT + correction : 0;
    int16_t pwm_r = is_driving ? WALL_FOLLOW_BASE_PWM_RIGHT - correction : 0;
    
    // OLED
    char buf[32];
    oled_clear();
    
    if (is_driving) oled_print(0, 0, "STATE: DRIVE");
    else oled_print(0, 0, "STATE: IDLE");
    
    sprintf(buf, "F:%u L:%u R:%u", dist_f, dist_l, dist_r);
    oled_print(0, 15, buf);
    
    sprintf(buf, "Err: %d mm", (int)lat_error);
    oled_print(0, 30, buf);
    
    sprintf(buf, "PWM L:%d R:%d", pwm_l, pwm_r);
    oled_print(0, 45, buf);
    
    sprintf(buf, "KP: %d", (int)wall_follower_get_kp());
    oled_print(80, 45, buf);
    
    oled_update();
    
    // Serial (only spam while driving)
    if (is_driving) {
      Serial.print("F:"); Serial.print(dist_f);
      Serial.print(" | L:"); Serial.print(dist_l);
      Serial.print(" | R:"); Serial.print(dist_r);
      Serial.print(" | Err:"); Serial.print(lat_error, 1);
      Serial.print(" | PWM_L:"); Serial.print(pwm_l);
      Serial.print(" | PWM_R:"); Serial.println(pwm_r);
    }
  }

  delay(1); // Small delay to prevent tight loop burning
}

