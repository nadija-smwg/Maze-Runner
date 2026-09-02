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
#include "src/sensors/sensor_fusion.h"
#include "src/sensors/sensor_manager.h"
#include "src/sensors/distance_manager.h"
#include "src/sensors/mpu6050.h"

// Localization
#include "src/localization/odometry.h"
#include "src/localization/position_estimator.h"

// Control
#include "src/control/motion_controller.h"

// Robot
#include "src/robot/mission_manager.h"
#include "src/robot/robot_state_machine.h"

// Display
#include "src/display/menu.h"
#include "src/display/oled_driver.h"
#include "src/display/status_screen.h"

// Utils
#include "src/utils/logger.h"
#include "src/utils/serial_debug.h"

// Phase testing mode flags (set to 1 for active test mode)
#define PHASE_1_TEST_MODE 0
#define PHASE_2_TEST_MODE 1
#define PHASE_3_TEST_MODE 0

#if PHASE_1_TEST_MODE == 1
volatile uint32_t phase1_timer_ticks = 0;
void phase1_timer_callback(void) { phase1_timer_ticks++; }
#endif

#if PHASE_2_TEST_MODE == 1
volatile uint32_t phase2_timer_ticks = 0;
void phase2_timer_callback(void) { phase2_timer_ticks++; }
#endif

#if PHASE_3_TEST_MODE == 1
volatile uint32_t phase3_timer_ticks = 0;
void phase3_timer_callback(void) { phase3_timer_ticks++; }
#endif

void setup() {
  Serial.begin(115200);
  delay(100);
  LOG_INFO("Micromouse Booting...");

#if PHASE_1_TEST_MODE == 1
  LOG_INFO("=== PHASE 1 HARDWARE TEST MODE ===");
  gpio_init_motor_pins();
  button_init();
  led_init();
  battery_init();

  Wire.setSCL(PIN_I2C_SCL);
  Wire.setSDA(PIN_I2C_SDA);
  Wire.begin();
  Wire.setClock(400000);
  if (oled_init()) {
      oled_clear();
      oled_print(0, 0, "Phase 1 Test Mode");
      oled_update();
  } else {
      LOG_ERROR("OLED Init Failed");
  }

  uint16_t v_mv = battery_get_voltage_mv();
  uint8_t v_pct = battery_get_percentage();
  Serial.print("Battery Voltage: ");
  Serial.print(v_mv);
  Serial.print(" mV (");
  Serial.print(v_pct);
  Serial.println("%)");

  // Test non-blocking LED blinking on status LED (250ms interval)
  led_blink(LED_STATUS, 250);

  // Initialize timer with 1kHz test callback
  timer_init(phase1_timer_callback);
  timer_start();
  LOG_INFO("Press BTN_START to test debug LED, BTN_MODE to toggle status LED.");
  return;
#endif

#if PHASE_2_TEST_MODE == 1
  LOG_INFO("=== PHASE 2 ACTUATION & ENCODER TEST MODE ===");
  gpio_init_motor_pins();
  pwm_init();
  encoder_init();
  motor_init();
  button_init();
  led_init();
  battery_init();

  Wire.setSCL(PIN_I2C_SCL);
  Wire.setSDA(PIN_I2C_SDA);
  Wire.begin();
  Wire.setClock(400000);
  if (oled_init()) {
      oled_clear();
      oled_print(0, 0, "Phase 2 Test Mode");
      oled_update();
  } else {
      LOG_ERROR("OLED Init Failed");
  }

  // Reset encoders to zero
  encoder_reset_all();

  // Start 1kHz control loop timer
  timer_init(phase2_timer_callback);
  timer_start();

  LOG_INFO("Phase 2 Ready!");
  LOG_INFO(" - Press BTN_START to cycle motor states (Stop -> Fwd -> Rev -> "
           "TurnL -> TurnR -> Stop).");
  LOG_INFO(" - Press BTN_MODE to zero/reset encoders.");
  return;
#endif

#if PHASE_3_TEST_MODE == 1
  LOG_INFO("=== PHASE 3 SENSING TEST MODE ===");
  button_init();
  led_init();
  battery_init();

  Wire.setSCL(PIN_I2C_SCL);
  Wire.setSDA(PIN_I2C_SDA);
  Wire.begin();
  Wire.setClock(400000);
  if (oled_init()) {
      oled_clear();
      oled_print(0, 0, "Phase 3 Booting");
      oled_update();
  } else {
      LOG_ERROR("OLED Init Failed");
  }

  // Initialize Sensors
  sensor_manager_init();

  // Calibrate Gyro
  calibrate_all();

  // Start 1kHz control loop timer
  timer_init(phase3_timer_callback);
  timer_start();

  LOG_INFO("Phase 3 Ready!");
  LOG_INFO(" - BTN_START: Re-calibrate Gyro");
  LOG_INFO(" - BTN_MODE: Toggle Debug LED");
  return;
#endif


  // 1. Hardware Initialization (Motors, Pins, Encoders, Battery, etc.)
  gpio_init_motor_pins();
  pwm_init();
  encoder_init();
  motor_init();
  button_init();
  led_init();
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

  // 3. Sensor Initialization (VL53L0X & MPU6050)
  // Note: XSHUT pins are toggled inside sensor_manager_init()
  // to change I2C addresses dynamically.
  sensor_manager_init();

  /*
   * Competition Boot Sequence:
   *   1. 90-second warm-up countdown on OLED (sensor reaches stable temp)
   *   2. Auto-calibrate at end of warm-up
   *   3. OLED shows "READY" → press BTN_START to begin run
   *
   * Press BTN_START during countdown to skip warm-up and calibrate now.
   * (Use this if the robot was already warm from a previous run.)
   */
  calibrate_with_warmup(90);

  // Hardware I2C Watchdog Setup (from plan)
  // setupWatchdog();

  // 4. Software Modules Initialization
  odometry_init();
  fusion_init();
  motion_controller_init();
  mission_manager_init();
  menu_init();
  fsm_init();

  // 5. Maze Array Initialization (from plan)
  /*
  memset(walls, 0, sizeof(walls));
  memset(visited, 0, sizeof(visited));
  for (int i = 0; i < MAZE_SIZE; i++) {
      walls[0][i]           |= 8;  // West border
      walls[MAZE_SIZE-1][i] |= 2;  // East border
      walls[i][0]           |= 4;  // South border
      walls[i][MAZE_SIZE-1] |= 1;  // North border
  }
  floodFill();
  */

  // 6. Start Control Loop Interrupt
  timer_init(motion_controller_update);
  timer_start();

  // 7. Wait for Start Button (from plan)
  LOG_INFO("Press BTN_START to begin exploration.");
  // Wait until button is pressed before engaging exploration
  // while(button_read() == false) { delay(10); }
  // delay(500);

  // 8. Enter Initial State
  fsm_set_state(STATE_IDLE);
  LOG_INFO("Boot Complete. Entering Idle/Exploring.");
}

void loop() {
#if PHASE_1_TEST_MODE == 1
  button_update();
  led_update();

  // Test button 0 (BUTTON_START)
  if (button_just_pressed(BUTTON_START)) {
    LOG_INFO("BTN_START Pressed! Toggling debug LED.");
    led_toggle(LED_DEBUG);
  }

  // Test button 1 (BUTTON_MODE)
  if (button_just_pressed(BUTTON_MODE)) {
    LOG_INFO("BTN_MODE Pressed! Toggling status LED.");
    led_toggle(LED_STATUS);
  }

  // Print timer heartbeat and battery status every 1000 ticks (~1 second)
  static uint32_t last_print_ticks = 0;
  if ((phase1_timer_ticks - last_print_ticks) >= 1000) {
    last_print_ticks = phase1_timer_ticks;
    
    char buf[32];
    oled_clear();
    oled_print(0, 0, "Phase 1 Test");
    sprintf(buf, "Ticks: %lu", phase1_timer_ticks);
    oled_print(0, 16, buf);
    sprintf(buf, "Bat: %u mV", battery_get_voltage_mv());
    oled_print(0, 32, buf);
    oled_update();

    Serial.print("[Heartbeat 1Hz] Timer Ticks: ");
    Serial.print(phase1_timer_ticks);
    Serial.print(" | Battery: ");
    Serial.print(battery_get_voltage_mv());
    Serial.println(" mV");
  }

  delay(5);
  return;
#endif

#if PHASE_2_TEST_MODE == 1
  button_update();
  led_update();

  static int      test_state  = 0;
  static uint16_t dz_pwm      = 400;   /* start above likely dead-zone */
  static uint32_t last_dz     = 0;
  static bool     dz_settled  = false; /* skip first read — motor hasn't moved yet */
  static int32_t  dz_count_prev = 0;  /* raw encoder count for motion detection */

  /*
   * BTN_START — cycle states 0-8 (9 states total)
   *   0  Stop
   *   1  Forward  PWM 1500
   *   2  Reverse  PWM 1500
   *   3  Turn Left  PWM 1500
   *   4  Turn Right PWM 1500
   *   5  Dead-zone sweep LEFT  (Test 3.3)
   *   6  Dead-zone sweep RIGHT (Test 3.3)
   *   7  CPR verify LEFT       (Test F.1)
   *   8  CPR verify RIGHT      (Test F.1)
   */
  if (button_just_pressed(BUTTON_START)) {
    test_state = (test_state + 1) % 9;
    motor_stop();   /* always safe-stop on every state change */

    switch (test_state) {
    /* ── Basic drive states ─────────────────────────────────── */
    case 0:
      LOG_INFO("[State 0] Motors STOPPED.");
      break;
    case 1:
      motor_forward(500);
      LOG_INFO("[State 1] FORWARD PWM 1500.");
      break;
    case 2:
      motor_reverse(600);
      LOG_INFO("[State 2] REVERSE PWM 1500.");
      break;
    case 3:
      motor_turn_left(1500);
      LOG_INFO("[State 3] TURN LEFT PWM 1500.");
      break;
    case 4:
      motor_turn_right(1500);
      LOG_INFO("[State 4] TURN RIGHT PWM 1500.");
      break;

    /* ── Dead-zone sweep ────────────────────────────────────── */
    case 5:
      Serial.println(F("\n=== TEST 3.3: DEAD-ZONE SWEEP (LEFT) ==="));
      Serial.println(F("Sweeping LEFT motor PWM 200->2500. Hold robot safely!"));
      oled_clear();
      oled_print(0,  0, "DZ: LEFT motor");
      oled_print(0, 15, "Sweeping...");
      oled_update();

      motor_stop();
      delay(300);           /* let motor fully brake before sweep */
      encoder_reset_all();  /* zero counts from a clean stop     */

      {
        bool dz_found = false;
        for (uint16_t pwm = 200; pwm <= 2500 && !dz_found; pwm += 50) {
          int32_t cnt_before = encoder_get_count(ENCODER_LEFT); /* snapshot BEFORE */
          motor_set_speed(MOTOR_LEFT,  (int16_t)pwm);
          motor_set_speed(MOTOR_RIGHT, 0);
          delay(400);  /* settle time */
          int32_t cnt_after = encoder_get_count(ENCODER_LEFT);

          int32_t diff = cnt_after - cnt_before;
          if (diff < 0) diff = -diff;

          Serial.print(F("LEFT PWM:")); Serial.print(pwm);
          Serial.print(F("  counts:")); Serial.print(diff);

          if (diff >= 3) {
            Serial.println(F("  <<< MOVING!"));
            Serial.print(F(">>> Set LEFT_MOTOR_DEAD_PWM = ")); Serial.println(pwm);
            char b[22];
            oled_clear();
            oled_print(0,  0, "LEFT: FOUND!");
            sprintf(b, "PWM = %u", pwm);
            oled_print(0, 20, b);
            sprintf(b, "Cnt diff = %ld", (long)diff);
            oled_print(0, 36, b);
            oled_print(0, 52, "Press START->next");
            oled_update();
            dz_found = true;
          } else {
            Serial.println(F("  (not moving)"));
          }

          motor_stop();  /* stop between steps for clean measurement */
          delay(100);
        }
        if (!dz_found) {
          Serial.println(F("!!! Motor did NOT move up to PWM 2500 !!!"));
          Serial.println(F("Check wiring, power, and motor connections."));
          oled_clear();
          oled_print(0,  0, "LEFT: NOT FOUND");
          oled_print(0, 20, "Check wiring!");
          oled_update();
        }
      }
      motor_stop();
      /* stay in state 5 so user can read result — press BTN_START to continue */
      break;

    case 6:
      Serial.println(F("\n=== TEST 3.3: DEAD-ZONE SWEEP (RIGHT) ==="));
      Serial.println(F("Sweeping RIGHT motor PWM 200->2500. Hold robot safely!"));
      oled_clear();
      oled_print(0,  0, "DZ: RIGHT motor");
      oled_print(0, 15, "Sweeping...");
      oled_update();

      motor_stop();
      delay(300);
      encoder_reset_all();

      {
        bool dz_found = false;
        for (uint16_t pwm = 200; pwm <= 2500 && !dz_found; pwm += 50) {
          int32_t cnt_before = encoder_get_count(ENCODER_RIGHT);
          motor_set_speed(MOTOR_LEFT,  0);
          motor_set_speed(MOTOR_RIGHT, (int16_t)pwm);
          delay(400);
          int32_t cnt_after = encoder_get_count(ENCODER_RIGHT);

          int32_t diff = cnt_after - cnt_before;
          if (diff < 0) diff = -diff;

          Serial.print(F("RIGHT PWM:")); Serial.print(pwm);
          Serial.print(F("  counts:")); Serial.print(diff);

          if (diff >= 3) {
            Serial.println(F("  <<< MOVING!"));
            Serial.print(F(">>> Set RIGHT_MOTOR_DEAD_PWM = ")); Serial.println(pwm);
            char b[22];
            oled_clear();
            oled_print(0,  0, "RIGHT: FOUND!");
            sprintf(b, "PWM = %u", pwm);
            oled_print(0, 20, b);
            sprintf(b, "Cnt diff = %ld", (long)diff);
            oled_print(0, 36, b);
            oled_print(0, 52, "Press START->next");
            oled_update();
            dz_found = true;
          } else {
            Serial.println(F("  (not moving)"));
          }

          motor_stop();
          delay(100);
        }
        if (!dz_found) {
          Serial.println(F("!!! Motor did NOT move up to PWM 2500 !!!"));
          Serial.println(F("Check wiring, power, and motor connections."));
          oled_clear();
          oled_print(0,  0, "RIGHT: NOT FOUND");
          oled_print(0, 20, "Check wiring!");
          oled_update();
        }
      }
      motor_stop();
      /* stay in state 6 — press BTN_START to continue */
      break;

    /* ── CPR verification ───────────────────────────────────── */
    case 7:
      encoder_reset_all();
      Serial.println(F("\n=== TEST F.1: CPR VERIFY (LEFT wheel) ==="));
      Serial.println(F("Mark wheel. Rotate exactly 1 revolution. Press BTN_MODE."));
      Serial.println(F("Expected: ~588 counts"));
      oled_clear();
      oled_print(0,  0, "CPR: LEFT wheel");
      oled_print(0, 16, "1. Mark wheel");
      oled_print(0, 28, "2. Rotate 1 rev");
      oled_print(0, 40, "3. Press MODE");
      oled_print(0, 52, "Exp: ~588");
      oled_update();
      break;
    case 8:
      encoder_reset_all();
      Serial.println(F("\n=== TEST F.1: CPR VERIFY (RIGHT wheel) ==="));
      Serial.println(F("Mark wheel. Rotate exactly 1 revolution. Press BTN_MODE."));
      Serial.println(F("Expected: ~588 counts"));
      oled_clear();
      oled_print(0,  0, "CPR: RIGHT wheel");
      oled_print(0, 16, "1. Mark wheel");
      oled_print(0, 28, "2. Rotate 1 rev");
      oled_print(0, 40, "3. Press MODE");
      oled_print(0, 52, "Exp: ~588");
      oled_update();
      break;
    }
    led_toggle(LED_DEBUG);
  }

  /* ── BTN_MODE: CPR capture (states 7,8) or encoder reset (others) ───── */
  if (button_just_pressed(BUTTON_MODE)) {
    if (test_state == 7 || test_state == 8) {
      EncoderID enc  = (test_state == 7) ? ENCODER_LEFT : ENCODER_RIGHT;
      int32_t   cnt  = encoder_get_count(enc);
      float     diff = (float)cnt - 1820.0f;
      float     err  = (diff / 1820.0f) * 100.0f;
      bool      ok   = (fabsf(diff) <= 20.0f);   /* ±1.1% tolerance */

      Serial.println();
      Serial.print(F("[CPR] "));
      Serial.print(test_state == 7 ? "LEFT " : "RIGHT");
      Serial.print(F(" wheel: measured=")); Serial.print(cnt);
      Serial.print(F(" | expected=1820 | error="));
      Serial.print(err, 1); Serial.print(F("%"));
      Serial.println(ok ? F("  PASS") : F("  FAIL!"));
      if (!ok) {
        Serial.print(F("[CPR] Fix in robot_config.h: #define "));
        Serial.print(test_state == 7 ? "LEFT_ENCODER_CPR  " : "RIGHT_ENCODER_CPR ");
        Serial.println(cnt);
      }

      char cpr_buf[20];
      oled_clear();
      oled_print(0,  0, test_state == 7 ? "CPR LEFT result:" : "CPR RIGHT result:");
      sprintf(cpr_buf, "Got: %ld", (long)cnt);
      oled_print(0, 16, cpr_buf);
      oled_print(0, 32, "Exp: ~588-592");
      oled_print(0, 48, ok ? "  >> PASS <<  " : "Check value");
      oled_update();
    } else {
      encoder_reset_all();
      LOG_INFO("Encoders reset to 0.");
      led_toggle(LED_STATUS);
    }
  }

  /* ── Velocity LPF update at 50 ms ─────────────────────────────────── */
  static uint32_t last_vel_update = 0;
  if ((phase2_timer_ticks - last_vel_update) >= 50) {
    last_vel_update = phase2_timer_ticks;
    encoder_update_velocity(0.05f);
  }

  /*
   * 500ms Serial + OLED print block.
   * ONLY active for states 0-4 (drive states).
   * States 5-8 own the OLED themselves — do not let this block interfere.
   */
  static uint32_t last_enc_print = 0;
  if (test_state <= 4 && (phase2_timer_ticks - last_enc_print) >= 500) {
    last_enc_print = phase2_timer_ticks;

    int32_t l_delta = encoder_get_delta(ENCODER_LEFT);
    int32_t r_delta = encoder_get_delta(ENCODER_RIGHT);
    /* encoder_get_delta() returns counts since the LAST 50ms velocity update   */
    /* (consumed by encoder_update_velocity every 50ms). Divide by 0.05s = ×20. */
    float l_speed_raw  = encoder_counts_to_speed(l_delta * 20.0f);
    float r_speed_raw  = encoder_counts_to_speed(r_delta * 20.0f);
    float l_speed_filt = encoder_get_speed_mms(ENCODER_LEFT);
    float r_speed_filt = encoder_get_speed_mms(ENCODER_RIGHT);
    float l_dist_filt  = encoder_get_distance_mm(ENCODER_LEFT);
    float r_dist_filt  = encoder_get_distance_mm(ENCODER_RIGHT);

    char buf[32];
    oled_clear();
    oled_print(0,  0, "- Phase 2 Test -");
    sprintf(buf, "L: %d mm/s", (int)l_speed_filt);
    oled_print(0, 16, buf);
    sprintf(buf, "R: %d mm/s", (int)r_speed_filt);
    oled_print(0, 28, buf);
    sprintf(buf, "dL:%ld dR:%ld mm", (long)l_dist_filt, (long)r_dist_filt);
    oled_print(0, 40, buf);
    sprintf(buf, "Bat:%u mV", battery_get_voltage_mv());
    oled_print(0, 52, buf);
    oled_update();

    Serial.print(F("[Phase 2]"));
    Serial.print(F(" L_raw:"));    Serial.print(l_speed_raw,  1);
    Serial.print(F(" L_filt:"));   Serial.print(l_speed_filt, 1);
    Serial.print(F(" mm/s | R_raw:")); Serial.print(r_speed_raw,  1);
    Serial.print(F(" R_filt:"));   Serial.print(r_speed_filt, 1);
    Serial.print(F(" mm/s | DistL:")); Serial.print(l_dist_filt, 0);
    Serial.print(F(" DistR:"));    Serial.print(r_dist_filt,  0);
    Serial.println(F(" mm"));
  }

  delay(5);
  return;
#endif

#if PHASE_3_TEST_MODE == 1
  button_update();
  led_update();

  if (button_just_pressed(BUTTON_START)) {
      LOG_INFO("Re-calibrating Gyro (no warm-up)...");
      calibrate_with_warmup(0);  /* 0 = skip warm-up, calibrate immediately */
  }
  
  if (button_just_pressed(BUTTON_MODE)) {
      led_toggle(LED_DEBUG);
  }

  static uint32_t last_sensor_print = 0;
  // Update sensors every 100ms (10Hz)
  if ((phase3_timer_ticks - last_sensor_print) >= 100) {
      last_sensor_print = phase3_timer_ticks;
      
      sensor_manager_update();
      
      // Use the new EMA filter (dt = 0.1s for 10Hz)
      mpu6050_set_stationary(true); // Tell filter robot is not moving
      mpu6050_update_filter(0.1f);
      
      IMUScaledData imu_filt;
      mpu6050_get_filtered(&imu_filt);
      
      IMUScaledData imu_raw;
      mpu6050_read_scaled(&imu_raw);
      
      uint16_t dist_f = distance_get_mm(TOF_FRONT);
      uint16_t dist_fl = distance_get_mm(TOF_FRONT_LEFT);
      uint16_t dist_fr = distance_get_mm(TOF_FRONT_RIGHT);
      uint16_t dist_l = distance_get_mm(TOF_LEFT);
      uint16_t dist_r = distance_get_mm(TOF_RIGHT);
      
      char buf[32];
      oled_clear();
      oled_print(0, 0, "- Phase 3 Test -");
      
      sprintf(buf, "F:%u FL:%u FR:%u", dist_f, dist_fl, dist_fr);
      oled_print(0, 15, buf);
      
      sprintf(buf, "L:%u R:%u", dist_l, dist_r);
      oled_print(0, 25, buf);
      
      String gyroStr = String(imu_filt.gyro_z_dps, 1);
      sprintf(buf, "GzFilt: %s dps", gyroStr.c_str());
      oled_print(0, 40, buf);
      
      sprintf(buf, "Bat: %u mV", battery_get_voltage_mv());
      oled_print(0, 50, buf);
      
      oled_update();
      
      Serial.print("[Phase3] F:");
      Serial.print(dist_f);
      Serial.print(" FL:");
      Serial.print(dist_fl);
      Serial.print(" FR:");
      Serial.print(dist_fr);
      Serial.print(" L:");
      Serial.print(dist_l);
      Serial.print(" R:");
      Serial.print(dist_r);
      Serial.print(" | GzRaw:");
      Serial.print(imu_raw.gyro_z_dps, 1);
      Serial.print(" | GzFilt:");
      Serial.print(imu_filt.gyro_z_dps, 2);
      Serial.print(" | Bat:");
      Serial.print(battery_get_voltage_mv());
      Serial.print(" | BiasZ:");
      Serial.println(mpu6050_get_gyro_bias_z(), 1);
  }

  delay(5);
  return;
#endif

  // Watchdog feed (from plan)
  // feedWatchdog();

  // Read user inputs
  button_update();
  serial_debug_update();

  // Update slow sensors (I2C ToF)
  sensor_manager_update();

  // Run high-level state machine (Exploring, Returning, Speed Run)
  fsm_update();

  // Update Display based on state
  if (fsm_get_state() == STATE_IDLE) {
    if (!menu_update()) {
      status_screen_draw_main();
    }
  }

  // Stall and Battery Checks (from plan)
  // checkMotorStall();
  // checkBattery();

  // Small delay to prevent tight loop lockup if needed
  delay(1);
}
