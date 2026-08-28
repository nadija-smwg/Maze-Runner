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
#include "src/sensors/distance_manager.h"
#include "src/sensors/mpu6050.h"
#include "src/sensors/sensor_fusion.h"
#include "src/sensors/sensor_manager.h"

// Localization
#include "src/localization/odometry.h"
#include "src/localization/position_estimator.h"

// Control
#include "src/control/heading_controller.h"
#include "src/control/motion_controller.h"
#include "src/control/speed_controller.h"
#include "src/control/velocity_controller.h"

// Robot
#include "src/robot/mission_manager.h"
#include "src/robot/robot_state_machine.h"

// Display
#include "src/display/menu.h"
#include "src/display/oled_driver.h"
#include "src/display/status_screen.h"

// Utils
#include "src/utils/debug_buffer.h"
#include "src/utils/logger.h"
#include "src/utils/serial_debug.h"

// Phase testing mode flags (set to 1 for active test mode)
#define PHASE_1_TEST_MODE 0
#define PHASE_2_TEST_MODE 0
#define PHASE_3_TEST_MODE 0
#define PHASE_4_TEST_MODE 0
#define PHASE_5_TEST_MODE 1

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

#if PHASE_4_TEST_MODE == 1
volatile uint32_t phase4_timer_ticks = 0;
void phase4_timer_callback(void) { phase4_timer_ticks++; }
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

#if PHASE_4_TEST_MODE == 1
  LOG_INFO("=== PHASE 4 SENSOR FUSION TEST MODE ===");
  button_init();
  led_init();
  battery_init();
  encoder_init();

  Wire.setSCL(PIN_I2C_SCL);
  Wire.setSDA(PIN_I2C_SDA);
  Wire.begin();
  Wire.setClock(400000);
  if (oled_init()) {
    oled_clear();
    oled_print(0, 0, "Phase 4 Booting");
    oled_update();
  }

  sensor_manager_init();
  calibrate_all();
  fusion_init();

  timer_init(phase4_timer_callback);
  timer_start();

  LOG_INFO("Phase 4 Ready!");
  return;
#endif

#if PHASE_5_TEST_MODE == 1
  LOG_INFO("=== PHASE 5 TEST MODE ===");
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
    oled_print(0, 0, "Phase 5");
    oled_update();
  }

  sensor_manager_init();
  calibrate_all();

  LOG_INFO("Phase 5 Ready!");
  LOG_INFO("Press START to begin Wall Following test.");
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

  // Optional Gyro Calibration Phase (from plan)
  LOG_INFO("Calibrating Gyro - Keep Robot Still");
  // calibrateGyro(); // Assume integrated in fusion_init or sensor_manager

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

  static int test_state = 0;
  if (button_just_pressed(BUTTON_START)) {
    test_state = (test_state + 1) % 6;
    switch (test_state) {
    case 0:
      motor_stop();
      LOG_INFO("[State 0] Motors STOPPED.");
      break;
    case 1:
      motor_forward(1500);
      LOG_INFO("[State 1] Motors FORWARD (PWM 1500).");
      break;
    case 2:
      motor_reverse(1500);
      LOG_INFO("[State 2] Motors REVERSE (PWM 1500).");
      break;
    case 3:
      motor_turn_left(1500);
      LOG_INFO("[State 3] Motors TURN LEFT (PWM 1500).");
      break;
    case 4:
      motor_turn_right(1500);
      LOG_INFO("[State 4] Motors TURN RIGHT (PWM 1500).");
      break;
    case 5:
      motor_stop();
      LOG_INFO("[State 5] Motors STOPPED.");
      break;
    }
    led_toggle(LED_DEBUG);
  }

  if (button_just_pressed(BUTTON_MODE)) {
    encoder_reset_all();
    LOG_INFO("Encoders Reset to 0!");
    led_toggle(LED_STATUS);
  }

  static uint32_t last_enc_print = 0;
  if ((phase2_timer_ticks - last_enc_print) >= 500) {
    last_enc_print = phase2_timer_ticks;

    // 1. Get absolute positions
    int32_t l_cnt = encoder_get_count(ENCODER_LEFT);
    int32_t r_cnt = encoder_get_count(ENCODER_RIGHT);
    float l_mm = encoder_counts_to_mm(l_cnt);
    float r_mm = encoder_counts_to_mm(r_cnt);

    // 2. Get deltas to calculate speed
    // This block runs every 500ms (0.5s), so multiply delta by 2 for
    // counts_per_sec
    int32_t l_delta = encoder_get_delta(ENCODER_LEFT);
    int32_t r_delta = encoder_get_delta(ENCODER_RIGHT);

    float l_speed = encoder_counts_to_speed(l_delta * 2.0f);
    float r_speed = encoder_counts_to_speed(r_delta * 2.0f);

    // Update OLED
    char buf[32];
    oled_clear();
    oled_print(0, 0, "- Phase 2 Test -");

    sprintf(buf, "Spd L: %d mm/s", (int)l_speed);
    oled_print(0, 15, buf);

    sprintf(buf, "Spd R: %d mm/s", (int)r_speed);
    oled_print(0, 25, buf);

    sprintf(buf, "Cnt L:%ld R:%ld", l_cnt, r_cnt);
    oled_print(0, 40, buf);

    sprintf(buf, "Dst L:%d R:%d", (int)l_mm, (int)r_mm);
    oled_print(0, 50, buf);

    oled_update();

    Serial.print("[Phase 2 2Hz] L: ");
    Serial.print(l_mm, 1);
    Serial.print("mm, ");
    Serial.print(l_speed, 1);
    Serial.print("mm/s | R: ");
    Serial.print(r_mm, 1);
    Serial.print("mm, ");
    Serial.print(r_speed, 1);
    Serial.println("mm/s");
  }

  delay(5);
  return;
#endif

#if PHASE_3_TEST_MODE == 1
  button_update();
  led_update();

  if (button_just_pressed(BUTTON_START)) {
    LOG_INFO("Re-calibrating Gyro...");
    calibrate_all();
  }

  if (button_just_pressed(BUTTON_MODE)) {
    led_toggle(LED_DEBUG);
  }

  static uint32_t last_sensor_print = 0;
  // Update sensors every 100ms (10Hz)
  if ((phase3_timer_ticks - last_sensor_print) >= 100) {
    last_sensor_print = phase3_timer_ticks;

    sensor_manager_update();

    IMUScaledData imu;
    mpu6050_read_scaled(&imu);

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

    String gyroStr = String(imu.gyro_z_dps, 1);
    sprintf(buf, "GyroZ: %s deg/s", gyroStr.c_str());
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
    Serial.print(" | Gz:");
    Serial.print(imu.gyro_z_dps, 1);
    Serial.print(" | Bat:");
    Serial.print(battery_get_voltage_mv());
    Serial.print(" | BiasZ:");
    Serial.print(mpu6050_get_gyro_bias_z(), 1);

    IMURawData raw;
    mpu6050_read_raw(&raw);
    Serial.print(" | RawZ:");
    Serial.println(raw.gyro_z);
  }

  delay(5);
  return;
#endif

#if PHASE_4_TEST_MODE == 1
  button_update();
  led_update();

  static uint32_t last_sensor_tick = millis();
  if (millis() - last_sensor_tick >= 10) {
    last_sensor_tick = millis();
    // Thanks to Round-Robin polling, this only takes ~30ms now!
    // It is totally safe to run alongside the gyro.
    sensor_manager_update();
  }

  // Sensor fusion update (Throttled to 100Hz)
  static uint32_t last_fusion_tick = millis();
  if (millis() - last_fusion_tick >= 10) {
    uint32_t now = millis();
    float dt = (now - last_fusion_tick) / 1000.0f;
    last_fusion_tick = now;

    // Guard against backward time
    if (dt <= 0.0f) {
      dt = 0.01f;
    }

    fusion_update(dt);
  }

  static uint32_t last_print = 0;
  if (millis() - last_print >= 100) {
    last_print = millis();
    Pose p = position_estimator_get_pose();

    char buf[32];
    oled_clear();
    oled_print(0, 0, "- Phase 4 -");

    // Also add button functionality to reset the pose
    if (button_is_pressed(BUTTON_MODE)) {
      fusion_reset_heading(0.0f);
      Pose zero = {0.0f, 0.0f, 0.0f};
      odometry_set_pose(zero);
    }

    sprintf(buf, "X: %d mm", (int)p.x_mm);
    oled_print(0, 15, buf);

    sprintf(buf, "Y: %d mm", (int)p.y_mm);
    oled_print(0, 25, buf);

    float deg = p.theta_rad * (180.0f / 3.14159265f);
    sprintf(buf, "H: %d deg", (int)deg);
    oled_print(0, 40, buf);

    oled_update();

    Serial.print("[Phase4] X: ");
    Serial.print(p.x_mm, 1);
    Serial.print(" mm | Y: ");
    Serial.print(p.y_mm, 1);
    Serial.print(" mm | H: ");
    Serial.print(deg, 1);
    Serial.println(" deg");
  }
  delay(5);
  return;
#endif

#if PHASE_5_TEST_MODE == 1
  button_update();
  serial_debug_update();

  // Update slow sensors (I2C ToF)
  sensor_manager_update();

  // Get Distance Readings
  uint16_t dist_f = distance_get_mm(TOF_FRONT);
  uint16_t dist_l = distance_get_mm(TOF_LEFT);
  uint16_t dist_r = distance_get_mm(TOF_RIGHT);
  uint16_t dist_fl = distance_get_mm(TOF_FRONT_LEFT);
  uint16_t dist_fr = distance_get_mm(TOF_FRONT_RIGHT);

  // Calculate Centering Error
  // If pushed to the left wall, L decreases, R increases -> Error becomes positive.
  int TARGET_DIST = 45; // mm (Target distance from a single side wall)
  int TARGET_DIAG = 65; // mm (Target distance from a single diagonal wall)
  int error = 0;

  bool has_l = (dist_l < 150);
  bool has_r = (dist_r < 150);
  bool has_fl = (dist_fl < 250); // Diagonals read longer distances
  bool has_fr = (dist_fr < 250);

  if (has_l && has_r) {
    // Both side walls present (Double wall following)
    error = (int)dist_r - (int)dist_l;
    
    // Add front diagonal sensors if they are also seeing walls!
    if (has_fl && has_fr) {
      error += (int)dist_fr - (int)dist_fl;
    }
  } else if (has_l) {
    // Only left wall present
    error = (TARGET_DIST - (int)dist_l) * 2;
    if (has_fl) {
      error += (TARGET_DIAG - (int)dist_fl) * 2;
    }
  } else if (has_r) {
    // Only right wall present
    error = ((int)dist_r - TARGET_DIST) * 2;
    if (has_fr) {
      error += ((int)dist_fr - TARGET_DIAG) * 2;
    }
  } else {
    // NO walls! Drive straight
    error = 0;
  }

  // Grid tracking
  enum Heading { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
  static int grid_x = 0;
  static int grid_y = 0;
  static int heading = NORTH;
  static int last_cells = 0;
  static bool is_first_run = true;
  static float boundary_dist = 0.0f;
  static bool cell_has_left = false;
  static bool cell_has_right = false;
  
  static bool visited[16][16] = {false};
  static int total_visited = 0;

  static int p5_state = 0; // 0 = IDLE, 1 = DRIVE
  static int kp = 2;       // Steering power (reduced to stop oscillation)
  static int kd = 15;      // Derivative damper (acts as a brake to stop wobble)
  static int prev_error = 0;

  static uint32_t start_press_time = 0;
  static bool start_was_pressed = false;
  bool start_pressed = button_is_pressed(BUTTON_START);

  if (start_pressed && !start_was_pressed) {
    start_press_time = millis();
  }

  if (!start_pressed && start_was_pressed) {
    // Button released
    if (millis() - start_press_time > 1000) { // Held for > 1 second
      LOG_INFO("Long Press Detected: REBOOTING...");
      oled_clear();
      oled_print(0, 0, "REBOOTING...");
      oled_update();
      delay(500);
      NVIC_SystemReset(); // Hardware reboot
    } else {
      // Short press: Toggle IDLE / DRIVE
      if (p5_state == 0) {
        p5_state = 1;
        is_first_run = true;
        encoder_reset_all(); // Reset encoders when starting a run
        
        // Reset grid tracker
        memset(visited, 0, sizeof(visited));
        grid_x = 0;
        grid_y = 0;
        heading = NORTH;
        visited[0][0] = true;
        total_visited = 1;
        boundary_dist = 0.0f;
        cell_has_left = false;
        cell_has_right = false;
        
        LOG_INFO("STATE -> DRIVE");
      } else {
        p5_state = 0;
        motor_stop();
        LOG_INFO("STATE -> IDLE");
      }
    }
  }
  start_was_pressed = start_pressed;

  static int left_pwm = 0;
  static int right_pwm = 0;

  if (p5_state == 0) {
    motor_stop(); // Ensure motors stay stopped in IDLE
    left_pwm = 0;
    right_pwm = 0;
    if (button_just_pressed(BUTTON_MODE)) {
      kd += 5;
      if (kd > 60)
        kd = 0;
    }
  } else if (p5_state == 1) {
    // DRIVE State

    // Grid Tracker
    int32_t avg_counts =
        (encoder_get_count(ENCODER_LEFT) + encoder_get_count(ENCODER_RIGHT)) /
        2;
    float dist_traveled_mm = encoder_counts_to_mm(avg_counts);
    float offset = (is_first_run) ? 0.0f : 90.0f;
    int current_cells = (int)((dist_traveled_mm + offset) / 180.0f);
    if (current_cells > last_cells) {
      boundary_dist = dist_traveled_mm;
      cell_has_left = false;
      cell_has_right = false;
      
      if (heading == NORTH)
        grid_y++;
      else if (heading == SOUTH)
        grid_y--;
      else if (heading == EAST)
        grid_x++;
      else if (heading == WEST)
        grid_x--;
      last_cells = current_cells;

      // Mark cell as visited
      if (grid_x >= 0 && grid_x < 16 && grid_y >= 0 && grid_y < 16) {
        if (!visited[grid_x][grid_y]) {
          visited[grid_x][grid_y] = true;
          total_visited++;
        }
      }

      // Check if we hit 20 cells
      if (total_visited >= 20) {
        p5_state = 3; // FINISHED state
        motor_stop();
        left_pwm = 0;
        right_pwm = 0;
        LOG_INFO("20 Cells Reached! STATE -> FINISHED");
      } else {
        // Take a tiny 10ms stop each time a cell (180mm) is crossed
        motor_stop();
        delay(10);
      }
    }

    // Determine if the next cell straight ahead is visited
    int next_x = grid_x;
    int next_y = grid_y;
    if (heading == NORTH) next_y++;
    else if (heading == SOUTH) next_y--;
    else if (heading == EAST) next_x++;
    else if (heading == WEST) next_x--;

    bool front_visited = true; // Default to true if out of bounds
    if (next_x >= 0 && next_x < 16 && next_y >= 0 && next_y < 16) {
      front_visited = visited[next_x][next_y];
    }

    // Check visited status of left and right adjacent cells to see if we have unvisited options
    int left_x = grid_x, left_y = grid_y;
    int left_h = (heading + 3) % 4;
    if (left_h == NORTH) left_y++; else if (left_h == SOUTH) left_y--; else if (left_h == EAST) left_x++; else if (left_h == WEST) left_x--;
    bool left_visited = true;
    if (left_x >= 0 && left_x < 16 && left_y >= 0 && left_y < 16) left_visited = visited[left_x][left_y];

    int right_x = grid_x, right_y = grid_y;
    int right_h = (heading + 1) % 4;
    if (right_h == NORTH) right_y++; else if (right_h == SOUTH) right_y--; else if (right_h == EAST) right_x++; else if (right_h == WEST) right_x--;
    bool right_visited = true;
    if (right_x >= 0 && right_x < 16 && right_y >= 0 && right_y < 16) right_visited = visited[right_x][right_y];

    float dist_into_cell = dist_traveled_mm - boundary_dist;

    // Latch side openings if we see them ANYWHERE in the cell before the center
    // This widened window prevents wheel-slip from causing missed openings!
    if (dist_into_cell >= 10.0f && dist_into_cell <= 150.0f) {
      if (dist_l > 130) cell_has_left = true;
      if (dist_r > 130) cell_has_right = true;
    }

    bool has_unvisited_turn = (cell_has_left && !left_visited) || (cell_has_right && !right_visited);

    // Only treat a visited front cell as a wall if we ACTUALLY have an unvisited path to turn into.
    // Otherwise, just keep driving straight through the visited cells until we find a new path.
    bool force_virtual_turn = front_visited && has_unvisited_turn;
    
    // Window to ensure we ONLY turn when physically near the center of the cell, 
    // preventing early turns if sensors see ahead while still in the previous cell.
    bool at_cell_center = (dist_into_cell >= 90.0f && dist_into_cell <= 110.0f);

    bool front_wall = (dist_f <= 160 && dist_f > 0); // Detect front wall early
    bool emergency_stop = (dist_f <= 45 && dist_f > 0); // Hard stop to prevent crash

    if (((front_wall || force_virtual_turn) && at_cell_center) || emergency_stop) {
      // Stop exactly at the center if there is a wall or an unvisited turn!
      p5_state = 2; // Auto-turn
      motor_stop();
      left_pwm = 0;
      right_pwm = 0;
      LOG_INFO("Center Reached (Wall/Turn)! STATE -> TURN");
    } else {
      // Wall following PD-Controller

      int d_error = error - prev_error;
      prev_error = error;

      int steering = 0;
      // Do not change motor speed if error is 0 (deadband)
      if (error != 0) {
        steering = (kp * error) + (kd * d_error);
      }

      // Fixed steering polarity! Positive error means too close to LEFT wall.
      // So if error > 0, we must steer RIGHT (left wheel faster, right wheel slower).
      left_pwm = 950 + steering;
      right_pwm = 950 - steering;

      motor_set_both(left_pwm, right_pwm);
    }
  } else if (p5_state == 2) {
    // TURN State
    motor_stop();
    delay(200); // Brief pause before turning

    float target_angle = 90.0;

    // Check visited status of adjacent cells
    int left_x = grid_x, left_y = grid_y;
    int left_h = (heading + 3) % 4;
    if (left_h == NORTH) left_y++; else if (left_h == SOUTH) left_y--; else if (left_h == EAST) left_x++; else if (left_h == WEST) left_x--;
    bool left_visited = true;
    if (left_x >= 0 && left_x < 16 && left_y >= 0 && left_y < 16) left_visited = visited[left_x][left_y];

    int right_x = grid_x, right_y = grid_y;
    int right_h = (heading + 1) % 4;
    if (right_h == NORTH) right_y++; else if (right_h == SOUTH) right_y--; else if (right_h == EAST) right_x++; else if (right_h == WEST) right_x--;
    bool right_visited = true;
    if (right_x >= 0 && right_x < 16 && right_y >= 0 && right_y < 16) right_visited = visited[right_x][right_y];
    
    // Use the latched values to prevent errors if the robot slid past the center!
    bool can_go_left = cell_has_left;
    bool can_go_right = cell_has_right;

    if (can_go_left && can_go_right) {
      // Intersection! Prioritize unvisited paths
      if (!left_visited && right_visited) {
        heading = left_h;
        LOG_INFO("Turning LEFT (Unvisited)");
        motor_set_both(-900, 900);
      } else if (left_visited && !right_visited) {
        heading = right_h;
        LOG_INFO("Turning RIGHT (Unvisited)");
        motor_set_both(900, -900);
      } else {
        // Both unvisited or both visited. Default to RIGHT hand rule to avoid live sensor noise!
        heading = right_h;
        LOG_INFO("Turning RIGHT (Tiebreaker)");
        motor_set_both(900, -900);
      }
    } else if (can_go_left) {
      heading = left_h;
      LOG_INFO("Turning LEFT");
      motor_set_both(-900, 900);
    } else if (can_go_right) {
      heading = right_h;
      LOG_INFO("Turning RIGHT");
      motor_set_both(900, -900);
    } else {
      // Dead End! Walls on both sides -> Turn 180 degrees
      heading = (heading + 2) % 4; // U-Turn
      LOG_INFO("DEAD END! Turning 180 degrees");
      motor_set_both(900, -900); // Pivot Right
      target_angle = 180.0;
    }

    // Gyroscope tracking loop
    float current_angle = 0.0;
    uint32_t last_time = micros();

    // Loop until we reach target angle
    while (abs(current_angle) < target_angle) {
      IMUScaledData imu;
      mpu6050_read_scaled(&imu); // Read the IMU

      uint32_t now = micros();
      float dt = (now - last_time) / 1000000.0f; // Time in seconds
      last_time = now;

      current_angle += imu.gyro_z_dps * dt; // Add degrees turned

      delay(2); // Small delay for stability
    }

    motor_stop();
    delay(100); // Brief pause after turning

    // Go back to driving
    last_cells = 0;
    is_first_run = false; // Turn complete, use normal offsets now
    encoder_reset_all();  // Reset cells for the new corridor
    boundary_dist = -90.0f; // Since we are at the center, the boundary is conceptually 90mm behind us
    cell_has_left = false;
    cell_has_right = false;
    p5_state = 1;
  } else if (p5_state == 3) {
    // FINISHED State
    motor_stop();
    left_pwm = 0;
    right_pwm = 0;
  }

  // Print sensor readings to Serial and OLED
  static uint32_t last_print = 0;
  if (millis() - last_print >= 100) { // Print at 10Hz
    last_print = millis();

    // Update OLED Display
    char buf[32];
    oled_clear();

    const char *dir_str = (heading == NORTH)   ? "N"
                          : (heading == EAST)  ? "E"
                          : (heading == SOUTH) ? "S"
                                               : "W";
    if (p5_state == 0) {
      sprintf(buf, "IDLE | (%d,%d) %s", grid_x, grid_y, dir_str);
      oled_print(0, 0, buf);
    } else if (p5_state == 1) {
      sprintf(buf, "DRV  | (%d,%d) %s", grid_x, grid_y, dir_str);
      oled_print(0, 0, buf);
    } else if (p5_state == 2) {
      sprintf(buf, "TURN | (%d,%d) %s", grid_x, grid_y, dir_str);
      oled_print(0, 0, buf);
    } else if (p5_state == 3) {
      sprintf(buf, "DONE! 20 Cells");
      oled_print(0, 0, buf);
    }

    if (p5_state == 3) {
      // If finished, only show the map and some basic info
      sprintf(buf, "Pos: (%d,%d)", grid_x, grid_y);
      oled_print(0, 15, buf);
      oled_print(0, 30, "Map ->");
    } else {
      // Normal driving info
      // Clamp values to 999 so they fit on the left side of the OLED (map is on the right)
      uint16_t df = (dist_f > 999) ? 999 : dist_f;
      uint16_t dl = (dist_l > 999) ? 999 : dist_l;
      uint16_t dr = (dist_r > 999) ? 999 : dist_r;
      uint16_t dfl = (dist_fl > 999) ? 999 : dist_fl;
      uint16_t dfr = (dist_fr > 999) ? 999 : dist_fr;

      sprintf(buf, "F:%u", df); oled_print(0, 10, buf);
      sprintf(buf, "L:%u R:%u", dl, dr); oled_print(0, 20, buf);
      sprintf(buf, "l:%u r:%u", dfl, dfr); oled_print(0, 30, buf); // lower case l/r for FL/FR
      sprintf(buf, "E:%d D:%d", error, kd); oled_print(0, 40, buf);
      sprintf(buf, "%d|%d", left_pwm, right_pwm); oled_print(0, 50, buf);
    }

    // Draw 16x16 Grid on the right side of the screen
    for (int x = 0; x < 16; x++) {
      for (int y = 0; y < 16; y++) {
        if (visited[x][y]) {
          // Bottom-left is (0,0). So OLED y should be inverted.
          // Cell size 4x4. Placed starting at x=64 so it uses the right half.
          int oled_x = 64 + (x * 4);
          int oled_y = 60 - (y * 4); // 64 - 4 = 60 (top-left of the 4x4 rect)
          
          // Flash the current position if we are in state 3
          if (p5_state == 3 && x == grid_x && y == grid_y && (millis() / 500) % 2 == 0) {
            continue; // Blink
          }
          oled_fill_rect(oled_x, oled_y, 4, 4);
        }
      }
    }

    oled_update();
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
