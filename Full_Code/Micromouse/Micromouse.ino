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
#include "src/control/wall_follower.h"

// Robot
#include "src/robot/mission_manager.h"
#include "src/robot/robot_state_machine.h"

// Maze
#include "src/maze/solver.h"

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
  LOG_INFO("=== PHASE 5 COMPETITION MODE ===");
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
    oled_print(0, 0, "MazeX v1.0");
    oled_print(0, 15, "Initializing...");
    oled_update();
  }

  sensor_manager_init();
  calibrate_all();
  wall_follower_init();

  oled_clear();
  oled_print(0, 0, "MazeX v1.0");
  oled_print(0, 15, "Ready!");
  oled_print(0, 30, "START = Search");
  oled_print(0, 42, "MODE  = KP+");
  oled_update();

  LOG_INFO("Phase 5 Competition Mode Ready!");
  LOG_INFO("Press START to begin maze search.");
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
  // ======================================================================
  //  PHASE 5 â€” COMPETITION MAZE SEARCH (Flood Fill + Wall Following)
  // ======================================================================

  // -- State Machine Enum ------------------------------------------------
  enum P5State {
    P5_IDLE,             // Waiting for button press
    P5_SEARCH_DRIVE,     // Wall-following forward through corridor
    P5_SEARCH_ARRIVED,   // Reached cell center â€” read walls, run solver
    P5_TURNING,          // Gyro-tracked pivot turn
    P5_AT_GOAL,          // Reached center goal!
    P5_RETURN_DRIVE,     // Returning to start
    P5_RETURN_ARRIVED,   // Cell center on return
    P5_RETURN_TURNING,   // Turning during return
    P5_DONE              // Back at start
  };

  // -- Static State Variables --------------------------------------------
  static P5State p5_state = P5_IDLE;
  static Solver solver;
  static bool solver_initialized = false;
  static bool is_returning = false;
  static float drive_target_mm = 180.0f;
  static Direction target_direction = DIR_NORTH;
  static uint32_t run_start_time = 0;
  static uint16_t cells_visited = 0;

  // -- Input Handling ----------------------------------------------------
  button_update();
  sensor_manager_update();

  // Long-press reboot detection
  static uint32_t start_press_time = 0;
  static bool start_was_pressed = false;
  bool start_pressed = button_is_pressed(BUTTON_START);

  if (start_pressed && !start_was_pressed) {
    start_press_time = millis();
  }
  if (!start_pressed && start_was_pressed) {
    if (millis() - start_press_time > 1500) {
      LOG_INFO("Long Press: REBOOTING...");
      oled_clear();
      oled_print(20, 25, "REBOOTING...");
      oled_update();
      delay(500);
      NVIC_SystemReset();
    }
  }
  start_was_pressed = start_pressed;

  // -- Sensor Readings (always available) --------------------------------
  uint16_t dist_f  = distance_get_mm(TOF_FRONT);
  uint16_t dist_l  = distance_get_mm(TOF_LEFT);
  uint16_t dist_r  = distance_get_mm(TOF_RIGHT);
  bool wall_f = distance_has_wall_front();
  bool wall_l = distance_has_wall_left();
  bool wall_r = distance_has_wall_right();
  float centering_error = distance_get_centering_error();

  // -- Helper strings ----------------------------------------------------
  const char* dir_names[] = {"N", "E", "S", "W"};
  const char* state_names[] = {"IDLE", "DRV", "ARR", "TURN",
                                "GOAL", "RET", "RARR", "RTURN", "DONE"};

  // ======================================================================
  //  STATE MACHINE
  // ======================================================================
  switch (p5_state) {

  // -- IDLE --------------------------------------------------------------
  case P5_IDLE: {
    motor_stop();

    // MODE button: adjust wall follower KP
    if (button_just_pressed(BUTTON_MODE)) {
      float kp = wall_follower_get_kp();
      kp += 1.0f;
      if (kp > 15.0f) kp = 1.0f;
      wall_follower_set_kp(kp);
    }

    // START button (short press): begin search
    if (button_just_pressed(BUTTON_START)) {
      LOG_INFO("Starting maze search...");

      // Initialize solver on first run
      if (!solver_initialized) {
        solver_init(&solver);
        solver_initialized = true;
      }

      // Robot starts at center of cell (0,0), facing NORTH
      // Read walls and record in the starting cell
      solver_record_walls(&solver, wall_f, wall_l, wall_r);

      is_returning = false;
      encoder_reset_all();
      run_start_time = millis();
      cells_visited = 1; // start cell counts

      // Get first direction from flood fill
      target_direction = solver_search_step(&solver);
      TurnType turn = get_turn_type(solver.mouse_heading, target_direction);

      if (turn == TURN_NONE) {
        // Go straight â€” drive to next cell center (180mm)
        drive_target_mm = 180.0f;
        encoder_reset_all();
        p5_state = P5_SEARCH_DRIVE;
        LOG_INFO("Driving forward");
      } else {
        // Need to turn first
        p5_state = P5_TURNING;
        LOG_INFO("Turning before first move");
      }
    }
    break;
  }

  // -- SEARCH DRIVE / RETURN DRIVE ---------------------------------------
  case P5_SEARCH_DRIVE:
  case P5_RETURN_DRIVE: {
    // Wall-following PD controller drives motors
    wall_follower_update(centering_error, 0.01f);

    // Emergency front wall stop
    if (dist_f <= 40 && dist_f > 0) {
      motor_stop();
      encoder_reset_all();
      p5_state = is_returning ? P5_RETURN_ARRIVED : P5_SEARCH_ARRIVED;
      LOG_INFO("Front wall stop!");
      break;
    }

    // Check if we've driven enough distance to reach next cell center
    int32_t avg_counts = (encoder_get_count(ENCODER_LEFT) +
                          encoder_get_count(ENCODER_RIGHT)) / 2;
    float dist_driven = encoder_counts_to_mm(avg_counts);

    if (dist_driven >= drive_target_mm) {
      // Arrived at next cell center!
      motor_stop();
      encoder_reset_all();
      p5_state = is_returning ? P5_RETURN_ARRIVED : P5_SEARCH_ARRIVED;
    }
    break;
  }

  // -- SEARCH ARRIVED (at cell center) -----------------------------------
  case P5_SEARCH_ARRIVED: {
    motor_stop();
    delay(250); // Pause to fully stop and stabilize sensors

    // Re-read sensors after stopping
    sensor_manager_update();
    wall_f = distance_has_wall_front();
    wall_l = distance_has_wall_left();
    wall_r = distance_has_wall_right();

    // Advance solver position to this new cell
    solver_advance(&solver, solver.mouse_heading);
    cells_visited++;

    // Record walls at current cell
    solver_record_walls(&solver, wall_f, wall_l, wall_r);

    // Check if we reached the goal!
    if (solver_at_goal(&solver)) {
      p5_state = P5_AT_GOAL;
      LOG_INFO("*** GOAL REACHED! ***");
      led_blink(LED_STATUS, 100);
      break;
    }

    // Get next direction from flood fill
    target_direction = solver_search_step(&solver);
    TurnType turn = get_turn_type(solver.mouse_heading, target_direction);

    Serial.print("[CELL] (");
    Serial.print(solver.mouse_x); Serial.print(",");
    Serial.print(solver.mouse_y); Serial.print(") ");
    Serial.print(dir_names[solver.mouse_heading]);
    Serial.print(" W:F"); Serial.print(wall_f);
    Serial.print(" L"); Serial.print(wall_l);
    Serial.print(" R"); Serial.print(wall_r);
    Serial.print(" ->Next:"); Serial.print(dir_names[target_direction]);
    Serial.print(" Turn:"); Serial.println(turn);

    if (turn == TURN_NONE) {
      // Continue straight
      drive_target_mm = 180.0f;
      encoder_reset_all();
      p5_state = P5_SEARCH_DRIVE;
    } else {
      // Need to turn
      p5_state = P5_TURNING;
    }
    break;
  }

  // -- TURNING (gyro-tracked pivot) --------------------------------------
  case P5_TURNING:
  case P5_RETURN_TURNING: {
    motor_stop();
    delay(100); // Stabilize before turn

    TurnType turn = get_turn_type(solver.mouse_heading, target_direction);

    float target_angle = 0.0f;
    int16_t turn_pwm_l = 0;
    int16_t turn_pwm_r = 0;

    switch (turn) {
      case TURN_RIGHT_90:
        target_angle = 90.0f;
        turn_pwm_l = 800;
        turn_pwm_r = -800;
        LOG_INFO("Turn RIGHT 90");
        break;
      case TURN_LEFT_90:
        target_angle = 90.0f;
        turn_pwm_l = -800;
        turn_pwm_r = 800;
        LOG_INFO("Turn LEFT 90");
        break;
      case TURN_180:
        target_angle = 180.0f;
        turn_pwm_l = 800;
        turn_pwm_r = -800;
        LOG_INFO("Turn 180");
        break;
      default:
        break;
    }

    if (target_angle > 0.0f) {
      // Show turn info on OLED before blocking turn
      char buf[32];
      oled_clear();
      sprintf(buf, "TURN %s %d deg",
              (turn == TURN_RIGHT_90) ? "RIGHT" :
              (turn == TURN_LEFT_90) ? "LEFT" : "180",
              (int)target_angle);
      oled_print(0, 0, buf);
      sprintf(buf, "(%d,%d) %s->%s",
              solver.mouse_x, solver.mouse_y,
              dir_names[solver.mouse_heading],
              dir_names[target_direction]);
      oled_print(0, 15, buf);
      sprintf(buf, "Cells: %u", cells_visited);
      oled_print(0, 30, buf);
      oled_update();

      // Execute gyro-tracked turn
      motor_set_both(turn_pwm_l, turn_pwm_r);

      float accumulated_angle = 0.0f;
      uint32_t turn_start = micros();
      uint32_t last_time = micros();

      while (fabs(accumulated_angle) < target_angle) {
        IMUScaledData imu;
        mpu6050_read_scaled(&imu);

        uint32_t now = micros();
        float dt = (now - last_time) / 1000000.0f;
        last_time = now;

        if (dt > 0.0f && dt < 0.1f) {
          accumulated_angle += imu.gyro_z_dps * dt;
        }

        // Safety timeout: 3 seconds max
        if ((now - turn_start) > 3000000UL) {
          LOG_ERROR("Turn timeout!");
          break;
        }
        delay(2);
      }

      motor_stop();
      delay(100);

      // Post-turn overshoot correction
      float overshoot = fabs(accumulated_angle) - target_angle;
      if (overshoot > 3.0f) {
        LOG_INFO("Fixing overshoot");
        Serial.println(overshoot, 1);

        motor_set_both(-turn_pwm_l / 2, -turn_pwm_r / 2);
        float corr_angle = 0.0f;
        uint32_t corr_start = micros();
        uint32_t corr_last = micros();

        while (fabs(corr_angle) < (overshoot - 2.0f)) {
          IMUScaledData imu;
          mpu6050_read_scaled(&imu);
          uint32_t now = micros();
          float dt = (now - corr_last) / 1000000.0f;
          corr_last = now;
          if (dt > 0.0f && dt < 0.1f) {
            corr_angle += imu.gyro_z_dps * dt;
          }
          if ((now - corr_start) > 1000000UL) break;
          delay(2);
        }
        motor_stop();
        delay(50);
      }

      // Update solver heading
      solver.mouse_heading = target_direction;

      Serial.print("[TURN] Done: ");
      Serial.print(fabs(accumulated_angle), 1);
      Serial.println(" deg");
    }

    // After turn: drive to next cell
    drive_target_mm = 180.0f;
    encoder_reset_all();

    if (p5_state == P5_RETURN_TURNING) {
      p5_state = P5_RETURN_DRIVE;
    } else {
      p5_state = P5_SEARCH_DRIVE;
    }
    break;
  }

  // -- AT GOAL -----------------------------------------------------------
  case P5_AT_GOAL: {
    motor_stop();
    led_update();

    // START button: return to start
    if (button_just_pressed(BUTTON_START)) {
      LOG_INFO("Returning to start...");

      // Reverse flood fill: set goal to start cell
      const uint8_t start_goal[1][2] = {{START_X, START_Y}};
      flood_fill_compute(&solver.maze, start_goal, 1);

      is_returning = true;

      // Read walls and get return direction
      sensor_manager_update();
      wall_f = distance_has_wall_front();
      wall_l = distance_has_wall_left();
      wall_r = distance_has_wall_right();
      solver_record_walls(&solver, wall_f, wall_l, wall_r);

      target_direction = flood_fill_choose_direction(
          &solver.maze, solver.mouse_x, solver.mouse_y, solver.mouse_heading);

      TurnType turn = get_turn_type(solver.mouse_heading, target_direction);
      encoder_reset_all();

      if (turn == TURN_NONE) {
        drive_target_mm = 180.0f;
        p5_state = P5_RETURN_DRIVE;
      } else {
        p5_state = P5_RETURN_TURNING;
      }
    }
    break;
  }

  // -- RETURN ARRIVED ----------------------------------------------------
  case P5_RETURN_ARRIVED: {
    motor_stop();
    delay(250); // Pause to fully stop and stabilize sensors

    sensor_manager_update();
    wall_f = distance_has_wall_front();
    wall_l = distance_has_wall_left();
    wall_r = distance_has_wall_right();

    solver_advance(&solver, solver.mouse_heading);
    solver_record_walls(&solver, wall_f, wall_l, wall_r);

    // Check if back at start
    if (solver_at_start(&solver)) {
      p5_state = P5_DONE;
      LOG_INFO("Back at START!");
      break;
    }

    // Recompute flood from start
    const uint8_t ret_goal[1][2] = {{START_X, START_Y}};
    flood_fill_compute(&solver.maze, ret_goal, 1);

    target_direction = flood_fill_choose_direction(
        &solver.maze, solver.mouse_x, solver.mouse_y, solver.mouse_heading);

    TurnType turn = get_turn_type(solver.mouse_heading, target_direction);
    encoder_reset_all();

    if (turn == TURN_NONE) {
      drive_target_mm = 180.0f;
      p5_state = P5_RETURN_DRIVE;
    } else {
      p5_state = P5_RETURN_TURNING;
    }
    break;
  }

  // -- DONE (back at start) ----------------------------------------------
  case P5_DONE: {
    motor_stop();

    // START button: run again with retained maze data
    if (button_just_pressed(BUTTON_START)) {
      is_returning = false;

      // Reset solver position to start (maze data kept!)
      solver.mouse_x = START_X;
      solver.mouse_y = START_Y;
      solver.mouse_heading = DIR_NORTH;

      // Re-read walls and recompute flood
      sensor_manager_update();
      wall_f = distance_has_wall_front();
      wall_l = distance_has_wall_left();
      wall_r = distance_has_wall_right();
      solver_record_walls(&solver, wall_f, wall_l, wall_r);

      flood_fill_compute(&solver.maze, GOAL_CELLS, NUM_GOAL_CELLS);
      target_direction = solver_search_step(&solver);

      TurnType turn = get_turn_type(solver.mouse_heading, target_direction);
      run_start_time = millis();
      cells_visited = 1;
      encoder_reset_all();

      if (turn == TURN_NONE) {
        drive_target_mm = 180.0f;
        p5_state = P5_SEARCH_DRIVE;
      } else {
        p5_state = P5_TURNING;
      }
      LOG_INFO("Run again with mapped maze!");
    }
    break;
  }

  default:
    break;
  } // end switch

  // ======================================================================
  //  OLED DEBUG DISPLAY (10Hz update for all states)
  // ======================================================================
  static uint32_t last_oled_update = 0;
  if (millis() - last_oled_update >= 100) {
    last_oled_update = millis();
    led_update();

    char buf[32];
    oled_clear();

    uint16_t flood_val = 0;
    if (solver_initialized) {
      flood_val = solver_get_flood_value(&solver,
                   solver.mouse_x, solver.mouse_y);
    }
    uint32_t elapsed_s = 0;
    if (run_start_time > 0) {
      elapsed_s = (millis() - run_start_time) / 1000;
    }

    switch (p5_state) {

    case P5_IDLE: {
      oled_print(0, 0, "MazeX v1.0  IDLE");
      sprintf(buf, "KP: %d  MODE=+KP", (int)wall_follower_get_kp());
      oled_print(0, 12, buf);
      sprintf(buf, "F:%u L:%u R:%u", dist_f, dist_l, dist_r);
      oled_print(0, 24, buf);
      sprintf(buf, "Err:%d", (int)centering_error);
      oled_print(0, 36, buf);
      oled_print(0, 52, "START=Go");
      break;
    }

    case P5_SEARCH_DRIVE:
    case P5_RETURN_DRIVE: {
      const char* mode = is_returning ? "RET" : "SRC";
      sprintf(buf, "%s(%d,%d)%s F:%u",
              mode, solver.mouse_x, solver.mouse_y,
              dir_names[solver.mouse_heading], flood_val);
      oled_print(0, 0, buf);

      sprintf(buf, "F:%u L:%u R:%u", dist_f, dist_l, dist_r);
      oled_print(0, 11, buf);

      sprintf(buf, "W:%c%c%c Err:%d",
              wall_f ? 'F' : '.', wall_l ? 'L' : '.', wall_r ? 'R' : '.',
              (int)centering_error);
      oled_print(0, 22, buf);

      sprintf(buf, "Cor:%d KP:%d",
              wall_follower_get_last_correction(),
              (int)wall_follower_get_kp());
      oled_print(0, 33, buf);

      int32_t avg = (encoder_get_count(ENCODER_LEFT) +
                     encoder_get_count(ENCODER_RIGHT)) / 2;
      float driven = encoder_counts_to_mm(avg);
      sprintf(buf, "D:%.0f/%d C:%u",
              driven, (int)drive_target_mm, cells_visited);
      oled_print(0, 44, buf);

      sprintf(buf, "T:%lus", elapsed_s);
      oled_print(90, 55, buf);
      break;
    }

    case P5_SEARCH_ARRIVED:
    case P5_RETURN_ARRIVED: {
      sprintf(buf, "ARR (%d,%d) %s",
              solver.mouse_x, solver.mouse_y,
              dir_names[solver.mouse_heading]);
      oled_print(0, 0, buf);
      sprintf(buf, "F:%u L:%u R:%u", dist_f, dist_l, dist_r);
      oled_print(0, 12, buf);
      sprintf(buf, "Walls: %c%c%c",
              wall_f ? 'F' : '.', wall_l ? 'L' : '.', wall_r ? 'R' : '.');
      oled_print(0, 24, buf);
      sprintf(buf, "Flood: %u", flood_val);
      oled_print(0, 36, buf);
      sprintf(buf, "Nxt:%s C:%u T:%lus",
              dir_names[target_direction], cells_visited, elapsed_s);
      oled_print(0, 48, buf);
      break;
    }

    case P5_TURNING:
    case P5_RETURN_TURNING: {
      TurnType t = get_turn_type(solver.mouse_heading, target_direction);
      sprintf(buf, "TURNING %s",
              (t == TURN_RIGHT_90) ? "RIGHT 90" :
              (t == TURN_LEFT_90) ? "LEFT 90" : "180");
      oled_print(0, 0, buf);
      sprintf(buf, "(%d,%d) %s->%s",
              solver.mouse_x, solver.mouse_y,
              dir_names[solver.mouse_heading],
              dir_names[target_direction]);
      oled_print(0, 15, buf);
      sprintf(buf, "Cells:%u T:%lus", cells_visited, elapsed_s);
      oled_print(0, 36, buf);
      break;
    }

    case P5_AT_GOAL: {
      oled_print(10, 0, "*** GOAL! ***");
      sprintf(buf, "Cell: (%d,%d)",
              solver.mouse_x, solver.mouse_y);
      oled_print(0, 14, buf);
      sprintf(buf, "Time: %lu:%02lu",
              elapsed_s / 60, elapsed_s % 60);
      oled_print(0, 26, buf);
      sprintf(buf, "Cells: %u/256", cells_visited);
      oled_print(0, 38, buf);
      oled_print(0, 52, "START=Return");
      break;
    }

    case P5_DONE: {
      oled_print(0, 0, "** BACK AT START **");
      sprintf(buf, "Time: %lu:%02lu",
              elapsed_s / 60, elapsed_s % 60);
      oled_print(0, 14, buf);
      sprintf(buf, "Cells: %u mapped", cells_visited);
      oled_print(0, 26, buf);
      oled_print(0, 38, "Maze retained!");
      oled_print(0, 52, "START=Run again");
      break;
    }

    default:
      oled_print(0, 0, "MazeX v1.0");
      break;
    }

    oled_update();

    // Serial debug output
    Serial.print("[P5] ");
    Serial.print(state_names[p5_state]);
    Serial.print(" ("); Serial.print(solver.mouse_x);
    Serial.print(","); Serial.print(solver.mouse_y);
    Serial.print(")"); Serial.print(dir_names[solver.mouse_heading]);
    Serial.print(" F:"); Serial.print(dist_f);
    Serial.print(" L:"); Serial.print(dist_l);
    Serial.print(" R:"); Serial.print(dist_r);
    Serial.print(" Fl:"); Serial.print(flood_val);
    Serial.print(" Er:"); Serial.print(centering_error, 0);
    Serial.println();
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
