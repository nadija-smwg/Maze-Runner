/**
 * @file robot_config.h
 * @brief Physical robot constants and derived parameters.
 *
 * Contains all mechanical dimensions, motor specifications, encoder
 * parameters, and derived conversion factors for the Micromouse robot.
 *
 * These values must match your actual hardware. Measure carefully.
 *
 * @note This file is separate from config.h (algorithm tuning) and
 *       pin_config.h (pin assignments) to keep concerns isolated.
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* ═══════════════════════════════════════════════════════════════════════════
 *  Motor Specifications — N20 Metal Gear Motor
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @defgroup MotorSpecs Motor Mechanical Specifications
 *  N20 metal gear motor with integrated quadrature encoder.
 *  @{
 */

#define GEAR_RATIO              65.0f   /**< Gearbox ratio (output:input)   */
#define ENCODER_PPR             7.0f    /**< Pulses per revolution (raw)    */
#define ENCODER_QUADRATURE      4.0f    /**< Quadrature multiplier (×4)     */

/**
 * Counts Per Revolution (after gearing and quadrature decoding).
 * CPR = PPR × GEAR_RATIO × QUADRATURE = 7 × 65 × 4 = 1820 counts/rev
 */
#define ENCODER_CPR             (ENCODER_PPR * GEAR_RATIO * ENCODER_QUADRATURE)

/** @} */ // end MotorSpecs

/* ═══════════════════════════════════════════════════════════════════════════
 *  Wheel Geometry
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @defgroup WheelGeometry Wheel Physical Dimensions
 *  @{
 */

#define WHEEL_DIAMETER_MM       34.0f   /**< Wheel outer diameter (mm)      */

/**
 * Wheel circumference (mm).
 * C = π × D = π × 34.0 ≈ 106.81 mm
 */
#define WHEEL_CIRCUMFERENCE_MM  (3.14159265358979f * WHEEL_DIAMETER_MM)

/**
 * Distance per encoder count (mm/count).
 * = Circumference / CPR ≈ 106.81 / 1820 ≈ 0.0587 mm/count
 */
#define MM_PER_COUNT            (WHEEL_CIRCUMFERENCE_MM / ENCODER_CPR)

/** @} */ // end WheelGeometry

/* ═══════════════════════════════════════════════════════════════════════════
 *  Per-Wheel Calibration Constants
 *
 *  Status of each value:
 *    [OK]  = Verified from Phase 2 hardware test
 *    [TODO] = Needs physical measurement before competition
 *
 *  How to update this file:
 *    CPR:       Lift robot, mark wheel, hand-rotate 10 full revolutions.
 *               CPR = encoder_count / 10
 *    Diameter:  Drive robot 1000 mm on flat tape. Measure actual distance.
 *               diameter = actual_mm / (counts / CPR) / π
 *    Dead-zone: Run Phase 2 State 6 (L) and State 7 (R) dead-zone sweep.
 *               Record first 'MOVING' PWM value from Serial.
 *    Wheel base: Place robot against a straight wall. Mark wheel centres.
 *               Measure between marks with calipers.
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @defgroup PerWheelCalib Per-Wheel Calibration
 *  @{
 */

/**
 * [OK] Left wheel counts per revolution.
 * Theoretical: 7 PPR × 65:1 × 4 quadrature = 1820
 * Verified: Phase 2 distance data consistent with 1820 (no correction needed).
 * To re-verify: lift robot, mark wheel, count 10 revolutions manually.
 */
#define LEFT_ENCODER_CPR        (ENCODER_PPR * GEAR_RATIO * ENCODER_QUADRATURE)  /* 1820 */

/**
 * [OK] Right wheel counts per revolution.
 * Same as left — verified consistent in Phase 2 data.
 */
#define RIGHT_ENCODER_CPR       (ENCODER_PPR * GEAR_RATIO * ENCODER_QUADRATURE)  /* 1820 */

/**
 * [OK] Left wheel effective outer diameter (mm).
 * Theoretical: 34.0 mm. Phase 2 data consistent with this value.
 * To re-verify: drive 1000 mm measured on tape, then:
 *   diameter = 1000 / (encoder_counts / CPR) / π
 */
#define LEFT_WHEEL_DIAMETER_MM  34.0f

/**
 * [OK] Right wheel effective outer diameter (mm).
 * Same as left — consistent in Phase 2 data.
 */
#define RIGHT_WHEEL_DIAMETER_MM 34.0f

/** Left wheel circumference (mm) — derived. */
#define LEFT_WHEEL_CIRC_MM      (3.14159265f * LEFT_WHEEL_DIAMETER_MM)

/** Right wheel circumference (mm) — derived. */
#define RIGHT_WHEEL_CIRC_MM     (3.14159265f * RIGHT_WHEEL_DIAMETER_MM)

/** Left wheel distance per count (mm/count) — derived ≈ 0.0587 mm/count */
#define LEFT_MM_PER_COUNT       (LEFT_WHEEL_CIRC_MM  / LEFT_ENCODER_CPR)

/** Right wheel distance per count (mm/count) — derived ≈ 0.0587 mm/count */
#define RIGHT_MM_PER_COUNT      (RIGHT_WHEEL_CIRC_MM / RIGHT_ENCODER_CPR)

/**
 * [TODO] Minimum PWM to overcome static friction on the LEFT motor.
 *
 * Run Phase 2, press BTN_START until State 6 (dead-zone sweep LEFT).
 * Find the first Serial line showing 'MOVING' and enter that PWM below.
 *
 * Current value: 0 (not yet measured — replace before competition!)
 */
#define LEFT_MOTOR_DEAD_PWM     0       /* TODO: fill from Test 3.3 State 6 */

/**
 * [TODO] Minimum PWM to overcome static friction on the RIGHT motor.
 *
 * Run Phase 2, press BTN_START until State 7 (dead-zone sweep RIGHT).
 * Find the first 'MOVING' PWM and enter below.
 *
 * Current value: 0 (not yet measured — replace before competition!)
 */
#define RIGHT_MOTOR_DEAD_PWM    0       /* TODO: fill from Test 3.3 State 7 */

/**
 * [OK] EMA coefficient for encoder velocity LPF.
 * 0.25 tested in Phase 2 — smooth output, acceptable lag at 20 Hz update.
 * Range: 0.20 (smoother) – 0.40 (more responsive).
 */
#define VELOCITY_LPF_ALPHA      0.25f

/** @} */ // end PerWheelCalib

/* ═══════════════════════════════════════════════════════════════════════════
 *  Motor Speed Characterization
 *  (Measured in Phase 2, battery ~7.4 V nominal)
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @defgroup MotorCharacterization Motor Speed vs PWM (Measured)
 *
 * Recorded from Phase 2 Serial output (forward, wheels lifted, ~7.4V):
 *
 *   PWM   | L mm/s  | R mm/s  | Avg
 *   ------+---------+---------+------
 *   1500  | 178.0   | 179.9   | 179.0  (first measurement)
 *
 * Derived KFF = 1500 / 179 = 8.4  (set in speed_controller.cpp)
 *
 * TODO: Add more rows when running Test 3.4 characterization:
 *   Run Phase 2 State 1 (FORWARD) at different PWM values and record
 *   the L_filt and R_filt values from Serial.
 *
 *   Suggested PWMs to test: 500, 800, 1000, 1500, 2000, 2500, 3000
 *
 *  @{
 */

/**
 * [TODO] Measure WHEEL_BASE_MM with calipers.
 * Place robot wheels against a flat surface.
 * Measure center-to-center distance between left and right wheel contact
 * patches (not the outer edge of the wheel — the centre of the tread).
 *
 * This value directly controls turning radius accuracy.
 * 1mm error in wheel base ≈ 1.2° error per 90° turn.
 */
/** @} */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Robot Chassis Geometry
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @defgroup ChassisGeometry Robot Chassis Dimensions
 *  @{
 */

/**
 * Distance between left and right wheel contact points (mm).
 * Measure this precisely — it directly affects turning accuracy.
 *
 * TODO: Measure actual wheel base on your robot and update this value.
 */
#define WHEEL_BASE_MM           75.0f   /**< TODO: Measure and update       */

/**
 * Distance from wheel axle to front sensor mounting point (mm).
 *
 * TODO: Measure from wheel axle center to front ToF sensor face.
 */
#define SENSOR_FRONT_OFFSET_MM  30.0f   /**< TODO: Measure and update       */

/** @} */ // end ChassisGeometry

/* ═══════════════════════════════════════════════════════════════════════════
 *  PWM Configuration
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @defgroup PWMConfig PWM Timer Configuration
 *  TIM1 PWM parameters for motor control.
 *  @{
 */

/**
 * PWM auto-reload value (determines frequency).
 * At 84MHz APB2 clock, PSC=0, ARR=4199 → 20kHz PWM.
 */
#define PWM_MAX                 4199    /**< TIM1 ARR value (0–4199 range)  */

/**
 * PWM frequency (Hz).
 * f_PWM = f_CLK / (PSC+1) / (ARR+1) = 84MHz / 1 / 4200 = 20kHz
 */
#define PWM_FREQUENCY_HZ        20000

/** @} */ // end PWMConfig

/* ═══════════════════════════════════════════════════════════════════════════
 *  Control Loop Timing
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @defgroup ControlTiming Control Loop Parameters
 *  @{
 */

#define CONTROL_LOOP_FREQ_HZ    1000    /**< Main control loop frequency    */
#define CONTROL_LOOP_DT_S       0.001f  /**< Control loop period (seconds)  */
#define CONTROL_LOOP_DT_MS      1       /**< Control loop period (ms)       */

/** @} */ // end ControlTiming

/* ═══════════════════════════════════════════════════════════════════════════
 *  Battery Monitoring
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @defgroup BatteryConfig Battery Parameters
 *  2S LiPo battery monitoring thresholds.
 *  @{
 */

#define BATTERY_FULL_MV         8400    /**< 2S LiPo full charge (mV)      */
#define BATTERY_NOMINAL_MV      7400    /**< 2S LiPo nominal voltage (mV)  */
#define BATTERY_LOW_MV          6600    /**< Low battery warning (mV)      */
#define BATTERY_CRITICAL_MV     6000    /**< Critical — stop robot (mV)    */

/**
 * Voltage divider ratio.
 * If using a 10kΩ / 10kΩ divider: ratio = 2.0
 * V_battery = V_adc × DIVIDER_RATIO
 *
 * TODO: Update to match your actual voltage divider.
 */
#define BATTERY_DIVIDER_RATIO   2.0f    /**< TODO: Match your divider      */

/** @} */ // end BatteryConfig

#endif /* ROBOT_CONFIG_H */
