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

/**
 * @defgroup MotorSpecs Motor Mechanical Specifications
 *  N20 metal gear motor with integrated quadrature encoder.
 *
 *  DATASHEET vs MEASURED:
 *  - Datasheet claimed: 65:1 gear ratio, 1820 CPR
 *  - Measured (Test F.1, 1 revolution hand-count):
 *      LEFT  ~592 counts, RIGHT ~588 counts  → avg ≈ 590
 *  - Derived gear ratio: 590 / (7 PPR × 4 quad) ≈ 21:1
 *    (Most likely a 20:1 or 21:1 N20 variant, NOT 65:1)
 *
 *  Hand-rotation has ±5-10% single-revolution error.
 *  For higher accuracy: re-run Test F.1 for 10 full revolutions
 *  and divide: CPR = count / 10.
 *  @{
 */

#define GEAR_RATIO              21.0f   /**< Gearbox ratio — MEASURED (was 65) */
#define ENCODER_PPR             7.0f    /**< Pulses per revolution (raw)       */
#define ENCODER_QUADRATURE      4.0f    /**< Quadrature multiplier (×4)        */

/**
 * Counts Per Revolution (after gearing and quadrature decoding).
 * CPR = PPR × GEAR_RATIO × QUADRATURE = 7 × 21 × 4 = 588 counts/rev
 *
 * Per-wheel values (measured) are overridden below in LEFT/RIGHT_ENCODER_CPR.
 * The difference L=592 / R=588 is normal motor-to-motor variation.
 *
 * [ACTION] Re-verify with 10-revolution test for ±1% accuracy:
 *   Phase 2 → State 7 (LEFT) or 8 (RIGHT)
 *   Mark wheel → rotate EXACTLY 10 full turns → press BTN_MODE
 *   CPR = printed count / 10
 */
#define ENCODER_CPR             (ENCODER_PPR * GEAR_RATIO * ENCODER_QUADRATURE)

/** @} */ // end MotorSpecs

/* ═══════════════════════════════════════════════════════════════════════════
 *  Wheel Geometry
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @defgroup WheelGeometry Wheel Physical Dimensions
 *  @{
 */

/**
 * Wheel outer diameter (mm) — CALIBRATED via Test 4.3.
 * Push test: robot pushed 180mm, odometry showed 165mm.
 * Correction: diameter = 43.0 × (180/165) = 46.9 mm
 *
 * History:
 *   34.0 mm  — initial assumption (wrong)
 *   43.0 mm  — caliper measurement
 *   46.9 mm  — calibrated via Test 4.3 push test ✅
 *
 * To re-verify: push robot exactly 180mm, read X from Serial.
 *   new_diameter = 46.9 × (180 / X_shown)
 */
#define WHEEL_DIAMETER_MM       46.9f   /**< Wheel outer diameter — CALIBRATED via Test 4.3 */

/**
 * Wheel circumference (mm).
 * C = π × D = π × 46.9 ≈ 147.34 mm
 */
#define WHEEL_CIRCUMFERENCE_MM  (3.14159265358979f * WHEEL_DIAMETER_MM)

/**
 * Distance per encoder count (mm/count).
 * = Circumference / CPR ≈ 147.34 / 581 ≈ 0.2536 mm/count
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

/**
 * [MEASURED] Left wheel counts per revolution.
 * Measured via Test F.1 (1 hand-revolution): ~581 counts.
 * Derived gear ratio: 581 / (7 × 4) ≈ 20.75:1
 *
 * [ACTION] Re-verify accuracy: repeat Test F.1 for 10 full revolutions.
 *   CPR_left = count / 10  (reduces hand-rotation error from ±5% to ±0.5%)
 *
 * Tolerance for control: ±10 counts (±1.7%) acceptable.
 * Beyond ±20 counts heading will drift noticeably in turns.
 */
#define LEFT_ENCODER_CPR        581.0f   /* Measured (1-rev hand test) */

/**
 * [MEASURED] Right wheel counts per revolution.
 * Measured via Test F.1 (1 hand-revolution): ~581 counts.
 *
 * [ACTION] Re-verify with 10-revolution test same as above.
 */
#define RIGHT_ENCODER_CPR       581.0f   /* Measured (1-rev hand test) */

/**
 * [CALIBRATED] Left wheel effective outer diameter (mm).
 * Test 4.3 push calibration: pushed 180mm → showed 165mm
 * Correction: 43.0 × (180/165) = 46.9 mm
 *
 * To re-calibrate:
 *   Push robot exactly 180mm on flat floor.
 *   new_diameter = 46.9 × (180 / X_shown)
 */
#define LEFT_WHEEL_DIAMETER_MM  46.9f    /* Calibrated via Test 4.3 push test */

/**
 * [CALIBRATED] Right wheel effective outer diameter (mm).
 * Same correction as left: 46.9 mm.
 */
#define RIGHT_WHEEL_DIAMETER_MM 46.9f    /* Calibrated via Test 4.3 push test */

/** Left wheel circumference (mm) — derived: π × 46.9 ≈ 147.34 mm */
#define LEFT_WHEEL_CIRC_MM      (3.14159265f * LEFT_WHEEL_DIAMETER_MM)

/** Right wheel circumference (mm) — derived: π × 46.9 ≈ 147.34 mm */
#define RIGHT_WHEEL_CIRC_MM     (3.14159265f * RIGHT_WHEEL_DIAMETER_MM)

/** Left wheel distance per count (mm/count) — derived ≈ 0.2536 mm/count */
#define LEFT_MM_PER_COUNT       (LEFT_WHEEL_CIRC_MM  / LEFT_ENCODER_CPR)

/** Right wheel distance per count (mm/count) — derived ≈ 0.2536 mm/count */
#define RIGHT_MM_PER_COUNT      (RIGHT_WHEEL_CIRC_MM / RIGHT_ENCODER_CPR)

/**
 * [MEASURED] Left motor dead-zone PWM.
 * This is the minimum PWM duty cycle required to overcome static
 * friction and make the left wheel start spinning from a standstill.
 *
 * Obtained from Phase 2, State 5 (Dead-zone sweep).
 */
#define LEFT_MOTOR_DEAD_PWM     300      /* Measured from Test 3.3 State 5 */

/**
 * [MEASURED] Right motor dead-zone PWM.
 * Obtained from Phase 2, State 6 (Dead-zone sweep).
 * Note: Right motor is slightly more efficient (starts at 250 vs 300).
 */
#define RIGHT_MOTOR_DEAD_PWM    250      /* Measured from Test 3.3 State 6 */

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
 * [TODO] Distance between left and right wheel contact points (mm).
 * [MEASURED] Distance between the center of the left wheel tread
 * and the center of the right wheel tread (track width).
 *
 * Current: 95.3 mm
 *
 * [ACTION] Verify via turn test:
 *   If robot turns 4x90° and ends up short of 360°, actual base is SMALLER.
 *   If it turns too far, actual base is LARGER.
 */
#define WHEEL_BASE_MM           95.3f    /* Tuned via Test 4.4 after fixing wheel diameter */

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
 * [TODO] Voltage divider ratio for battery ADC.
 *
 * ANOMALY DETECTED: Serial shows ~910 mV but a 2S LiPo should read ~7400 mV.
 * This means the current ratio of 2.0 is WRONG for this hardware.
 *
 * To find the correct value:
 *   1. Measure actual battery voltage with multimeter: e.g. 7.82 V = 7820 mV
 *   2. Read the Serial output 'Bat:' value: e.g. 919 mV
 *   3. Correct ratio = measured_mV / serial_mV = 7820 / 919 = 8.51
 *   4. Update this value:
 *      #define BATTERY_DIVIDER_RATIO   8.51f
 *
 * Note: If using a 100kΩ + 33kΩ voltage divider:
 *   ratio = (100 + 33) / 33 = 4.03
 *
 * Current value: 2.0f (almost certainly wrong — verify before competition!)
 */
#define BATTERY_DIVIDER_RATIO   2.0f    /**< [TODO] Measure and correct!      */

/** @} */ // end BatteryConfig

#endif /* ROBOT_CONFIG_H */
