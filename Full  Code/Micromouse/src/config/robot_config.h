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
