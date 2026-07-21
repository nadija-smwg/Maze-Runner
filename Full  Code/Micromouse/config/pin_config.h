/**
 * @file pin_config.h
 * @brief Centralized pin assignment map for the Micromouse robot.
 *
 * All hardware pin definitions live here. Every module that touches a
 * physical pin must include this file instead of hard-coding pin numbers.
 *
 * Platform: STM32F411CEU6 (Black Pill)
 * Framework: Arduino Core for STM32
 *
 * @note Pin assignments marked TODO need to be filled in once the PCB
 *       layout is finalized.
 */

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

/* ═══════════════════════════════════════════════════════════════════════════
 *  Motor Driver — TB6612FNG
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @defgroup MotorPins Motor Driver Pin Assignments
 *  TB6612FNG H-bridge motor driver connections.
 *  @{
 */

/* Left Motor (Motor A) */
#define PIN_MOTOR_LEFT_PWM      PA8     /**< TIM1_CH1 — Left motor PWM      */
#define PIN_MOTOR_LEFT_IN1      PB12    /**< AIN1 — Left motor direction 1  */
#define PIN_MOTOR_LEFT_IN2      PB13    /**< AIN2 — Left motor direction 2  */

/* Right Motor (Motor B) */
#define PIN_MOTOR_RIGHT_PWM     PA9     /**< TIM1_CH2 — Right motor PWM     */
#define PIN_MOTOR_RIGHT_IN1     PB15    /**< BIN1 — Right motor direction 1 */
#define PIN_MOTOR_RIGHT_IN2     PA10    /**< BIN2 — Right motor direction 2 */

/* Standby */
#define PIN_MOTOR_STBY          PB14    /**< STBY — Motor driver enable     */

/** @} */ // end MotorPins

/* ═══════════════════════════════════════════════════════════════════════════
 *  Encoders — Quadrature (Hardware Timer Encoder Mode)
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @defgroup EncoderPins Encoder Pin Assignments
 *  N20 motor quadrature encoder connections.
 *  @{
 */

/* Left Encoder — TIM2 (32-bit timer) */
#define PIN_ENC_LEFT_A          PA0     /**< TIM2_CH1 — Left encoder A      */
#define PIN_ENC_LEFT_B          PA1     /**< TIM2_CH2 — Left encoder B      */

/* Right Encoder — TIM3 (16-bit timer) */
#define PIN_ENC_RIGHT_A         PA6     /**< TIM3_CH1 — Right encoder A     */
#define PIN_ENC_RIGHT_B         PA7     /**< TIM3_CH2 — Right encoder B     */

/** @} */ // end EncoderPins

/* ═══════════════════════════════════════════════════════════════════════════
 *  I2C Bus — Shared (MPU6050, SSD1306, VL53L0x)
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @defgroup I2CPins I2C Bus Pin Assignments
 *  Shared I2C1 bus for all I2C peripherals.
 *  @{
 */

#define PIN_I2C_SCL             PB8     /**< I2C1_SCL                       */
#define PIN_I2C_SDA             PB9     /**< I2C1_SDA                       */
#define I2C_CLOCK_SPEED         400000  /**< I2C Fast Mode (400kHz)         */

/** @} */ // end I2CPins

/* ═══════════════════════════════════════════════════════════════════════════
 *  IMU — MPU6050
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @defgroup IMUPins IMU Pin Assignments
 *  MPU6050 6-axis IMU on the shared I2C bus.
 *  @{
 */

#define MPU6050_I2C_ADDR        0x68    /**< Default I2C address (AD0=LOW)  */

/** @} */ // end IMUPins

/* ═══════════════════════════════════════════════════════════════════════════
 *  OLED — SSD1306 (128×64)
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @defgroup OLEDPins OLED Pin Assignments
 *  SSD1306 128×64 OLED on the shared I2C bus.
 *  @{
 */

#define SSD1306_I2C_ADDR        0x3C    /**< Default I2C address            */
#define OLED_WIDTH              128     /**< Display width in pixels        */
#define OLED_HEIGHT             64      /**< Display height in pixels       */

/** @} */ // end OLEDPins

/* ═══════════════════════════════════════════════════════════════════════════
 *  ToF Distance Sensors — VL53L0X (×5)
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @defgroup ToFPins VL53L0X XSHUT Pin Assignments
 *  Each VL53L0X sensor has an XSHUT pin for I2C address reassignment.
 *  All sensors share the same I2C bus; XSHUT is used to bring them
 *  online one at a time during initialization and assign unique addresses.
 *  @{
 */

/* TODO: Assign XSHUT pins once PCB layout is finalized */
#define PIN_TOF_XSHUT_FRONT         PB0     /**< TODO: Front sensor XSHUT       */
#define PIN_TOF_XSHUT_FRONT_LEFT    PB1     /**< TODO: Front-left sensor XSHUT  */
#define PIN_TOF_XSHUT_FRONT_RIGHT   PB2     /**< TODO: Front-right sensor XSHUT */
#define PIN_TOF_XSHUT_LEFT          PB3     /**< TODO: Left sensor XSHUT        */
#define PIN_TOF_XSHUT_RIGHT         PB4     /**< TODO: Right sensor XSHUT       */

/* Assigned I2C addresses (default is 0x29, reassigned during init) */
#define TOF_ADDR_FRONT              0x30    /**< Front sensor I2C address       */
#define TOF_ADDR_FRONT_LEFT         0x31    /**< Front-left sensor I2C address  */
#define TOF_ADDR_FRONT_RIGHT        0x32    /**< Front-right sensor I2C address */
#define TOF_ADDR_LEFT               0x33    /**< Left sensor I2C address        */
#define TOF_ADDR_RIGHT              0x34    /**< Right sensor I2C address       */

#define NUM_TOF_SENSORS             5       /**< Total number of ToF sensors    */

/** @} */ // end ToFPins

/* ═══════════════════════════════════════════════════════════════════════════
 *  Battery Monitoring — 2S LiPo
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @defgroup BatteryPins Battery Monitoring Pin Assignments
 *  ADC input from voltage divider on 2S LiPo.
 *  @{
 */

/* TODO: Assign battery voltage sense pin once hardware is finalized */
#define PIN_BATTERY_SENSE       PA4     /**< TODO: Battery ADC input pin    */

/** @} */ // end BatteryPins

/* ═══════════════════════════════════════════════════════════════════════════
 *  User Interface — Buttons, Buzzer, LEDs
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @defgroup UIPins User Interface Pin Assignments
 *  Buttons, buzzer, and LED indicators.
 *  @{
 */

/* TODO: Assign UI pins once hardware is finalized */
#define PIN_BUTTON_START        PB5     /**< TODO: Start button pin         */
#define PIN_BUTTON_MODE         PB6     /**< TODO: Mode select button pin   */

#define PIN_BUZZER              PB7     /**< TODO: Buzzer output pin        */

#define PIN_LED_STATUS          PC13    /**< Onboard LED (Black Pill)       */
#define PIN_LED_DEBUG           PA5     /**< TODO: External debug LED pin   */

/** @} */ // end UIPins

#endif /* PIN_CONFIG_H */
