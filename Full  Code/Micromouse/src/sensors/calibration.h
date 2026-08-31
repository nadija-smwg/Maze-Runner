/**
 * @file calibration.h
 * @brief Routines for calibrating various sensors.
 *
 * Competition Calibration Strategy
 * ─────────────────────────────────
 * The MPU6050 gyroscope has a temperature-dependent bias (thermal drift).
 * When cold, the bias is different from when the chip has been running for
 * 1–2 minutes. If you calibrate while cold and then run immediately, the
 * residual bias causes yaw drift during the maze run.
 *
 * Solution — Two-stage boot sequence:
 *
 *   Stage 1: WARM-UP  (~90 seconds)
 *     Power on → OLED counts down 90 s → sensor reaches stable temperature.
 *     Use this time to place the robot at the start cell.
 *
 *   Stage 2: CALIBRATE (2 seconds)
 *     Auto-triggers at end of warm-up → samples 1000 readings from
 *     the now-stable sensor → bias captured at operating temperature.
 *
 *   Result: GzFilt ≈ 0.00 °/s immediately after calibration.
 *           Drift corrector handles any residual drift in <2 sec at 1 kHz.
 *
 * Competition checklist:
 *   [ ] Power on robot at least 90 seconds before your run slot
 *   [ ] Keep robot still during warm-up + calibration (15 seconds total)
 *   [ ] OLED shows "READY" → press BTN_START to begin maze run
 *   [ ] If bumped during warm-up → press BTN_START to force re-calibrate
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Run full startup calibration sequence (no warm-up).
 *
 * Waits 2 seconds for vibration to settle, then calibrates.
 * Use during testing. For competition, use calibrate_with_warmup().
 *
 * @return true if successful
 */
bool calibrate_all(void);

/**
 * @brief Calibrate IMU gyro zero-offsets.
 *
 * Takes 1000 samples at 2ms intervals (~2 seconds total).
 * Robot must be completely still.
 */
void calibrate_gyro(void);

/**
 * @brief Competition-ready warm-up + calibration sequence.
 *
 * Shows a live countdown on the OLED while the MPU6050 warms up to its
 * operating temperature. Automatically triggers calibration at the end.
 *
 * OLED display during warm-up:
 *   Line 0: "Warming up..."
 *   Line 1: "Keep still!"
 *   Line 2: "XX sec remaining"
 *   Line 3: "Bat: XXXX mV"
 *
 * After warm-up, calibrates and shows:
 *   "Calibrated!"  "BiasZ: X.XX"  "READY - Press START"
 *
 * Pressing BTN_START during warm-up skips the countdown and
 * immediately triggers calibration (useful if robot was already warm).
 *
 * @param warmup_seconds  How long to warm up (recommended: 90).
 *                        Set to 0 to skip warm-up (testing only).
 */
void calibrate_with_warmup(uint16_t warmup_seconds);

/**
 * @brief Calibrate ToF distance sensors against walls.
 *
 * Placed in a known position in a cell, measure and store offsets
 * for the distance sensors.
 */
void calibrate_distance_sensors(void);

/**
 * @brief Tune encoder wheel size offsets (advanced).
 */
void calibrate_encoders(void);

#endif /* CALIBRATION_H */
