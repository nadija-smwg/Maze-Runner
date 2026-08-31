/**
 * @file calibration.cpp
 * @brief Calibration implementation.
 * @see calibration.h
 */

#include "calibration.h"
#include "mpu6050.h"
#include "../display/oled_driver.h"
#include "../hardware/button.h"
#include "../hardware/battery.h"
#include <Arduino.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  calibrate_all — quick calibration, no warm-up (for testing)
 * ═══════════════════════════════════════════════════════════════════════════ */

bool calibrate_all(void)
{
    oled_clear();
    oled_print(0,  0, "Calibrating IMU");
    oled_print(0, 15, "Hands off!");
    oled_update();

    /*
     * Wait 2 seconds:
     *  - User releases robot
     *  - Vibrations from button press damp out
     *  - MPU6050 DLPF settles
     */
    delay(2000);

    oled_print(0, 30, "Calibrating...");
    oled_update();

    calibrate_gyro();

    oled_clear();
    oled_print(0,  0, "Calibration");
    oled_print(0, 15, "Complete!");
    oled_update();
    delay(1000);

    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  calibrate_gyro — core 1000-sample gyro calibration
 * ═══════════════════════════════════════════════════════════════════════════ */

void calibrate_gyro(void)
{
    Serial.println();
    Serial.println("================================");
    Serial.println("MPU6050 Calibration");
    Serial.println("Keep the robot completely still.");
    Serial.println("Calibrating 1000 samples...");
    Serial.println("================================");

    mpu6050_calibrate_gyro(1000);

    Serial.print("[Calib] GyroZ bias (raw LSB): ");
    Serial.println(mpu6050_get_gyro_bias_z(), 2);
    Serial.println("[Calib] If |GyroZ bias| > 200 raw LSB, re-run calibration.");
    Serial.println();
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  calibrate_with_warmup — competition-ready warm-up + calibration
 * ═══════════════════════════════════════════════════════════════════════════ */

void calibrate_with_warmup(uint16_t warmup_seconds)
{
    Serial.println();
    Serial.println("========================================");
    Serial.println("Competition Boot: Warm-up + Calibration");
    Serial.print  ("Warm-up duration: ");
    Serial.print  (warmup_seconds);
    Serial.println(" seconds");
    Serial.println("Press BTN_START to skip warm-up.");
    Serial.println("========================================");

    /* ── Stage 1: Warm-up countdown ─────────────────────────────────────── */
    for (uint16_t remaining = warmup_seconds; remaining > 0; remaining--)
    {
        /* Update button state so BTN_START skip works */
        button_update();

        if (button_just_pressed(BUTTON_START))
        {
            Serial.println("[Warmup] Skip requested — calibrating now.");
            break;
        }

        /* Update OLED every second */
        char buf[24];
        oled_clear();
        oled_print(0,  0, "Warming up...");
        oled_print(0, 13, "Keep still!");

        sprintf(buf, "%u sec remaining", remaining);
        oled_print(0, 26, buf);

        uint16_t bat_mv = battery_get_voltage_mv();
        sprintf(buf, "Bat: %u mV", bat_mv);
        oled_print(0, 45, buf);

        oled_update();

        /* Print progress every 10 seconds */
        if (remaining % 10 == 0)
        {
            Serial.print("[Warmup] ");
            Serial.print(remaining);
            Serial.println(" sec remaining...");
        }

        delay(1000);
    }

    /* ── Stage 2: Calibrate ──────────────────────────────────────────────── */
    oled_clear();
    oled_print(0,  0, "Calibrating...");
    oled_print(0, 15, "Do not move!");
    oled_update();

    Serial.println("[Warmup] Complete. Running calibration...");
    calibrate_gyro();

    /* ── Stage 3: Show result and wait for START ─────────────────────────── */
    float bias_z = mpu6050_get_gyro_bias_z();
    bool  good   = (bias_z > -200.0f && bias_z < 200.0f);

    char buf[24];
    oled_clear();

    if (good)
    {
        oled_print(0,  0, "Calibrated! OK");
        sprintf(buf, "BiasZ: %.1f", bias_z);
        oled_print(0, 15, buf);
        oled_print(0, 30, "READY");
        oled_print(0, 45, "Press START");
        oled_update();

        Serial.println("[Calib] Sensor ready. Press BTN_START to run.");
    }
    else
    {
        oled_print(0,  0, "CALIB FAILED!");
        sprintf(buf, "BiasZ: %.1f", bias_z);
        oled_print(0, 15, buf);
        oled_print(0, 30, "Press START to");
        oled_print(0, 45, "retry calib.");
        oled_update();

        Serial.println("[Calib] WARNING: Bias out of range — robot may have moved!");
        Serial.println("[Calib] Press BTN_START to re-calibrate.");

        /* Wait for START then re-run calibration */
        while (true)
        {
            button_update();
            if (button_just_pressed(BUTTON_START))
            {
                calibrate_with_warmup(0); /* 0 = skip warm-up, calib only */
                return;
            }
            delay(10);
        }
    }

    /* Wait for START button before allowing robot to run */
    while (true)
    {
        button_update();
        if (button_just_pressed(BUTTON_START))
            break;
        delay(10);
    }
}

void calibrate_distance_sensors(void)
{
    /* Future implementation — per-sensor offset calibration */
}

void calibrate_encoders(void)
{
    /* Future implementation — CPR and diameter measurement */
}
