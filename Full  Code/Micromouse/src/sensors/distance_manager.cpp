/**
 * @file distance_manager.cpp
 * @brief Distance manager implementation.
 * @see distance_manager.h
 *
 * Filtering pipeline per sensor (called in distance_manager_update()):
 *
 *   vl53l0x_read_distance_mm()
 *           │
 *   tof_filter_process()    ← validity + calibration + median-3
 *           │                  + jump rejection + EMA
 *   _distances_mm[id]
 *           │
 *   update_wall_states()    ← hysteresis state machine per side
 */

#include "distance_manager.h"
#include "tof_filter.h"
#include "vl53l0x.h"
#include "../config/pin_config.h"
#include <Arduino.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private State
 * ═══════════════════════════════════════════════════════════════════════════ */

/** Hardware sensor descriptors (XSHUT pin + I2C address). */
static VL53L0X_Sensor _sensors[TOF_COUNT];

/** Latest filtered distance for each sensor (mm). */
static uint16_t _distances_mm[TOF_COUNT] = {
    TOF_MAX_DIST_MM, TOF_MAX_DIST_MM, TOF_MAX_DIST_MM,
    TOF_MAX_DIST_MM, TOF_MAX_DIST_MM
};

/** Per-sensor filter state (one instance per ToF sensor). */
static ToF_FilterState _filter[TOF_COUNT];

/** Hysteresis wall-state flags (true = wall currently detected). */
static bool _wall_left  = false;
static bool _wall_right = false;
static bool _wall_front = false;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Update one hysteresis wall-state flag.
 *
 * Implements the two-threshold hysteresis:
 *   - NO_WALL → WALL  when distance < enter_mm
 *   - WALL    → NO_WALL when distance > exit_mm
 *
 * @param current_state  Current wall flag value
 * @param distance_mm    Latest filtered distance
 * @param enter_mm       Threshold to enter WALL state
 * @param exit_mm        Threshold to exit WALL state
 * @return               Updated wall flag
 */
static bool hysteresis_update(bool current_state,
                              uint16_t distance_mm,
                              float enter_mm,
                              float exit_mm)
{
    if (!current_state)
    {
        /* Currently NO_WALL: enter WALL if distance drops below enter threshold */
        if ((float)distance_mm < enter_mm)
            return true;
    }
    else
    {
        /* Currently WALL: exit WALL if distance rises above exit threshold */
        if ((float)distance_mm > exit_mm)
            return false;
    }
    return current_state;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialization
 * ═══════════════════════════════════════════════════════════════════════════ */

void distance_manager_init(void)
{
    /* 1. Populate sensor array with XSHUT pins and I2C addresses */
    _sensors[TOF_FRONT].xshut_pin      = PIN_TOF_XSHUT_FRONT;
    _sensors[TOF_FRONT].i2c_address    = TOF_ADDR_FRONT;

    _sensors[TOF_FRONT_LEFT].xshut_pin   = PIN_TOF_XSHUT_FRONT_LEFT;
    _sensors[TOF_FRONT_LEFT].i2c_address = TOF_ADDR_FRONT_LEFT;

    _sensors[TOF_FRONT_RIGHT].xshut_pin   = PIN_TOF_XSHUT_FRONT_RIGHT;
    _sensors[TOF_FRONT_RIGHT].i2c_address = TOF_ADDR_FRONT_RIGHT;

    _sensors[TOF_LEFT].xshut_pin      = PIN_TOF_XSHUT_LEFT;
    _sensors[TOF_LEFT].i2c_address    = TOF_ADDR_LEFT;

    _sensors[TOF_RIGHT].xshut_pin     = PIN_TOF_XSHUT_RIGHT;
    _sensors[TOF_RIGHT].i2c_address   = TOF_ADDR_RIGHT;

    /* 2. Initialize filter states (offsets default to 0) */
    for (int i = 0; i < TOF_COUNT; i++)
    {
        _filter[i].offset = 0;          /* set before init to preserve offset */
        tof_filter_init(&_filter[i]);
    }

    /* 3. Initialize VL53L0X hardware (XSHUT sequencing + I2C address assign) */
    uint8_t count = vl53l0x_init_all(_sensors, TOF_COUNT);
    Serial.print("[Distance] Init: ");
    Serial.print(count);
    Serial.println("/5 sensors OK");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Update
 * ═══════════════════════════════════════════════════════════════════════════ */

void distance_manager_update(void)
{
    for (int i = 0; i < TOF_COUNT; i++)
    {
        /* Read raw distance from hardware */
        uint16_t raw = vl53l0x_read_distance_mm(&_sensors[i]);

        /* Run full filter pipeline — validity, calibration, median, jump, EMA */
        _distances_mm[i] = tof_filter_process(&_filter[i], raw);
    }

    /* Update hysteresis wall-state flags */
    _wall_left  = hysteresis_update(_wall_left,
                                    _distances_mm[TOF_LEFT],
                                    WALL_ENTER_SIDE_MM,
                                    WALL_EXIT_SIDE_MM);

    _wall_right = hysteresis_update(_wall_right,
                                    _distances_mm[TOF_RIGHT],
                                    WALL_ENTER_SIDE_MM,
                                    WALL_EXIT_SIDE_MM);

    _wall_front = hysteresis_update(_wall_front,
                                    _distances_mm[TOF_FRONT],
                                    WALL_ENTER_FRONT_MM,
                                    WALL_EXIT_FRONT_MM);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Distance Accessors
 * ═══════════════════════════════════════════════════════════════════════════ */

uint16_t distance_get_filtered_mm(DistanceSensorID id)
{
    if ((int)id < TOF_COUNT)
        return _distances_mm[id];
    return TOF_MAX_DIST_MM;
}

uint16_t distance_get_mm(DistanceSensorID id)
{
    return distance_get_filtered_mm(id);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Wall Detection — Hysteresis-Based
 * ═══════════════════════════════════════════════════════════════════════════ */

bool distance_has_wall_left(void)
{
    return _wall_left;
}

bool distance_has_wall_right(void)
{
    return _wall_right;
}

bool distance_has_wall_front(void)
{
    return _wall_front;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  PID Error Helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

float distance_get_centering_error(void)
{
    bool wall_l = distance_has_wall_left();
    bool wall_r = distance_has_wall_right();

    /*
     * Target distance from robot center to each side wall.
     * Standard maze cell = 180 mm. Robot width ≈ 80 mm.
     * Target clearance ≈ 50 mm on each side.
     * TODO: Tune TARGET_WALL_DIST_MM based on actual robot width.
     */
    const float TARGET_WALL_DIST_MM = 50.0f;

    if (wall_l && wall_r)
    {
        /*
         * Both walls present: error = right_dist - left_dist
         * Positive error → robot shifted left → correct rightward
         * Example: L=40, R=60 → error = +20 (need to move right)
         */
        return (float)_distances_mm[TOF_RIGHT] -
               (float)_distances_mm[TOF_LEFT];
    }
    else if (wall_l)
    {
        /*
         * Only left wall: error = target - left_dist
         * Positive error → too close to left wall → correct rightward
         */
        return TARGET_WALL_DIST_MM - (float)_distances_mm[TOF_LEFT];
    }
    else if (wall_r)
    {
        /*
         * Only right wall: error = right_dist - target
         * Negative error → too close to right wall → correct leftward
         */
        return (float)_distances_mm[TOF_RIGHT] - TARGET_WALL_DIST_MM;
    }

    return 0.0f;
}

float distance_get_front_alignment_error(void)
{
    /*
     * If both front-diagonal sensors see a wall nearby, the difference
     * between them indicates how much the robot is angled relative to
     * the wall. Positive = angled to the right.
     */
    const float FRONT_DIAG_WALL_THRESH = 150.0f;

    if ((float)_distances_mm[TOF_FRONT_LEFT]  < FRONT_DIAG_WALL_THRESH &&
        (float)_distances_mm[TOF_FRONT_RIGHT] < FRONT_DIAG_WALL_THRESH)
    {
        return (float)_distances_mm[TOF_FRONT_RIGHT] -
               (float)_distances_mm[TOF_FRONT_LEFT];
    }

    return 0.0f;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Calibration
 * ═══════════════════════════════════════════════════════════════════════════ */

void distance_set_sensor_offset(DistanceSensorID id, int16_t offset)
{
    if ((int)id < TOF_COUNT)
        tof_filter_set_offset(&_filter[id], offset);
}
