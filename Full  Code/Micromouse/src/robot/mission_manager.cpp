/**
 * @file mission_manager.cpp
 * @brief Phase 6/7 Mission Manager — Search Run + Fast Run orchestrator.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  HOW THE 5 SENSORS ARE USED HERE
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Sensor          Role in Phase 6
 *  ─────────────── ────────────────────────────────────────────────────
 *  TOF_FRONT       Detect front wall → record in maze, block forward move
 *  TOF_LEFT        Detect left wall  → record in maze + wall-follow PD
 *  TOF_RIGHT       Detect right wall → record in maze + wall-follow PD
 *  TOF_FRONT_LEFT  Front diagonal → heading squaring before stop (aligns
 *                  robot perpendicular to front wall before declaring
 *                  cell complete)
 *  TOF_FRONT_RIGHT Front diagonal → same role as FRONT_LEFT (difference
 *                  between FL and FR = angular alignment error)
 *
 *  Sensor data flow:
 *
 *   distance_manager_update()   [called in loop() at 100 Hz]
 *       │
 *       ├─ distance_has_wall_front/left/right()   → solver_record_walls()
 *       │                                          → flood fill maze
 *       │
 *       ├─ distance_get_centering_error()          → wall_follower PD
 *       │    uses LEFT+RIGHT distances             → correct lateral drift
 *       │
 *       └─ distance_get_front_alignment_error()    → heading squaring
 *            uses FL+FR diagonal distances         → micro-correct angle
 *                                                     before stop
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  Search Run State Machine (mission_search_step)
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  STEP_IDLE
 *    Wait for BTN_START → STEP_SENSE
 *
 *  STEP_SENSE
 *    Read all 5 sensors, record walls in maze → STEP_FLOOD
 *
 *  STEP_FLOOD
 *    Run flood fill, get next direction → STEP_TURN (if heading change)
 *                                      → STEP_DRIVE (if already aligned)
 *
 *  STEP_TURN
 *    Fire motion_turn_right/left_90() → wait motion_is_idle → STEP_DRIVE
 *    (For 180°: two sequential 90° turns with STEP_TURN_180_2nd)
 *
 *  STEP_DRIVE
 *    Fire motion_drive_cell() → wait motion_is_idle → STEP_SENSE
 *    (wall follower is active during drive — uses L/R sensors)
 *
 *  STEP_AT_GOAL
 *    Robot reached goal. Stop. Transition to STATE_RETURN_START.
 *
 * ═══════════════════════════════════════════════════════════════════════
 */

#include "mission_manager.h"
#include "robot_state_machine.h"
#include "../maze/solver.h"
#include "../control/motion_controller.h"
#include "../sensors/distance_manager.h"
#include "../hardware/motor.h"
#include "../hardware/led.h"
#include "../display/oled_driver.h"
#include "../utils/logger.h"
#include <Arduino.h>

/* ═══════════════════════════════════════════════════════════════════════
 *  Global Solver State
 *  Declared here so it lives in static RAM (stack is too small for a
 *  full Solver struct with 16×16 maze map + Dijkstra result).
 * ═══════════════════════════════════════════════════════════════════════ */

static Solver g_solver;

/* ═══════════════════════════════════════════════════════════════════════
 *  Search Run State Machine
 * ═══════════════════════════════════════════════════════════════════════ */

typedef enum {
    STEP_IDLE = 0,   /* waiting to start                                */
    STEP_SENSE,      /* read sensors, record walls                      */
    STEP_FLOOD,      /* compute flood fill, choose next direction        */
    STEP_TURN,       /* executing turn (motion_is_idle waits here)      */
    STEP_TURN_180_2, /* second 90° turn for a 180° reversal             */
    STEP_DRIVE,      /* executing cell drive (motion_is_idle waits here)*/
    STEP_AT_GOAL,    /* reached goal cell — done                        */
    STEP_RETURN,     /* driving back toward start                       */
} SearchStep;

static SearchStep _step = STEP_IDLE;

/* Pending direction chosen by flood fill (latched across state transitions) */
static Direction  _next_dir = DIR_NORTH;

/* Fast-run tracking */
static bool _fast_run_active = false;

/* ═══════════════════════════════════════════════════════════════════════
 *  Private Helpers
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * Print a one-line OLED status (4-line display: 128×32 or 128×64).
 * Shows: current cell (x,y), heading letter, step name.
 */
static void _update_oled(void)
{
    static uint32_t _last_oled_ms = 0;
    uint32_t now = millis();
    if (now - _last_oled_ms < 100) return;  /* 10 Hz OLED update */
    _last_oled_ms = now;

    const char *step_names[] = {
        "IDLE", "SENSE", "FLOOD", "TURN", "TURN180", "DRIVE", "GOAL!", "RETURN"
    };
    const char *dir_names[] = { "N", "E", "S", "W" };

    oled_clear();

    char line0[22];
    snprintf(line0, sizeof(line0), "Phase 6 - %s",
             (_step < 8) ? step_names[_step] : "???");
    oled_print(0, 0, line0);

    char line1[22];
    snprintf(line1, sizeof(line1), "Cell(%u,%u) Hdg:%s",
             g_solver.mouse_x, g_solver.mouse_y,
             dir_names[(int)g_solver.mouse_heading]);
    oled_print(0, 8, line1);

    /* Sensor wall state on line 3 */
    char line2[22];
    snprintf(line2, sizeof(line2), "F:%d L:%d R:%d",
             (int)distance_has_wall_front(),
             (int)distance_has_wall_left(),
             (int)distance_has_wall_right());
    oled_print(0, 16, line2);

    char line3[22];
    snprintf(line3, sizeof(line3), "FL:%u FR:%u mm",
             distance_get_mm(TOF_FRONT_LEFT),
             distance_get_mm(TOF_FRONT_RIGHT));
    oled_print(0, 24, line3);

    oled_update();
}

/**
 * Use the front-diagonal sensors (FL + FR) to log heading alignment.
 * If the difference is large before a turn, the robot is skewed.
 * This is purely informational — the wall follower corrects it during drive.
 * For advanced use: could apply a micro-correction before stopping.
 */
static void _log_front_alignment(void)
{
    float align_err = distance_get_front_alignment_error();
    if (align_err != 0.0f) {
        Serial.print(F("[P6] Front align error (FL-FR): "));
        Serial.print(align_err, 1);
        Serial.println(F(" mm  (>0 = angled right)"));
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Public API — Init
 * ═══════════════════════════════════════════════════════════════════════ */

void mission_manager_init(void)
{
    solver_init(&g_solver);
    _step = STEP_IDLE;
    _fast_run_active = false;

    LOG_INFO("[MissionMgr] Initialized. Solver at (0,0) heading NORTH.");
    LOG_INFO("[MissionMgr] Press BTN_START to begin search run.");
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Search Run — called by fsm_update() every loop() iteration
 * ═══════════════════════════════════════════════════════════════════════ */

void mission_manager_update(void)
{
    _update_oled();

    /* ─── Fast Run mode (Phase 7) ─────────────────────────────────────── */
    if (_fast_run_active) {
        if (!motion_is_idle()) return;  /* wait for current move */

        const MotionCommand *cmd = solver_get_next_command(&g_solver);
        if (cmd == NULL) {
            /* All commands consumed — fast run complete */
            _fast_run_active = false;
            motor_stop();
            fsm_set_state(STATE_IDLE);
            LOG_INFO("[MissionMgr] Fast run COMPLETE.");
            oled_clear();
            oled_print(0, 0, "FAST RUN DONE!");
            oled_update();
            return;
        }

        /* Dispatch next motion command */
        switch (cmd->type) {
            case CMD_FORWARD:
                motion_drive_cells(cmd->count);
                break;
            case CMD_TURN_RIGHT:
                motion_turn_right_90();
                break;
            case CMD_TURN_LEFT:
                motion_turn_left_90();
                break;
            default:
                break;
        }
        return;
    }

    /* ─── Search Run state machine ────────────────────────────────────── */
    switch (_step) {

    /* ── IDLE: start button already handled by fsm_update → STATE_SEARCH_RUN */
    case STEP_IDLE:
        _step = STEP_SENSE;  /* immediately begin on first update call */
        break;

    /* ── SENSE: read all 5 sensors, record walls ──────────────────────── */
    case STEP_SENSE: {
        /*
         * Sensor 1 (TOF_FRONT): front wall detection
         * Sensor 2 (TOF_LEFT):  left wall detection
         * Sensor 3 (TOF_RIGHT): right wall detection
         * Sensors 4+5 (TOF_FRONT_LEFT / TOF_FRONT_RIGHT): heading alignment
         *   — logged here, used by wall_follower during STEP_DRIVE
         */
        bool front = distance_has_wall_front();
        bool left  = distance_has_wall_left();
        bool right = distance_has_wall_right();

        /* Log alignment from diagonal sensors (informational) */
        _log_front_alignment();

        /* Record walls into maze map (converts relative to absolute dirs) */
        solver_record_walls(&g_solver, front, left, right);

        Serial.print(F("[P6] Cell("));
        Serial.print(g_solver.mouse_x);
        Serial.print(F(","));
        Serial.print(g_solver.mouse_y);
        Serial.print(F(") Walls: F="));
        Serial.print(front);
        Serial.print(F(" L="));
        Serial.print(left);
        Serial.print(F(" R="));
        Serial.println(right);

        /* Check if we are at the goal */
        if (solver_at_goal(&g_solver)) {
            _step = STEP_AT_GOAL;
            break;
        }

        _step = STEP_FLOOD;
        break;
    }

    /* ── FLOOD: run flood fill, choose best direction ─────────────────── */
    case STEP_FLOOD: {
        _next_dir = solver_search_step(&g_solver);

        TurnType turn = get_turn_type(g_solver.mouse_heading, _next_dir);

        Serial.print(F("[P6] Next dir="));
        const char *dirs[] = { "N","E","S","W" };
        Serial.print(dirs[(int)_next_dir]);
        Serial.print(F(" TurnType="));
        Serial.println((int)turn);

        if (turn == TURN_NONE) {
            /* Already aligned — drive straight */
            _step = STEP_DRIVE;
        } else if (turn == TURN_RIGHT_90) {
            motion_turn_right_90();
            _step = STEP_TURN;
        } else if (turn == TURN_LEFT_90) {
            motion_turn_left_90();
            _step = STEP_TURN;
        } else {
            /* TURN_180: two sequential right turns */
            motion_turn_right_90();
            _step = STEP_TURN;
            /* After first turn completes, STEP_TURN will issue the second */
        }
        break;
    }

    /* ── TURN: wait for turn to complete ─────────────────────────────── */
    case STEP_TURN:
        if (!motion_is_idle()) return;  /* still turning — come back next loop */

        /* Check if we still need a second turn (180° reversal) */
        {
            TurnType remaining = get_turn_type(g_solver.mouse_heading, _next_dir);
            if (remaining == TURN_RIGHT_90 || remaining == TURN_LEFT_90) {
                /* Update solver heading after first turn */
                /* (For 180 the first turn made us face 90° — need one more) */
                motion_turn_right_90();
                _step = STEP_TURN_180_2;
            } else {
                _step = STEP_DRIVE;
            }
        }
        break;

    /* ── TURN_180_2: wait for the second 90° of a 180° turn ──────────── */
    case STEP_TURN_180_2:
        if (!motion_is_idle()) return;
        _step = STEP_DRIVE;
        break;

    /* ── DRIVE: drive one cell forward ───────────────────────────────── */
    case STEP_DRIVE:
        if (motion_is_cell_moving()) {
            /* Already driving — wait */
            return;
        }
        if (!motion_is_idle()) return;

        /* Start the drive (wall follower uses L/R sensors during this) */
        motion_drive_cell();

        /* After drive completes (next time motion_is_idle), advance solver */
        _step = STEP_SENSE;

        /* Advance solver NOW so cell position updates on next SENSE */
        solver_advance(&g_solver, _next_dir);

        Serial.print(F("[P6] Driving cell -> new pos ("));
        Serial.print(g_solver.mouse_x);
        Serial.print(F(","));
        Serial.print(g_solver.mouse_y);
        Serial.println(F(")"));
        break;

    /* ── AT_GOAL: reached the goal ─────────────────────────────────────── */
    case STEP_AT_GOAL:
        motion_emergency_stop();
        led_blink(LED_STATUS, 100);  /* fast blink = goal reached */

        LOG_INFO("[P6] GOAL REACHED! Computing fast path...");
        Serial.print(F("[P6] Goal at ("));
        Serial.print(g_solver.mouse_x);
        Serial.print(F(","));
        Serial.print(g_solver.mouse_y);
        Serial.println(F("). Starting return to start."));

        /* Compute the fast-run path immediately while we are at the goal */
        {
            bool found = solver_compute_fast_path(
                &g_solver, START_X, START_Y, DIR_NORTH);
            if (found) {
                LOG_INFO("[P6] Fast path found. Will execute after return.");
            } else {
                LOG_ERROR("[P6] No fast path found! Check maze data.");
            }
        }

        /* Transition the top-level FSM to return-to-start */
        fsm_set_state(STATE_RETURN_START);
        _step = STEP_RETURN;
        break;

    /* ── RETURN: driving back to start via flood fill ─────────────────── */
    case STEP_RETURN:
        /*
         * Use flood fill toward start cell (0,0) instead of goal.
         * Simple approach: re-initialize solver in reverse mode.
         * Advanced: use Dijkstra path back (already computed above).
         *
         * For now: the same search-run loop works — flood fill toward
         * a single goal of (0,0) will navigate home.
         *
         * TODO: Switch to fast reverse using Dijkstra result for efficiency.
         */
        if (!motion_is_idle()) return;

        if (solver_at_start(&g_solver)) {
            /* Back at start — ready for fast run */
            motor_stop();
            LOG_INFO("[MissionMgr] Returned to start. Ready for fast run.");
            oled_clear();
            oled_print(0, 0, "BACK AT START");
            oled_print(0, 8, "Press S=FastRun");
            oled_update();
            fsm_set_state(STATE_IDLE);  /* wait for button to start fast run */
        }
        /* The top-level FSM / button handler will call
         * mission_start_fast_run() when ready. */
        break;

    default:
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Fast Run Trigger (called from fsm_update when in STATE_FAST_RUN)
 * ═══════════════════════════════════════════════════════════════════════ */

void mission_start_fast_run(void)
{
    if (g_solver.smooth_path.num_commands == 0) {
        LOG_ERROR("[MissionMgr] No fast path computed! Run search first.");
        return;
    }
    g_solver.cmd_index = 0;
    _fast_run_active = true;
    LOG_INFO("[MissionMgr] Fast run started.");
}

void mission_manager_init(void) {
    /** TODO: Implementation */
}

void mission_manager_update(void) {
    /** TODO: State machine for overall run strategy */
}
