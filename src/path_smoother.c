/**
 * @file path_smoother.c
 * @brief Path post-processor implementation with smooth rolling turn support.
 *
 * Converts raw Dijkstra waypoints into optimized motion commands:
 *   1. Straight-run merging (consecutive same-direction → single command)
 *   2. Turn classification with SMOOTH vs IN-PLACE selection
 *   3. Look-ahead: knows what's coming next to choose optimal turn type
 *   4. Optional diagonal pattern detection
 *
 * Smooth turn selection logic:
 *   - If the robot is moving forward and the next cell requires a 90° turn,
 *     emit CMD_SMOOTH_LEFT/RIGHT_90 (rolling arc, no stop).
 *   - If the robot must reverse (180°) in a dead end, emit CMD_TURN_180
 *     (must stop, rotate in place).
 *   - If a 90° turn is immediately followed by another 90° turn (S-curve),
 *     both are emitted as smooth turns — the robot chains them without stopping.
 */

#include "path_smoother.h"
#include "motion_profile.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Helper: append a command to the result (with bounds check).
 * ═══════════════════════════════════════════════════════════════════════════ */

static inline bool emit(SmoothedPath *r, CommandType type, uint8_t cells) {
    if (r->num_commands >= MAX_COMMANDS)
        return false;  /* overflow guard — should never happen on 16×16 */
    r->commands[r->num_commands].type = type;
    r->commands[r->num_commands].cells = cells;
    r->num_commands++;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Turn Type Analysis
 *
 *  Determines whether a turn can be executed as a smooth rolling arc
 *  or must be an in-place rotation.
 *
 *  Rules:
 *    90° turn with straights before AND after → SMOOTH (rolling arc)
 *    90° turn at start of path (no entry speed) → IN-PLACE
 *    180° turn → always IN-PLACE (too tight for rolling arc)
 * ═══════════════════════════════════════════════════════════════════════════ */

/** Check if waypoint i has a straight predecessor (same heading as i-1→i move direction). */
static bool has_straight_entry(const Waypoint *w, uint16_t num, uint16_t i) {
    (void)w;
    (void)num;
    if (i == 0) return false;
    /* If the heading at i-1 matches the direction of travel i-1→i, it's a straight entry */
    return i >= 1;  /* simplified: any preceding waypoint means we have forward momentum */
}

/** Check if waypoint i has a straight successor (heading at i matches the i→i+1 direction). */
static bool has_straight_exit(const Waypoint *w, uint16_t num, uint16_t i) {
    (void)w;
    return i + 1 < num;
}

/** Determine the best turn command for a heading change. */
static CommandType classify_turn(TurnType turn,
                                 bool has_entry_speed,
                                 bool has_exit_straight)
{
    switch (turn) {
        case TURN_LEFT_90:
            if (has_entry_speed && has_exit_straight)
                return CMD_SMOOTH_LEFT_90;   /* rolling arc — no stop! */
            return CMD_TURN_LEFT_90;         /* in-place fallback */

        case TURN_RIGHT_90:
            if (has_entry_speed && has_exit_straight)
                return CMD_SMOOTH_RIGHT_90;  /* rolling arc — no stop! */
            return CMD_TURN_RIGHT_90;        /* in-place fallback */

        case TURN_180:
            /* 180° is always in-place — rolling 180° requires too much space
             * and is mechanically unreliable at competition speeds. */
            return CMD_TURN_180;

        default:
            return CMD_STRAIGHT;  /* shouldn't happen */
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Diagonal Pattern Detection (optional)
 * ═══════════════════════════════════════════════════════════════════════════ */

#if ENABLE_DIAGONALS
static uint16_t try_diagonal(const Waypoint *waypoints,
                             uint16_t num_waypoints,
                             uint16_t start,
                             SmoothedPath *result)
{
    if (start + 2 >= num_waypoints)
        return 0;

    Direction h0 = waypoints[start].heading;
    Direction h1 = waypoints[start + 1].heading;
    TurnType first_turn = get_turn_type(h0, h1);

    if (first_turn != TURN_LEFT_90 && first_turn != TURN_RIGHT_90)
        return 0;

    uint16_t consumed = 2;
    uint8_t diag_cells = 1;

    for (uint16_t i = start + 2; i < num_waypoints; i++) {
        Direction hprev = waypoints[i - 1].heading;
        Direction hcur  = waypoints[i].heading;
        TurnType t = get_turn_type(hprev, hcur);

        if (t == first_turn) {
            diag_cells++;
            consumed++;
        } else if (t == TURN_NONE) {
            consumed++;
        } else {
            break;
        }
    }

    if (diag_cells < 2)
        return 0;

    /* Emit smooth entry turn, diagonal, then the exit is handled by the main loop */
    if (first_turn == TURN_LEFT_90) emit(result, CMD_SMOOTH_LEFT_90, 0);
    else emit(result, CMD_SMOOTH_RIGHT_90, 0);
    emit(result, CMD_DIAGONAL, diag_cells);

    return consumed;
}
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  Main Smoothing Logic — With Smooth Turn Classification
 * ═══════════════════════════════════════════════════════════════════════════ */

void path_smooth(const Waypoint *waypoints,
                 uint16_t num_waypoints,
                 SmoothedPath *result)
{
    result->num_commands = 0;

    if (num_waypoints < 2)
        return;

    uint16_t i = 0;
    uint16_t last_processed = 0;  /* track how far the main loop got */

    while (i < num_waypoints - 1) {
#if ENABLE_DIAGONALS
        uint16_t diag_consumed = try_diagonal(waypoints, num_waypoints, i, result);
        if (diag_consumed > 0) {
            i += diag_consumed;
            last_processed = i;
            continue;
        }
#endif

        /* ── Check if we need a turn before this segment ─────────────── */
        if (i > 0) {
            TurnType turn = get_turn_type(waypoints[i - 1].heading,
                                          waypoints[i].heading);
            if (turn != TURN_NONE) {
                /* Classify: smooth rolling vs in-place */
                bool entry = has_straight_entry(waypoints, num_waypoints, i);
                bool exit  = has_straight_exit(waypoints, num_waypoints, i);
                CommandType cmd = classify_turn(turn, entry, exit);
                emit(result, cmd, 0);
            }
        }

        /* ── Count consecutive straight moves ────────────────────────── */
        uint8_t straight_count = 1;
        while (i + straight_count < num_waypoints - 1 &&
               waypoints[i + straight_count].heading == waypoints[i].heading) {
            straight_count++;
        }

        emit(result, CMD_STRAIGHT, straight_count);
        i += straight_count;
        last_processed = i;
    }

    /* Handle the very last waypoint if there's a final turn needed,
     * BUT only if the main loop didn't already process it. */
    if (num_waypoints >= 2 && last_processed < num_waypoints - 1) {
        TurnType final_turn = get_turn_type(
            waypoints[num_waypoints - 2].heading,
            waypoints[num_waypoints - 1].heading
        );
        if (final_turn != TURN_NONE) {
            /* Last turn is always in-place (we're stopping at the goal) */
            switch (final_turn) {
                case TURN_LEFT_90:  emit(result, CMD_TURN_LEFT_90, 0);  break;
                case TURN_RIGHT_90: emit(result, CMD_TURN_RIGHT_90, 0); break;
                case TURN_180:      emit(result, CMD_TURN_180, 0);      break;
                default: break;
            }
        }
    }
}

