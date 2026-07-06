/**
 * @file maze.h
 * @brief Maze data structures, direction utilities, and the MazeMap class.
 *
 * Embedded-safe: all fixed-size, zero heap allocation, pure value types.
 * Designed for STM32F411 (128KB RAM) — total maze footprint < 2KB.
 */

#ifndef MAZE_H
#define MAZE_H

#include "config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>  /* memset */

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  Direction System
 * ═══════════════════════════════════════════════════════════════════════════ */

/** Absolute heading enum — clockwise from North. */
typedef enum {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3
} Direction;

/** Wall bitmask values (matches direction order for easy indexing). */
typedef enum {
    WALL_N = 0x01,
    WALL_E = 0x02,
    WALL_S = 0x04,
    WALL_W = 0x08
} WallBit;

/** Delta-X for each direction: N=0, E=+1, S=0, W=-1 */
static const int8_t DX[4] = { 0, 1, 0, -1 };

/** Delta-Y for each direction: N=+1, E=0, S=-1, W=0 */
static const int8_t DY[4] = { 1, 0, -1, 0 };

/** Wall bitmask indexed by Direction. */
static const uint8_t DIR_WALL_BIT[4] = { WALL_N, WALL_E, WALL_S, WALL_W };

/** Opposite direction lookup. */
static const Direction DIR_OPPOSITE[4] = { DIR_SOUTH, DIR_WEST, DIR_NORTH, DIR_EAST };

/**
 * Compute the number of 90° turns to go from `from` heading to `to` heading.
 * Returns: 0 (straight), 1 (left or right 90°), 2 (180°).
 */
static inline uint8_t turn_cost_steps(Direction from, Direction to) {
    int diff = ((int)to - (int)from + 4) % 4;
    if (diff == 0) return 0;
    if (diff == 1 || diff == 3) return 1;
    return 2;  /* diff == 2 → 180° */
}

/**
 * Turn type: relative turn from current heading to target heading.
 */
typedef enum {
    TURN_NONE      = 0,  /* same direction, no turn */
    TURN_RIGHT_90  = 1,
    TURN_180       = 2,
    TURN_LEFT_90   = 3
} TurnType;

static inline TurnType get_turn_type(Direction from, Direction to) {
    return (TurnType)(((int)to - (int)from + 4) % 4);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Cell & Maze Structures
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * Per-cell data. Compact: 5 bytes per cell, 16×16 = 1280 bytes total.
 */
typedef struct {
    uint8_t  walls;        /**< Known wall bitmask (WALL_N | WALL_E | ...) */
    uint8_t  visited;      /**< Has the mouse physically visited this cell */
    uint16_t flood_value;  /**< BFS flood distance to goal (search run)    */
} Cell;

/**
 * The maze map. Statically allocated, no heap.
 */
typedef struct {
    Cell cells[MAZE_SIZE][MAZE_SIZE];
} MazeMap;

/* ─── Maze Initialization ──────────────────────────────────────────────── */

/**
 * Initialize maze to "unknown" state: only the outer border walls are known,
 * all interior walls are unknown (not present), all cells unvisited.
 * This is the correct starting state for a competition maze.
 */
static inline void maze_init_unknown(MazeMap *m) {
    memset(m, 0, sizeof(MazeMap));

    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            m->cells[i][j].flood_value = FLOOD_INFINITY;
        }
    }

    /* Border walls — always present in a competition maze. */
    for (int i = 0; i < MAZE_SIZE; i++) {
        m->cells[i][0].walls             |= WALL_S;   /* south border  */
        m->cells[i][MAZE_SIZE - 1].walls |= WALL_N;   /* north border  */
        m->cells[0][i].walls             |= WALL_W;   /* west border   */
        m->cells[MAZE_SIZE - 1][i].walls |= WALL_E;   /* east border   */
    }
}

/**
 * Reset all flood values to FLOOD_INFINITY (before recomputing flood fill).
 */
static inline void maze_reset_flood(MazeMap *m) {
    for (int i = 0; i < MAZE_SIZE; i++)
        for (int j = 0; j < MAZE_SIZE; j++)
            m->cells[i][j].flood_value = FLOOD_INFINITY;
}

/* ─── Wall Operations ──────────────────────────────────────────────────── */

/**
 * Set a wall at cell (x,y) in direction `dir`, and its mirror in the
 * adjacent cell. Always updates both sides to maintain consistency.
 */
static inline void maze_set_wall(MazeMap *m, uint8_t x, uint8_t y, Direction dir) {
    m->cells[x][y].walls |= DIR_WALL_BIT[dir];
    int8_t nx = (int8_t)(x + DX[dir]);
    int8_t ny = (int8_t)(y + DY[dir]);
    if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
        m->cells[nx][ny].walls |= DIR_WALL_BIT[DIR_OPPOSITE[dir]];
    }
}

/**
 * Check if cell (x,y) has a known wall in direction `dir`.
 */
static inline bool maze_has_wall(const MazeMap *m, uint8_t x, uint8_t y, Direction dir) {
    return (m->cells[x][y].walls & DIR_WALL_BIT[dir]) != 0;
}

/**
 * Check if a neighbor in direction `dir` from (x,y) is within bounds.
 */
static inline bool maze_in_bounds(int8_t x, int8_t y) {
    return x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE;
}

/* ─── Waypoint / Path Types ────────────────────────────────────────────── */

/** A single waypoint in a reconstructed path. */
typedef struct {
    uint8_t   x, y;
    Direction heading;
} Waypoint;

/** Motion command types for the path smoother output. */
typedef enum {
    /* Basic commands (in-place turns, stop before turning) */
    CMD_STRAIGHT,
    CMD_TURN_LEFT_90,
    CMD_TURN_RIGHT_90,
    CMD_TURN_180,
    CMD_DIAGONAL,          /* only if ENABLE_DIAGONALS                      */

    /* Smooth rolling turn commands (turn while moving, no stop) */
    CMD_SMOOTH_LEFT_90,    /* arc left  90° at constant forward speed       */
    CMD_SMOOTH_RIGHT_90,   /* arc right 90° at constant forward speed       */
    CMD_SMOOTH_LEFT_180,   /* arc left  180° (hairpin)                      */
    CMD_SMOOTH_RIGHT_180,  /* arc right 180° (hairpin)                      */

    /* Compound moves (straight + turn merged for zero-pause transitions) */
    CMD_SS_LEFT_90,        /* straight(n) → smooth left 90 (no stop between) */
    CMD_SS_RIGHT_90        /* straight(n) → smooth right 90                  */
} CommandType;

/** A single motion command (what the locomotion layer executes). */
typedef struct {
    CommandType type;
    uint8_t     cells;  /**< number of cells for straight/diagonal moves */
} MotionCommand;

#ifdef __cplusplus
}
#endif

#endif /* MAZE_H */
