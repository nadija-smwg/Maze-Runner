/**
 * @file flood_fill.c
 * @brief Modified Flood Fill implementation — multi-source BFS with
 *        straight-preference tie-breaking.
 */

#include "flood_fill.h"

/* ── Fixed-size BFS queue (ring buffer) ───────────────────────────────── */

typedef struct {
    uint8_t x, y;
} QItem;

/* 256 cells max in a 16×16 maze — queue never exceeds this. */
static QItem bfs_queue[MAZE_SIZE * MAZE_SIZE];

/* ═══════════════════════════════════════════════════════════════════════ */

void flood_fill_compute(MazeMap *m,
                        const uint8_t goal_cells[][2],
                        uint8_t num_goals)
{
    /* Reset all flood values. */
    maze_reset_flood(m);

    int head = 0, tail = 0;

    /* Seed goal cells with distance 0. */
    for (uint8_t g = 0; g < num_goals; g++) {
        uint8_t gx = goal_cells[g][0];
        uint8_t gy = goal_cells[g][1];
        m->cells[gx][gy].flood_value = 0;
        bfs_queue[tail].x = gx;
        bfs_queue[tail].y = gy;
        tail++;
    }

    /* BFS expansion. */
    while (head < tail) {
        QItem cur = bfs_queue[head++];
        uint16_t cur_dist = m->cells[cur.x][cur.y].flood_value;

        for (int d = 0; d < 4; d++) {
            /* Skip if there's a known wall blocking this direction. */
            if (maze_has_wall(m, cur.x, cur.y, (Direction)d))
                continue;

            int8_t nx = cur.x + DX[d];
            int8_t ny = cur.y + DY[d];

            /* Skip out-of-bounds. */
            if (!maze_in_bounds(nx, ny))
                continue;

            /* Relax: if we found a shorter path, update and enqueue. */
            if (m->cells[nx][ny].flood_value > cur_dist + 1) {
                m->cells[nx][ny].flood_value = cur_dist + 1;
                bfs_queue[tail].x = (uint8_t)nx;
                bfs_queue[tail].y = (uint8_t)ny;
                tail++;
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */

Direction flood_fill_choose_direction(const MazeMap *m,
                                     uint8_t x, uint8_t y,
                                     Direction current_heading)
{
    uint16_t best_val = FLOOD_INFINITY;
    Direction best_dir = current_heading;

    /*
     * Check directions in preference order:
     *   1. Straight ahead  (current_heading)
     *   2. Left            ((current_heading + 3) % 4)
     *   3. Right           ((current_heading + 1) % 4)
     *   4. Reverse         ((current_heading + 2) % 4)
     *
     * By checking straight first, equal-cost ties naturally resolve
     * to the straight-ahead option, reducing unnecessary turns.
     */
    const int offsets[4] = { 0, 3, 1, 2 };

    for (int i = 0; i < 4; i++) {
        Direction d;
#if PREFER_STRAIGHT
        d = (Direction)((current_heading + offsets[i]) % 4);
#else
        d = (Direction)i;
#endif

        /* Skip if wall blocks this direction. */
        if (maze_has_wall(m, x, y, d))
            continue;

        int8_t nx = x + DX[d];
        int8_t ny = y + DY[d];

        if (!maze_in_bounds(nx, ny))
            continue;

        uint16_t val = m->cells[nx][ny].flood_value;

        if (val < best_val) {
            best_val = val;
            best_dir = d;
        }
    }

    return best_dir;
}
