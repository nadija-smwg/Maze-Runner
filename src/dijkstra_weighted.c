/**
 * @file dijkstra_weighted.c
 * @brief Weighted Dijkstra with turn penalties — fixed-size binary min-heap,
 *        state = (x, y, heading), path reconstruction via parent array.
 *
 * Memory usage:
 *   dist[1024]   = 2048 bytes (uint16_t per state)
 *   parent[1024] = 2048 bytes (uint16_t per state, stores parent state index)
 *   heap[1024]   = 4096 bytes (HeapNode: state_idx + cost)
 *   Total: ~8KB — well within STM32F411's 128KB RAM.
 */

#include "dijkstra_weighted.h"
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  State Encoding
 *
 *  state_index = x * MAZE_SIZE * 4 + y * 4 + heading
 *  Total states = 16 × 16 × 4 = 1024
 * ═══════════════════════════════════════════════════════════════════════════ */

static inline uint16_t encode_state(uint8_t x, uint8_t y, Direction h) {
    return (uint16_t)x * MAZE_SIZE * NUM_DIRECTIONS
         + (uint16_t)y * NUM_DIRECTIONS
         + (uint16_t)h;
}

static inline void decode_state(uint16_t idx, uint8_t *x, uint8_t *y, Direction *h) {
    *h = (Direction)(idx % NUM_DIRECTIONS);
    idx /= NUM_DIRECTIONS;
    *y = idx % MAZE_SIZE;
    *x = idx / MAZE_SIZE;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Binary Min-Heap (fixed-size, embedded-safe)
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef struct {
    uint16_t state_idx;
    uint16_t cost;
} HeapNode;

static HeapNode heap[DIJKSTRA_STATES];
static int heap_size;

static void heap_init(void) {
    heap_size = 0;
}

static void heap_swap(int a, int b) {
    HeapNode tmp = heap[a];
    heap[a] = heap[b];
    heap[b] = tmp;
}

static void heap_push(uint16_t state_idx, uint16_t cost) {
    int i = heap_size++;
    heap[i].state_idx = state_idx;
    heap[i].cost = cost;
    /* Bubble up. */
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (heap[parent].cost <= heap[i].cost) break;
        heap_swap(parent, i);
        i = parent;
    }
}

static HeapNode heap_pop(void) {
    HeapNode top = heap[0];
    heap[0] = heap[--heap_size];
    /* Bubble down. */
    int i = 0;
    while (1) {
        int left = 2 * i + 1, right = 2 * i + 2, smallest = i;
        if (left < heap_size && heap[left].cost < heap[smallest].cost)
            smallest = left;
        if (right < heap_size && heap[right].cost < heap[smallest].cost)
            smallest = right;
        if (smallest == i) break;
        heap_swap(i, smallest);
        i = smallest;
    }
    return top;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Dijkstra Core
 * ═══════════════════════════════════════════════════════════════════════════ */

static uint16_t dist[DIJKSTRA_STATES];
static uint16_t parent[DIJKSTRA_STATES];

#define NO_PARENT 0xFFFF

void dijkstra_compute(const MazeMap *m,
                      uint8_t start_x, uint8_t start_y,
                      Direction start_heading,
                      const uint8_t goal_cells[][2],
                      uint8_t num_goals,
                      DijkstraResult *result)
{
    /* Initialize. */
    memset(dist, 0xFF, sizeof(dist));       /* all distances = 0xFFFF       */
    memset(parent, 0xFF, sizeof(parent));   /* no parents                   */
    heap_init();

    result->found = false;
    result->path_length = 0;
    result->total_cost = FLOOD_INFINITY;

    /* Seed start state. */
    uint16_t start_idx = encode_state(start_x, start_y, start_heading);
    dist[start_idx] = 0;
    heap_push(start_idx, 0);

    uint16_t goal_state_idx = NO_PARENT;  /* will hold the best goal state */

    while (heap_size > 0) {
        HeapNode cur = heap_pop();

        /* Skip stale entries (heap may contain outdated costs). */
        if (cur.cost > dist[cur.state_idx])
            continue;

        /* Check if we reached a goal cell. */
        uint8_t cx, cy;
        Direction ch;
        decode_state(cur.state_idx, &cx, &cy, &ch);

        for (uint8_t g = 0; g < num_goals; g++) {
            if (cx == goal_cells[g][0] && cy == goal_cells[g][1]) {
                /* First goal state popped = optimal (Dijkstra guarantee). */
                goal_state_idx = cur.state_idx;
                result->found = true;
                result->total_cost = cur.cost;
                goto found_goal;
            }
        }

        /*
         * Explore all 4 possible moves: for each direction d, the cost to
         * arrive at the neighbor cell facing direction d is:
         *   COST_STRAIGHT  +  turn_penalty(current_heading → d)
         */
        for (int d = 0; d < 4; d++) {
            Direction new_heading = (Direction)d;

            /* Can't pass through a known wall. */
            if (maze_has_wall(m, cx, cy, new_heading))
                continue;

            int8_t nx = cx + DX[d];
            int8_t ny = cy + DY[d];

            if (!maze_in_bounds(nx, ny))
                continue;

            /* Compute edge cost: straight + turn penalty. */
            uint8_t turns = turn_cost_steps(ch, new_heading);
            uint16_t edge_cost = COST_STRAIGHT;
            if (turns == 1) edge_cost += COST_TURN_90;
            else if (turns == 2) edge_cost += COST_TURN_180;

            uint16_t new_cost = cur.cost + edge_cost;
            uint16_t new_idx = encode_state((uint8_t)nx, (uint8_t)ny, new_heading);

            if (new_cost < dist[new_idx]) {
                dist[new_idx] = new_cost;
                parent[new_idx] = cur.state_idx;
                heap_push(new_idx, new_cost);
            }
        }
    }

found_goal:
    if (!result->found)
        return;

    /* ── Path reconstruction (reverse walk from goal to start) ────────── */
    /* Build path in reverse into a temporary buffer, then copy forward. */
    Waypoint rev_path[MAX_PATH_LENGTH];
    uint16_t rev_len = 0;

    uint16_t idx = goal_state_idx;
    while (idx != NO_PARENT && rev_len < MAX_PATH_LENGTH) {
        uint8_t px, py;
        Direction ph;
        decode_state(idx, &px, &py, &ph);
        rev_path[rev_len].x = px;
        rev_path[rev_len].y = py;
        rev_path[rev_len].heading = ph;
        rev_len++;
        idx = parent[idx];
    }

    /* Reverse into result->path (start→goal order). */
    result->path_length = rev_len;
    for (uint16_t i = 0; i < rev_len; i++) {
        result->path[i] = rev_path[rev_len - 1 - i];
    }
}
