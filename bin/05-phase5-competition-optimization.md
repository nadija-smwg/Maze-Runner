# Phase 5 — Competition Optimization

Prerequisite: a working end-to-end search run from Phase 4.

## 5.1 State Machine Architecture — Search Run vs. Fast Run

Competition rules (standard IEEE/APEC-style micromouse rules, which SLIIT
ROBOFEST typically follows closely) give you multiple runs within a time
limit. Strategy: **Run 1 (or first few) = exploration ("search run")**,
mapping the maze conservatively; **later runs = "fast run"**, replaying the
best known path at maximum safe speed using the fully-explored (or
best-effort) maze map. Your score is typically your best (fastest) run.

### Top-level state machine
```cpp
enum class RobotState {
    IDLE,           // waiting for start button, all motors off
    CALIBRATING,     // gyro bias calc, sensor warm-up
    SEARCH_RUN,      // explore + flood-fill navigate to center
    RETURN_TO_START, // optional: navigate back to start cell after reaching center
    FAST_RUN,        // replay best path at speed, using fully-known maze
    FINISHED,        // stopped, awaiting reset/next run
    FAULT            // hard-fault/safety-stop state — see 5.3
};

RobotState state = RobotState::IDLE;

void state_machine_step(void) {
    switch (state) {
        case RobotState::IDLE:
            if (start_button_pressed()) state = RobotState::CALIBRATING;
            break;
        case RobotState::CALIBRATING:
            calibrate_gyro();
            init_maze_unknown();          // all walls unknown except maze border
            state = RobotState::SEARCH_RUN;
            break;
        case RobotState::SEARCH_RUN:
            search_run_step();             // Phase 4 loop, one cell at a time
            if (at_goal_center()) {
                compute_best_path();       // Dijkstra/flood-fill from start using now-known maze
                state = RobotState::RETURN_TO_START;
            }
            break;
        case RobotState::RETURN_TO_START:
            search_run_step();             // reuse same logic, target = start cell
            if (at_start_cell()) state = RobotState::FAST_RUN;
            break;
        case RobotState::FAST_RUN:
            fast_run_step();                // follow precomputed path, higher speed profile
            if (at_goal_center()) state = RobotState::FINISHED;
            break;
        case RobotState::FAULT:
            all_motors_off();
            // stay here until physical reset — do NOT auto-recover from a fault silently
            break;
        default: break;
    }
}
```

Why explicitly separate `RETURN_TO_START`: many competitions score/allow a
run only if the mouse returns to start (or you simply want the maze fully
explored before committing to a fast run) — and computing the **true optimal
path** (not just "shortest as discovered greedily") should happen with full
knowledge after search completes, typically via a clean Dijkstra/flood-fill
from start using the complete wall map, since the greedy search-run path
might not have been optimal (flood fill during search only guarantees
shortest path *given information known at that moment*, which changes as you
go).

**Checkpoint 5.1:** Implement the full state machine, confirm clean
transitions IDLE → CALIBRATING → SEARCH_RUN → RETURN_TO_START → FAST_RUN →
FINISHED on a real maze, with the fast run visibly faster and taking the
truly optimal path (verify by hand for a small maze).

---

## 5.2 Speed Tuning Strategy & Avoiding Wall-Contact Penalties

Most rulesets penalize (or disqualify the run for) sustained wall contact —
often phrased as a **penalty if the mouse touches a wall for more than ~3
seconds continuously**, or point deductions per contact. Practical
implications for your control code:

1. **Never rely on wall contact as a feature ("wall following via bumping")**
   at the fast-run stage — some hobbyist mice use light wall contact for
   centering during search, which is a legitimate technique in some rule
   variants, but you must **detect and break contact quickly**, not sit
   against a wall. If your ruleset penalizes *any* prolonged contact, treat
   wall proximity sensing as strictly non-contact (stay a few mm off).

2. **Side-sensor centering PID**: use the two side IR/ToF readings to correct
   lateral position **within** a cell (a mismatch between left/right wall
   distance means the mouse has drifted off-center), added as another small
   correction term to the heading/differential command, separate from your
   main heading PID:

```cpp
float centering_correction(float left_dist, float right_dist, bool left_wall, bool right_wall) {
    if (left_wall && right_wall) {
        // Both walls present — the ground-truth centering signal, most reliable
        float error = left_dist - right_dist; // positive = closer to right wall
        return CENTERING_KP * error;
    }
    // If only one wall present, centering off a single wall is less reliable
    // (no ground truth for "correct" distance) — weight it down or skip.
    return 0.0f;
}
```

3. **Speed profile per run phase**: don't fast-run at max speed on your first
   attempt. A staged approach many competitive teams use:
   - Search run: conservative speed, prioritize sensing accuracy over time.
   - Fast run attempt 1: moderate speed increase, validate the path holds up.
   - Later fast runs (if multiple attempts allowed): push speed further,
     since your **best** run counts — a crashed high-speed run costs you
     nothing if you already banked a slower successful one.

4. **Corner/turn speed reduction**: even in a fast run, reduce forward speed
   before a required turn (don't try to "drift" through 90° turns until your
   mechanical and control tuning is proven very solid) — the trapezoidal
   profile from Phase 4.3 already handles this if you set your deceleration
   target correctly ahead of turn cells.

**Checkpoint 5.2:** Implement and tune side-wall centering, confirm the mouse
stays visually centered through a straight corridor even when started
slightly off-center, and never maintains sustained wall contact.

---

## 5.3 Memory Optimization & Debugging Hard Faults

### Memory discipline on the F401 (64KB RAM, 256KB Flash)
- **No dynamic allocation (`malloc`/`new`) after initialization**, ideally
  none at all. Pre-allocate every buffer (maze array, sensor buffers, PID
  objects) as static/global or on the stack at a known bounded depth.
- **Watch stack usage in ISRs** — deep call chains or large local arrays
  inside an interrupt context can blow the stack silently, which manifests
  as mysterious corruption or a HardFault far from the actual bug. Keep ISR
  call depth shallow (this is another reason for the flag-and-poll pattern
  from Phase 1.3).
- **`-Os` or `-O2` optimization** for release builds — verify timing-critical
  code (PID loop, flood fill) still meets deadlines after optimization;
  occasionally `volatile` is needed on variables shared between ISR and main
  loop context that the compiler might otherwise incorrectly cache in a
  register (`control_tick_flag` from Phase 1.3, ADC DMA buffer reads, etc.
  should all be `volatile` or accessed via proper memory barriers).

### Debugging HardFaults
A HardFault on Cortex-M4 usually means: NULL pointer dereference, stack
overflow, unaligned memory access, or division by zero (with the FPU, this
is often a NaN propagating rather than a trap, so also watch for silently
wrong math rather than a crash). Standard technique — a HardFault handler
that captures the stack frame for post-mortem inspection:

```c
void HardFault_Handler(void) {
    __asm volatile (
        "TST LR, #4                                                \n"
        "ITE EQ                                                    \n"
        "MRSEQ R0, MSP                                              \n"
        "MRSNE R0, PSP                                              \n"
        "B hard_fault_handler_c                                    \n"
    );
}

void hard_fault_handler_c(uint32_t *hardfault_args) {
    volatile uint32_t stacked_r0  = hardfault_args[0];
    volatile uint32_t stacked_pc  = hardfault_args[6]; // return address — where it faulted
    volatile uint32_t stacked_lr  = hardfault_args[5];
    volatile uint32_t cfsr = SCB->CFSR; // Configurable Fault Status Register — decode this!
    (void)stacked_r0; (void)stacked_lr;
    // Set a breakpoint here; inspect stacked_pc against your .map/.lst file
    // to find the exact faulting instruction, and decode cfsr bits per the
    // ARM Cortex-M4 TRM (e.g. bit 1 = undefined instruction, bit 9 = unaligned access...)
    while (1) { /* halt for debugger inspection; blink an LED pattern for field diagnosis */ }
}
```

In practice: build with debug symbols (`-g3`), use `arm-none-eabi-addr2line`
or your IDE's disassembly view on the captured `stacked_pc` to find the exact
line. For **field debugging without a debugger attached** (i.e. at
competition), have the fault handler blink a distinct LED pattern or write a
fault code to a spare flash sector so you can diagnose after the fact without
needing a laptop connected mid-event.

### Practical robustness checklist before competition day
- [ ] Battery voltage monitored (ADC channel on a divided VBAT line) — low
      voltage causes weird motor/sensor behavior that looks like software bugs.
- [ ] `FAULT` state reachable from anywhere on: encoder tick rate implausibly
      high (slip/skid detection), control loop overrun (tick took >1ms),
      sensor readings out of physical range, unexpected wall (front sensor
      trips mid-cell when flood fill expected clear path) — decide per-case
      whether to hard-stop or replan.
- [ ] Watchdog timer (`IWDG`) enabled, refreshed only from a known-healthy
      main loop location — if the state machine hangs, the watchdog resets
      the MCU rather than leaving the robot stuck mid-maze burning the clock.
- [ ] All tuning constants (PID gains, speed profiles, cell size, wheel base)
      centralized in one config header, not scattered — you WILL be
      re-tuning at the venue on unfamiliar maze surface material.

**Checkpoint 5.3:** Deliberately trigger a HardFault (e.g. dereference a bad
pointer) in a test build, confirm your handler captures a usable `stacked_pc`
you can map back to source. Implement and test the watchdog actually resetting
the board when the main loop is artificially hung.

---

## Phase 5 Summary — What You Should Now Be Able To Do
- Run a full competition-shaped state machine: calibrate → search → optimal
  replan → fast run, with clean state transitions.
- Tune wall-centering and staged speed profiles to avoid wall-contact
  penalties while still running fast.
- Apply embedded memory discipline (no heap in the hot path, `volatile`
  correctness, bounded ISR stack usage).
- Debug HardFaults from a captured stack frame, and have watchdog/fault-state
  robustness so a bug degrades gracefully instead of stranding the robot mid-run.

---

## You've Now Covered the Full Stack
Bare metal → sensors/actuators → control theory → maze algorithms →
competition-grade robustness. From here, the highest-leverage next steps are
mechanical (wheel grip, weight distribution) and **empirical tuning time on
the actual maze surface** — no amount of additional software sophistication
substitutes for hours of real runs at competition-realistic speed.

Tell me which phase you want to start actually building first, or if you want
to revisit any section in more depth (e.g. a from-scratch complementary
filter derivation, or the full VL53L0X register sequence) before writing code.
