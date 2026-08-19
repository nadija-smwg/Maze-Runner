# Phase 1: Hardware Abstraction & Basic I/O

> **Goal:** Establish the foundational layer of the firmware. This phase guarantees that the STM32 microcontroller can reliably read simple inputs (buttons, battery voltage) and drive simple outputs (LEDs, general GPIO), while setting up the critical heartbeat timer that will drive the entire robot later.

---

## 1. GPIO & Pin Configurations (`hardware/gpio.cpp`)
*   **Purpose:** Centralize the initialization of all basic digital pins so they aren't scattered across the codebase.
*   **Execution Steps:**
    1.  Read mappings from `src/config/pin_config.h`.
    2.  Set `PIN_BUTTON_START` (`PB5`) and `PIN_BUTTON_MODE` (`PB4`) as `INPUT_PULLUP`.
    3.  Set Motor Standby (`PB14`) as `OUTPUT` (used later to enable the TB6612FNG driver).

## 2. LED Management (`hardware/led.cpp`)
*   **Status:** *Already Completed & Synced.*
*   **Flow Breakdown:**
    *   **External Status LED (`PA5`):** Configured as active-HIGH. Used to signal state machine transitions (e.g., solid when ready, blinking when exploring).
    *   **Onboard Debug LED (`PC13`):** Configured as active-LOW. Used for low-level hardware debugging and error signaling.
    *   **Non-Blocking Blink:** Uses `millis()` inside `led_update()` to toggle LEDs without `delay()`, ensuring the main loop never freezes.

## 3. Button Debouncing (`hardware/button.cpp`)
*   **Purpose:** Prevent mechanical button "bouncing" from triggering multiple unintended state changes.
*   **Flow Breakdown:**
    1.  `button_update()` is called every cycle in the main loop.
    2.  It reads the physical state of `PB4` and `PB5`.
    3.  It compares the current state to the previous state. If the state changes, it records a timestamp.
    4.  If the state remains stable for `DEBOUNCE_DELAY_MS` (e.g., 50ms), the button press is validated, and `button_just_pressed()` returns `true` for exactly one frame.

## 4. Battery Monitoring (`hardware/battery.cpp`)
*   **Purpose:** Read the 7.4V LiPo battery level via a voltage divider to prevent over-discharge and brownouts.
*   **Execution Steps:**
    1.  Configure `PB0` as an Analog Input.
    2.  Read the raw 12-bit ADC value (0-4095).
    3.  **Math Flow:** 
        *   `Voltage_Pin = (ADC_Value / 4095.0) * 3.3V`
        *   `Battery_Voltage = Voltage_Pin * ((R1 + R2) / R2)` (Using the specific resistor values on your PCB).
    4.  Provide `battery_get_voltage_mv()` and `battery_get_percentage()` for telemetry.

## 5. The Heartbeat Timer (`hardware/timer.cpp`)
*   **Purpose:** The entire robot's stability relies on a strict 1kHz (1 millisecond) control loop. PID calculations are mathematically invalid if the loop timing is inconsistent.
*   **Execution Steps:**
    1.  Configure a hardware timer (e.g., `TIM4` or SysTick) to trigger an interrupt exactly every 1000 microseconds.
    2.  Assign a callback function pointer to this timer (e.g., `timer_init(motion_controller_update)`).
    3.  When the timer fires, the STM32 stops whatever it is doing, runs the motion controller (PID, odometry), and then returns to the main loop.

---

## 🛠 Testing & Verification for Phase 1
1.  **Flash the robot.**
2.  Press the `BTN_START` button. Ensure the `DEBUG_LED` toggles cleanly without bouncing.
3.  Press the `BTN_MODE` button. Ensure the `STATUS_LED` toggles cleanly.
4.  Monitor the Serial terminal to ensure the battery voltage reads ~7.4V (or whatever the current charge is) and the 1kHz timer heartbeat is printing steadily.
