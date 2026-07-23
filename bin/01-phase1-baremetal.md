# Phase 1 — STM32 Ecosystem & Bare-Metal Basics

## 1.1 Toolchain Setup

You have two realistic paths for competition firmware:

| Option | Pros | Cons |
|---|---|---|
| **STM32CubeIDE** | Integrated CubeMX pin/clock config, GDB debugging, free | Eclipse-based, heavier |
| **PlatformIO + STM32Cube framework (VS Code)** | Better editor, git-friendly project files, fast builds | You configure clocks/pins by hand or import CubeMX-generated files |

**Recommendation for you specifically** (strong C++/CS background): Use
**STM32CubeIDE to generate the initial clock/peripheral init code via CubeMX**,
then **move active development into PlatformIO or plain Makefile + arm-none-eabi-gcc**
once the skeleton exists. CubeMX's `.ioc` file is just a pin/clock spec — you
regenerate `Core/Src/main.c`, `stm32f4xx_hal_msp.c`, etc. from it whenever you
add a peripheral. Treat CubeMX as your "peripheral compiler," not your IDE.

Toolchain components:
- `arm-none-eabi-gcc` — cross compiler (Cortex-M4F target: `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb`)
- `ST-Link V2` (clone is fine) — SWD programmer/debugger
- OpenOCD or ST-Link GDB server — for `.elf` flashing + live debugging
- HAL library (`STM32F4xx_HAL_Driver`) — what we'll build on top of; CMSIS underneath that

**Checkpoint 1.1:** Get a blank CubeMX project building and flashing an LED
blink on PC13 (onboard LED, active-low on Black Pill). Confirm you can hit a
breakpoint in `main()` with GDB before moving on.

---

## 1.2 Black Pill Pinout, Clock Tree, GPIO

### Clock Configuration
The F401 runs its core off `SYSCLK`, derived from `PLL` sourced from `HSE`
(25 MHz external crystal on most Black Pill boards) or `HSI` (internal 16 MHz,
less accurate — avoid for competition timing precision).

```
HSE (25 MHz) → PLL → SYSCLK (84 MHz max on F401)
SYSCLK → AHB prescaler → HCLK (up to 100 MHz)
HCLK → APB1 prescaler → PCLK1 (max 50 MHz) — Timers 2-5, I2C, USART2/3
HCLK → APB2 prescaler → PCLK2 (max 100 MHz) — Timers 1/9-11, ADC1, SPI1
```

Why this matters for you: **timer tick resolution and PWM frequency are
derived from PCLK**, not SYSCLK directly. When you compute PSC/ARR for a PWM
channel later, you need the *actual* timer clock, which HAL exposes via
`HAL_RCC_GetPCLK1Freq()` / `GetPCLK2Freq()` (and note: if the APBx prescaler
is >1, the timer clock is 2× PCLK, per an F4 quirk — CubeMX's Clock
Configuration tab shows this correctly, always cross-check there).

In CubeMX: RCC → HSE = Crystal/Ceramic Resonator, SYSCLK Mux = PLLCLK, set
PLL M/N/P to hit 100 MHz. CubeMX validates this for you — don't hand-calculate
unless the crystal isn't standard.

### GPIO Fundamentals (register-level, since HAL abstracts this)
Every GPIO pin is configured through 4 core registers per port:
- `MODER` — 2 bits/pin: input (00), output (01), alternate function (10), analog (11)
- `OTYPER` — push-pull (0) vs open-drain (1)
- `OSPEEDR` — slew rate (matters at high PWM switching frequencies — use High/Very High for motor PWM pins)
- `PUPDR` — pull-up/pull-down/none

`HAL_GPIO_Init()` just writes these registers based on your `GPIO_InitTypeDef`.
Example — configuring an IR sensor's digital "obstacle" pin as input with
pull-down, and a motor driver's STBY pin as push-pull output:

```c
GPIO_InitTypeDef GPIO_InitStruct = {0};

// STBY pin — push-pull output
GPIO_InitStruct.Pin = MOTOR_STBY_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(MOTOR_STBY_GPIO_Port, &GPIO_InitStruct);

// Encoder index / button — input with pull-down
GPIO_InitStruct.Pin = BUTTON_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_PULLDOWN;
HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);
```

**Black Pill pin budget for a micromouse** (typical allocation — confirm
against your actual sensor count):
- 4x PWM (2 motors × 2 direction pins, or 2 PWM + 2 DIR with TB6612)
- 4x Timer Encoder Mode pins (TIM2 CH1/CH2, TIM3 CH1/CH2 — 2 encoders)
- 3–5x ADC pins (analog IR sensors) — all must be on ADC1-capable pins
- I2C1 (PB6/PB7 or PB8/PB9) for MPU6050 and/or VL53L0X (note: multiple VL53L0X
  need an XSHUT GPIO each to reassign I2C addresses at boot, since they all
  default to `0x29`)
- 1x GPIO for start button, 1-2 for status LEDs/buzzer

### EXTI (External Interrupts)
Used for: start button press, and optionally a "wall hit"/collision microswitch.
NOT typically used for encoders (use Timer Encoder Mode instead — Phase 2) or
high-speed sensor polling (use timer-triggered ADC or DMA instead — more on
this in Phase 2).

```c
// CubeMX sets GPIO_MODE_IT_FALLING on the button pin and enables NVIC for EXTIx_IRQn
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == START_BUTTON_Pin) {
        start_flag = 1;   // keep ISR bodies tiny — set a flag, handle in main loop
    }
}
```

**Critical embedded discipline point** (you'll appreciate this from computer
architecture): ISRs must be short and deterministic. Never call `HAL_Delay()`
inside an ISR, never do floating-point-heavy PID math inside an EXTI callback.
Set a flag or push to a lock-free queue, process in `main()` or a scheduled task.

### SysTick
HAL configures SysTick as a 1ms tick by default (`HAL_InitTick()`), which is
what drives `HAL_Delay()` and `HAL_GetTick()`. For a micromouse this 1ms
resolution is your **coarse system heartbeat** — good for state machine
timeouts, bad for control loop timing (your PID loop needs to run at a fixed,
precise rate, e.g. 1kHz, which you'll drive from a **hardware timer interrupt**,
not SysTick — see 1.3).

```c
uint32_t t0 = HAL_GetTick();
while (HAL_GetTick() - t0 < 500) {
    // non-blocking-style wait pattern — always subtract, never compare
    // absolute values, to survive the 32-bit overflow (~49 days, fine for us)
}
```

**Checkpoint 1.2:** Explain in your own words why we'd use a hardware timer
interrupt instead of SysTick/`HAL_Delay()` for the PID control loop. Wire up
the start button on EXTI and blink the status LED only after button press.

---

## 1.3 Hardware Timers — The Real Workhorse

STM32 timers are the single most important peripheral class for this project.
You'll use them for 4 distinct jobs:

| Job | Timer Mode | Phase |
|---|---|---|
| PID loop heartbeat (e.g. 1kHz) | Timer interrupt (Update event) | 1.3 here |
| Motor speed control | PWM generation | Phase 2 |
| Encoder tick counting | Encoder Mode (hardware quadrature decode) | Phase 2 |
| Precise time-of-flight / ranging delays (if not using I2C ToF sensor) | Input Capture | optional |

### Timer Interrupt for the Control Loop
The F401 has TIM1 (advanced), TIM2/TIM5 (32-bit general purpose), TIM3/TIM4
(16-bit general purpose), and TIM9-11 (basic). Reserve **TIM2 and TIM3 for
encoders** (Phase 2), **TIM1 or TIM4 for motor PWM**, and use e.g. **TIM10** as
a pure interrupt source for your control loop tick.

Timer frequency math: `Timer_tick_freq = Timer_input_clock / (PSC + 1)`, and
interrupt (Update event) frequency = `Timer_tick_freq / (ARR + 1)`.

Example: PCLK2 = 100 MHz feeding TIM10, want 1kHz interrupt:
```c
// CubeMX TIM10 config:
// Prescaler (PSC) = 99      → 100MHz / 100 = 1MHz counter clock
// Counter Period (ARR) = 999 → 1MHz / 1000 = 1kHz update event

// In main.c after MX_TIM10_Init():
HAL_TIM_Base_Start_IT(&htim10);
```

```c
volatile uint8_t control_tick_flag = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM10) {
        control_tick_flag = 1;   // main loop checks this and runs PID
    }
}

// main loop
while (1) {
    if (control_tick_flag) {
        control_tick_flag = 0;
        run_control_loop();   // read encoders, run PID, update PWM — all here
    }
    // lower-priority work: maze logic, telemetry, etc.
}
```

Why flag-and-poll instead of doing the PID math directly in the ISR? Because
your PID math may call into shared state (odometry, sensor readings) that's
also touched elsewhere; keeping the ISR minimal avoids priority-inversion
headaches and keeps worst-case ISR latency predictable — important since
you'll likely stack this interrupt with encoder overflow interrupts and
possibly ADC/DMA completion interrupts later. That said, for a *single*
1kHz loop with nothing else time-critical, calling `run_control_loop()`
directly from the ISR is also a common, acceptable competition pattern —
the key discipline is knowing your worst-case ISR execution time and keeping
it well under your tick period.

**Checkpoint 1.3:** Set up TIM10 (or any free basic timer) at exactly 1kHz,
toggle a debug GPIO in the callback, and verify the frequency on a logic
analyzer or oscilloscope (or even a slow-motion phone camera on an LED for a
rough sanity check). Confirm you understand the PSC/ARR relationship well
enough to retarget it to 500Hz without me telling you the numbers.

---

## Phase 1 Summary — What You Should Now Be Able To Do
- Build/flash/debug a CubeMX-generated project via GDB.
- Explain the clock tree from HSE → SYSCLK → PCLK1/PCLK2 and why it matters
  for timer math.
- Configure GPIO in input/output/AF modes and know which registers HAL is
  touching underneath.
- Trigger and handle an EXTI interrupt correctly (short ISR, flag pattern).
- Configure a hardware timer to fire a periodic interrupt at an exact
  frequency, and understand why this (not `HAL_Delay`) drives the control loop.

When you're ready, confirm and we'll move to **[Phase 2: Sensor & Actuator
Interfacing](02-phase2-sensors-actuators.md)** — PWM motor drive, quadrature
encoders in hardware, ADC for IR sensors, and I2C for IMU/ToF.
