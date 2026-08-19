# Phase 2: Actuation & Low-Level Kinematics

> **Goal:** Give the robot "muscles" and "proprioception." Port the raw STM32 timer logic from the testing sketches to spin the motors at precise speeds, and use hardware quadrature decoding to measure exactly how far the wheels have turned.

---

## 1. Pulse Width Modulation (PWM) Generation (`hardware/pwm.cpp`)
*   **Source Logic:** `Testing Codes/4.Motors_Combine_With_Motors.ino`
*   **Purpose:** Generate a 20kHz square wave to control motor speed without creating an audible whine.
*   **Execution Steps:**
    1.  **Clock Setup:** Enable `RCC_AHB1ENR_GPIOAEN` and `RCC_APB2ENR_TIM1EN`.
    2.  **Pin Multiplexing:** Set `PA8` (PWMA) and `PA9` (PWMB) to Alternate Function 1 (AF1) to link them directly to Timer 1 hardware.
    3.  **Timer Configuration:** Set `TIM1->ARR` (Auto-Reload Register) to the max PWM value (e.g., 4199 for an 84MHz clock divided down to 20kHz).
    4.  **Duty Cycle Control:** Create `pwm_set_duty(channel, value)`. This writes the desired speed directly to `TIM1->CCR1` or `CCR2`.

## 2. Motor Direction Control (`hardware/motor.cpp`)
*   **Source Logic:** `Testing Codes/4.Motors_Combine_With_Motors.ino`
*   **Purpose:** Translate high-level commands (Forward, Reverse) into the specific H-Bridge logic signals required by the TB6612FNG driver.
*   **Execution Steps:**
    1.  Create `motor_set_speed(MotorID motor, int16_t speed)`.
    2.  **Flow:**
        *   If `speed > 0`: Set `IN1 = HIGH`, `IN2 = LOW`. Send `speed` to PWM.
        *   If `speed < 0`: Set `IN1 = LOW`, `IN2 = HIGH`. Send `abs(speed)` to PWM.
        *   If `speed == 0`: Set `IN1 = LOW`, `IN2 = LOW` (Coast) or `HIGH/HIGH` (Brake).

## 3. Hardware Quadrature Encoders (`hardware/encoder.cpp`)
*   **Source Logic:** `Testing Codes/3.Motors_With_Encoders.ino`
*   **Purpose:** Read the magnetic encoders on the backs of the N20 motors using zero CPU cycles by offloading the work to STM32 hardware timers.
*   **Execution Steps:**
    1.  **Left Motor (TIM2):** Configure `PA0` and `PA1` to AF1. Set `TIM2` to **Encoder Mode 3** (`TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1`). This counts on both rising and falling edges of channel A and B, giving maximum resolution (4X).
    2.  **Right Motor (TIM3):** Configure `PA6` and `PA7` to AF2 (Standard for TIM3). Set `TIM3` to Encoder Mode 3.
    3.  **Reading:** Create `encoder_get_count(EncoderID enc)`. This simply reads and returns `TIM2->CNT` or `TIM3->CNT`. Because it's hardware, it will never miss a pulse, even at top speed.

## 4. Proportional-Integral-Derivative Control (`control/pid.cpp`)
*   **Purpose:** A mathematical algorithm that calculates the optimal motor speed required to reach a target distance or heading smoothly.
*   **Flow Breakdown:**
    1.  **Error Calculation:** `error = setpoint - current_value`
    2.  **Proportional (P):** `P_term = Kp * error` (Pushes the robot toward the target).
    3.  **Integral (I):** `I_term += Ki * error * dt` (Corrects steady-state drift, e.g., if one motor is slightly weaker). *Crucial: Implement anti-windup clamping to prevent the I-term from growing infinitely if the wheels are stuck.*
    4.  **Derivative (D):** `D_term = Kd * (error - last_error) / dt` (Acts as a damper, slowing down the robot as it approaches the target to prevent overshoot).
    5.  **Output:** Return `P_term + I_term + D_term`, capped at the maximum allowed PWM value.

---

## 🛠 Testing & Verification for Phase 2
1.  **Flash the robot.**
2.  Trigger Phase 2 Test Mode using `BTN_START`.
3.  The robot should execute a sequence: `STOP` -> `FORWARD` -> `REVERSE` -> `TURN LEFT` -> `TURN RIGHT`.
4.  While moving, the Serial terminal will stream the exact hardware encoder counts. Physically push the robot back and forth to ensure the encoder counts increment and decrement smoothly without dropping values.
