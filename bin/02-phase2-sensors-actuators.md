# Phase 2 — Sensor & Actuator Interfacing

Prerequisite: Phase 1 checkpoints complete (timers, GPIO, EXTI, ISR discipline).

## 2.1 PWM Motor Control (TB6612FNG)

The TB6612FNG takes, per motor: `PWM`, `IN1`, `IN2` (direction), plus a shared
`STBY`. Direction truth table:

| IN1 | IN2 | Result |
|---|---|---|
| 1 | 0 | Forward |
| 0 | 1 | Reverse |
| 0 | 0 | Coast (high-Z) |
| 1 | 1 | Brake (short) |

PWM duty cycle controls speed (0–100%), independent of direction pins.

### Timer PWM Setup
Use TIM1 (advanced timer, good for motors) with 2 channels for left/right PWM.
Target PWM frequency: **20 kHz** — above audible range (avoids motor whine)
and fast enough not to cause mechanical ripple at your control loop rate.

```c
// PCLK2 = 100MHz (TIM1 is on APB2)
// Want 20kHz PWM with good duty resolution:
// PSC = 0  → timer clock = 100MHz
// ARR = 4999 → 100MHz / 5000 = 20kHz, duty resolution = 1/5000 (plenty)

htim1.Instance = TIM1;
htim1.Init.Prescaler = 0;
htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
htim1.Init.Period = 4999;
htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
HAL_TIM_PWM_Init(&htim1);

HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // Left motor PWM
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // Right motor PWM
```

```c
void set_motor(TIM_HandleTypeDef *htim, uint32_t channel,
               GPIO_TypeDef *in1_port, uint16_t in1_pin,
               GPIO_TypeDef *in2_port, uint16_t in2_pin,
               int16_t speed /* -1000..1000 */) {
    if (speed >= 0) {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
        speed = -speed;
    }
    if (speed > 1000) speed = 1000;
    uint32_t ccr = ((uint32_t)speed * 4999) / 1000; // scale to ARR
    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}
```

**Design note:** normalize your motor command API to a signed integer range
(e.g. -1000..1000) independent of ARR, so PID and maze-logic code never needs
to know timer register details. This is the abstraction boundary between
"control" and "hardware" layers — keep it clean, you'll thank yourself during
tuning.

**Checkpoint 2.1:** Drive both motors forward at 30% duty, then reverse, then
brake, confirmed on the physical robot (wheels off the ground first!).

---

## 2.2 Quadrature Encoders via Hardware Timer Encoder Mode

This is the single most important "free precision" feature on STM32 for a
micromouse. Instead of using EXTI + software counting (which drops ticks at
high RPM and burns CPU), configure a timer in **Encoder Mode**: the hardware
quadrature-decodes channel A/B automatically and the timer's counter register
IS your tick count — increasing or decreasing based on direction, entirely in
silicon.

```c
// TIM2 CH1 = Encoder A, TIM2 CH2 = Encoder B (left motor)
// TIM3 CH1/CH2 = right motor encoder
htim2.Instance = TIM2;
htim2.Init.Period = 0xFFFFFFFF;  // TIM2 is 32-bit on F411 — full range, rarely wraps
htim2.Init.Prescaler = 0;
TIM_Encoder_InitTypeDef sConfig = {0};
sConfig.EncoderMode = TIM_ENCODERMODE_TI12;  // decode both edges of both channels = 4x resolution
sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
sConfig.IC1Filter = 6;  // digital filter — debounces noisy encoder edges
sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
sConfig.IC2Filter = 6;
HAL_TIM_Encoder_Init(&htim2, &sConfig);
HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
```

Reading it:
```c
int32_t left_ticks = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
```

Note: the counter is free-running (unsigned, wraps at 2^32 for 32-bit timers).
For **delta ticks per control period**, capture and subtract each loop:
```c
static int32_t last_left = 0;
int32_t now_left = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
int32_t delta_left = now_left - last_left;  // signed subtraction handles wraparound correctly
last_left = now_left;
```

**Why `IC1Filter = 6`:** raw encoder signals from cheap magnetic encoders are
electrically noisy; the hardware input filter samples N times before
accepting an edge, rejecting glitches without CPU involvement. Tune this
value empirically — too high and you'll miss real edges at high RPM.

**16-bit timers (TIM3/TIM4):** if you only have 16-bit timers free for the
second encoder, `Period` maxes at 0xFFFF and you must handle wraparound more
carefully (still fine with signed delta subtraction, just wraps more often —
harmless as long as you read frequently, e.g. every 1ms).

**Checkpoint 2.2:** Spin a wheel by hand, print `left_ticks`/`right_ticks` over
UART or SWO, confirm direction sign is correct for both forward and reverse,
and compute your **ticks-per-wheel-revolution** empirically (mark the wheel,
rotate exactly once, read the tick delta).

---

## 2.3 ADC for IR Distance Sensors

Analog IR sensors (e.g. Sharp GP2Y0A21) output a voltage inversely related to
distance (non-linear — you'll need a lookup table or fitted curve, not a
straight scale).

For a micromouse with 3-5 analog sensors, **don't poll ADC in the main loop
one channel at a time with blocking `HAL_ADC_PollForConversion`** — it's slow
and jittery. Instead use **ADC scan mode + DMA**, so all channels are
continuously converted into a memory buffer in the background with zero CPU
overhead per sample.

```c
// CubeMX: ADC1, enable however many IN channels you need (e.g. IN0, IN1, IN4),
// Scan Conversion Mode = Enabled, Continuous Conversion = Enabled,
// DMA Continuous Requests = Enabled, rank order = your channel order

#define NUM_ADC_CH 3
uint16_t adc_buf[NUM_ADC_CH];  // DMA writes here continuously

HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, NUM_ADC_CH);

// Anywhere in main loop, just read the latest values — always fresh:
uint16_t front_ir  = adc_buf[0];
uint16_t left_ir    = adc_buf[1];
uint16_t right_ir   = adc_buf[2];
```

Convert raw ADC (0-4095 for 12-bit) to millivolts, then to distance via your
calibration curve:
```c
float raw_to_mv(uint16_t raw) { return (raw * 3300.0f) / 4095.0f; }

// Sharp-sensor-style inverse relationship, coefficients from your own calibration:
float mv_to_distance_mm(float mv) {
    if (mv < 400.0f) return 800.0f; // clamp — out of reliable range
    return (constA / (mv - constB)) - constC; // fit via curve_fit against measured data
}
```

**Calibration methodology (do this for real, don't guess coefficients):**
place the sensor at known distances (e.g. 20mm to 200mm in 10mm steps against
the maze wall material specifically — reflectivity matters), log raw ADC, fit
a curve (`1/x` family fits Sharp sensors well) in Python/Excel, hardcode the
fitted constants into firmware.

**If using VL53L0X ToF sensors instead/additionally:** skip ADC entirely,
these are I2C digital sensors returning millimeters directly — see 2.4.
Many competitive mice use ToF for walls because they're far more linear and
less affected by surface color/reflectivity than analog IR.

**Checkpoint 2.3:** Get continuous DMA-driven ADC readings streaming, build
your calibration table for at least one sensor, and validate: sensor reads
consistent, correct-direction distance when you move a wall panel toward/away.

---

## 2.4 I2C for MPU6050 (IMU) and/or VL53L0X (ToF)

### I2C Bus Setup
```c
hi2c1.Instance = I2C1;
hi2c1.Init.ClockSpeed = 400000; // 400kHz Fast Mode — both MPU6050 and VL53L0X support this
hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
hi2c1.Init.OwnAddress1 = 0;
hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
HAL_I2C_Init(&hi2c1);
```

### MPU6050 — reading gyro Z (yaw rate) for turn control
MPU6050 register map essentials: `PWR_MGMT_1 (0x6B)` to wake it from sleep,
`GYRO_ZOUT_H/L (0x47/0x48)` for yaw rate.

```c
#define MPU6050_ADDR (0x68 << 1)  // HAL wants the 8-bit address (7-bit addr shifted left)

void mpu6050_init(void) {
    uint8_t data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 100); // wake up
    data = 0x00; // gyro full scale ±250 deg/s (most precise range)
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &data, 1, 100);
}

float mpu6050_read_gyro_z_dps(void) {
    uint8_t buf[2];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x47, 1, buf, 2, 100);
    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
    return raw / 131.0f; // LSB sensitivity for ±250dps range, per datasheet
}
```

**Critical: calibrate gyro bias at startup.** MCUs and MEMS gyros have
nonzero output at rest ("bias"/"offset"). Average ~500 samples with the robot
stationary at boot, subtract that bias from every subsequent reading, or your
yaw integration will drift steadily even sitting still.

```c
float gyro_bias = 0;
void calibrate_gyro(void) {
    float sum = 0;
    for (int i = 0; i < 500; i++) {
        sum += mpu6050_read_gyro_z_dps();
        HAL_Delay(2);
    }
    gyro_bias = sum / 500.0f;
}
```

### VL53L0X — Time-of-Flight distance
This sensor needs a real driver (register sequence is complex — timing
budgets, SPAD calibration). **Don't hand-roll this**: use ST's official API
or a well-tested port (e.g. the Pololu VL53L0X Arduino-derived C library,
adaptable to HAL since it just needs `i2c_write`/`i2c_read` glue functions).
Your job is writing the **HAL I2C glue layer**, not reimplementing the ranging
algorithm.

```c
// Glue functions the VL53L0X driver expects — implement using HAL:
int VL53L0X_WriteMulti(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Write(&hi2c1, addr << 1, reg, 1, data, len, 100) == HAL_OK ? 0 : -1;
}
int VL53L0X_ReadMulti(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(&hi2c1, addr << 1, reg, 1, data, len, 100) == HAL_OK ? 0 : -1;
}
```

**Multiple VL53L0X on one bus:** they all boot at address `0x29`. Wire each
sensor's `XSHUT` pin to a separate GPIO, hold all but one in reset at boot,
initialize + reassign that one to a unique address via the driver's
`setAddress()` call, release the next, repeat.

**Checkpoint 2.4:** Get calibrated gyro-Z streaming with near-zero drift at
rest over ~10 seconds. If using ToF, get at least one sensor returning stable
millimeter readings and confirm multi-sensor address reassignment works if
you have more than one.

---

## Phase 2 Summary — What You Should Now Be Able To Do
- Drive both motors bidirectionally at arbitrary speed via PWM + direction pins.
- Read precise, hardware-decoded encoder ticks with correct sign, with a
  known ticks-per-revolution constant.
- Stream ADC IR sensor data continuously via DMA with a calibrated
  voltage→distance conversion.
- Read calibrated, bias-corrected gyro yaw rate over I2C, and/or get working
  ToF distance readings with proper multi-sensor addressing.

Confirm and we'll move to **[Phase 3: Kinematics & Control
Systems](03-phase3-kinematics-control.md)** — dead reckoning, PID for
straight-line and turning motion, and sensor fusion.
