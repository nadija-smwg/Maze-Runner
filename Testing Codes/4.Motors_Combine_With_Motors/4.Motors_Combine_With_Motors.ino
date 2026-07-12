#include <Arduino.h>

#define PWMA   PA8
#define AIN1   PB12
#define AIN2   PB13

#define PWMB   PA9
#define BIN1   PB15
#define BIN2   PA10

#define STBY   PB14

void setupMotorDriver() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);
}

void setupPWM() {
  // Enable clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  // PA8 and PA9 -> Alternate Function
  GPIOA->MODER &= ~(3 << (8 * 2));
  GPIOA->MODER &= ~(3 << (9 * 2));

  GPIOA->MODER |= (2 << (8 * 2));
  GPIOA->MODER |= (2 << (9 * 2));

  // AF1 = TIM1
  GPIOA->AFR[1] &= ~(0xF << 0);
  GPIOA->AFR[1] &= ~(0xF << 4);

  GPIOA->AFR[1] |= (1 << 0);
  GPIOA->AFR[1] |= (1 << 4);

  // Reset timer
  TIM1->CR1 = 0;
  TIM1->CR2 = 0;
  TIM1->SMCR = 0;
  TIM1->DIER = 0;
  TIM1->CCER = 0;
  TIM1->CCMR1 = 0;
  TIM1->CCMR2 = 0;

  // 20 kHz PWM
  TIM1->PSC = 0;
  TIM1->ARR = 4999;
  TIM1->CNT = 0;

  // Duty cycle = 0%
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;

  // PWM Mode 1
  TIM1->CCMR1 |= (6 << 4);
  TIM1->CCMR1 |= (6 << 12);

  // Enable preload
  TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
  TIM1->CCMR1 |= TIM_CCMR1_OC2PE;

  // Enable outputs
  TIM1->CCER |= TIM_CCER_CC1E;
  TIM1->CCER |= TIM_CCER_CC2E;

  // Main output enable
  TIM1->BDTR |= TIM_BDTR_MOE;

  // Update registers
  TIM1->EGR = TIM_EGR_UG;

  // Auto-reload preload
  TIM1->CR1 |= TIM_CR1_ARPE;

  // Start timer
  TIM1->CR1 |= TIM_CR1_CEN;
}

void leftForward() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
}

void leftReverse() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
}

void leftBrake() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
}

void leftCoast() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

void rightForward() {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void rightReverse() {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void rightBrake() {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
}

void rightCoast() {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void setLeftPWM(uint16_t pwm) {
  if (pwm > 4999) {
    pwm = 4999;
  }
  TIM1->CCR1 = pwm;
}

void setRightPWM(uint16_t pwm) {
  if (pwm > 4999) {
    pwm = 4999;
  }
  TIM1->CCR2 = pwm;
}

void setMotorPWM(uint16_t left, uint16_t right) {
  setLeftPWM(left);
  setRightPWM(right);
}

void setLeftMotor(int16_t pwm) {
  if (pwm > 4999) {
    pwm = 4999;
  }

  if (pwm < -4999) {
    pwm = -4999;
  }

  if (pwm > 0) {
    leftForward();
    TIM1->CCR1 = pwm;
  } else if (pwm < 0) {
    leftReverse();
    TIM1->CCR1 = -pwm;
  } else {
    leftBrake();
    TIM1->CCR1 = 0;
  }
}

void setRightMotor(int16_t pwm) {
  if (pwm > 4999) {
    pwm = 4999;
  }

  if (pwm < -4999) {
    pwm = -4999;
  }

  if (pwm > 0) {
    rightForward();
    TIM1->CCR2 = pwm;
  } else if (pwm < 0) {
    rightReverse();
    TIM1->CCR2 = -pwm;
  } else {
    rightBrake();
    TIM1->CCR2 = 0;
  }
}

void robotForward(uint16_t pwm) {
  setLeftMotor(pwm);
  setRightMotor(pwm);
}

void robotReverse(uint16_t pwm) {
  setLeftMotor(-pwm);
  setRightMotor(-pwm);
}

void robotTurnLeft(uint16_t pwm) {
  setLeftMotor(-pwm);
  setRightMotor(pwm);
}

void robotTurnRight(uint16_t pwm) {
  setLeftMotor(pwm);
  setRightMotor(-pwm);
}

void robotStop() {
  setLeftMotor(0);
  setRightMotor(0);
}

void setupLeftEncoder() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  // PA0 PA1 Alternate Function (TIM2 CH1 and CH2)
  GPIOA->MODER &= ~(3 << (0 * 2));
  GPIOA->MODER &= ~(3 << (1 * 2));
  GPIOA->MODER |= (2 << (0 * 2));
  GPIOA->MODER |= (2 << (1 * 2));

  // AF1 for TIM2
  GPIOA->AFR[0] &= ~(0xF << 0);
  GPIOA->AFR[0] &= ~(0xF << 4);
  GPIOA->AFR[0] |= (1 << 0);
  GPIOA->AFR[0] |= (1 << 4);

  // Reset timer
  TIM2->CR1   = 0;
  TIM2->CR2   = 0;
  TIM2->SMCR  = 0;
  TIM2->CCMR1 = 0;
  TIM2->CCER  = 0;

  TIM2->PSC = 0;
  TIM2->ARR = 0xFFFFFFFF; // TIM2 is 32-bit on most modern STM32s

  // Encoder Mode 3
  TIM2->SMCR |= TIM_SMCR_SMS_0;
  TIM2->SMCR |= TIM_SMCR_SMS_1;

  // CH1 CH2 Input
  TIM2->CCMR1 |= TIM_CCMR1_CC1S_0;
  TIM2->CCMR1 |= TIM_CCMR1_CC2S_0;

  // Input Filter
  TIM2->CCMR1 |= (3 << 4);
  TIM2->CCMR1 |= (3 << 12);

  TIM2->CNT = 0;
  TIM2->CR1 |= TIM_CR1_CEN;
}

void setupRightEncoder() {
  // Note: Assuming standard STM32 mappings where PA6 and PA7 are TIM3 CH1 and CH2
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  // PA6 PA7 Alternate Function (TIM3 CH1 and CH2)
  GPIOA->MODER &= ~(3 << (6 * 2));
  GPIOA->MODER &= ~(3 << (7 * 2));
  GPIOA->MODER |= (2 << (6 * 2));
  GPIOA->MODER |= (2 << (7 * 2));

  // AF2 for TIM3 (Standard on STM32F4/etc)
  GPIOA->AFR[0] &= ~(0xF << (6 * 4));
  GPIOA->AFR[0] &= ~(0xF << (7 * 4));
  GPIOA->AFR[0] |= (2 << (6 * 4)); 
  GPIOA->AFR[0] |= (2 << (7 * 4)); 

  // Reset timer
  TIM3->CR1   = 0;
  TIM3->CR2   = 0;
  TIM3->SMCR  = 0;
  TIM3->CCMR1 = 0;
  TIM3->CCER  = 0;

  TIM3->PSC = 0;
  TIM3->ARR = 0xFFFF; // TIM3 is typically a 16-bit timer

  // Encoder Mode 3
  TIM3->SMCR |= TIM_SMCR_SMS_0;
  TIM3->SMCR |= TIM_SMCR_SMS_1;

  // CH1 CH2 Input
  TIM3->CCMR1 |= TIM_CCMR1_CC1S_0;
  TIM3->CCMR1 |= TIM_CCMR1_CC2S_0;

  // Input Filter
  TIM3->CCMR1 |= (3 << 4);
  TIM3->CCMR1 |= (3 << 12);

  TIM3->CNT = 0;
  TIM3->CR1 |= TIM_CR1_CEN;
}

int32_t lastLeftCount = 0;
int32_t lastRightCount = 0;

int32_t getLeftCount() {
  return (int32_t)TIM2->CNT; // 32-bit timer reads naturally
}

int32_t getRightCount() {
  // 1. (int16_t) converts 65535 back into -1
  // 2. (int32_t) safely upgrades it to match your 32-bit left timer
  return (int32_t)(int16_t)TIM3->CNT;
}

void resetLeftEncoder() {
  TIM2->CNT = 0;
}

void resetRightEncoder() {
  TIM3->CNT = 0;
}

void resetEncoders() {
  TIM2->CNT = 0;
  TIM3->CNT = 0;

  lastLeftCount = 0;
  lastRightCount = 0;
}

int32_t getLeftDelta() {
  int32_t current = TIM2->CNT;
  int32_t delta = current - lastLeftCount;
  lastLeftCount = current;
  return delta;
}

int32_t getRightDelta() {
  // Cast to int16_t to properly handle 16-bit underflow/overflow math
  int16_t current = (int16_t)TIM3->CNT;
  int16_t delta = current - (int16_t)lastRightCount;
  lastRightCount = current;
  return (int32_t)delta;
}

// ******** Motor Specifications ********
const float GEAR_RATIO = 65.0f;   
const float PPR = 7.0f;             
const float QUADRATURE = 4.0f;

const float CPR = PPR * GEAR_RATIO * QUADRATURE;

const float WHEEL_DIAMETER = 34.0f;     
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

float countsToRPM(float countsPerSecond)
{
    return (countsPerSecond * 60.0f) / CPR;
}

float rpmToSpeed(float rpm)
{
    return rpm * WHEEL_CIRCUMFERENCE / 60.0f;
}

void characterizeMotor(uint16_t pwm)
{
    // Start motor
    robotForward(pwm);

    // Allow motor to reach steady speed
    delay(1000);

    // Reset encoders AFTER acceleration
    resetEncoders();

    uint32_t start = millis();

    // Measure for exactly 2 seconds
    delay(2000);

    uint32_t elapsed = millis() - start;

    robotStop();

    int32_t leftCounts  = getLeftCount();
    int32_t rightCounts = getRightCount();

    // Counts per second
    float leftCPS  = leftCounts  * 1000.0f / elapsed;
    float rightCPS = rightCounts * 1000.0f / elapsed;

    // Wheel RPM
    float leftRPM  = countsToRPM(leftCPS);
    float rightRPM = countsToRPM(rightCPS);

    // Linear speed
    float leftSpeed  = rpmToSpeed(leftRPM);
    float rightSpeed = rpmToSpeed(rightRPM);

    Serial.println("--------------------------------");

    Serial.print("PWM = ");
    Serial.println(pwm);

    Serial.print("Elapsed(ms) = ");
    Serial.println(elapsed);

    Serial.print("Left Counts = ");
    Serial.println(leftCounts);

    Serial.print("Right Counts = ");
    Serial.println(rightCounts);

    Serial.print("Left CPS = ");
    Serial.println(leftCPS);

    Serial.print("Right CPS = ");
    Serial.println(rightCPS);

    Serial.print("Left RPM = ");
    Serial.println(leftRPM);

    Serial.print("Right RPM = ");
    Serial.println(rightRPM);

    Serial.print("Left Speed (mm/s) = ");
    Serial.println(leftSpeed);

    Serial.print("Right Speed (mm/s) = ");
    Serial.println(rightSpeed);

    Serial.println("--------------------------------");

    delay(3000);
}

void accelerationTest()
{
    resetEncoders();

    robotForward(4999);

    uint32_t start = millis();

    while(millis()-start < 3000)
    {
        delay(200);

        int32_t L = getLeftCount();
        int32_t R = getRightCount();

        Serial.print("Time=");
        Serial.print(millis()-start);

        Serial.print(" L=");
        Serial.print(L);

        Serial.print(" R=");
        Serial.println(R);
    }

    robotStop();
}


void setup() {
  Serial.begin(115200);
  setupMotorDriver();
  setupPWM();
  setupLeftEncoder();
  setupRightEncoder();
  resetEncoders();
  
  Serial.println("System Initialized. Starting test sequence...");
  delay(2000); // Give the user time to open the serial monitor
}

void loop(){
  characterizeMotor(500);
  characterizeMotor(1000);
  characterizeMotor(1500);
  characterizeMotor(2000);
  characterizeMotor(2500);
  characterizeMotor(3000);
  characterizeMotor(3500);
  characterizeMotor(4000);
  characterizeMotor(4500);
  characterizeMotor(4999);
}

// void loop()
// {
//     accelerationTest();

//     while(1);
// }

