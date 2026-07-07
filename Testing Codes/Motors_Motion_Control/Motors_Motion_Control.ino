#include <Arduino.h>

// --- SYSTEM CONSTANTS ---
const float WHEEL_DIAMETER = 34.0f;          // mm
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;
const float COUNTS_PER_REV = 600.0f;
const float MM_PER_COUNT = 0.178f;

// --- GLOBAL VARIABLES ---
int32_t lastLeftCount = 0;
int16_t lastRightCount = 0; // Explicitly 16-bit to match TIM3

int32_t totalLeftCount = 0;
int32_t totalRightCount = 0;

float leftCPS = 0, rightCPS = 0;
float leftRPS = 0, rightRPS = 0;
float leftRPM = 0, rightRPM = 0;
float leftRad = 0, rightRad = 0;
float leftSpeed = 0, rightSpeed = 0; // mm/s

unsigned long lastUpdate = 0;

// --- SETUP FUNCTIONS ---

void setupLeftEncoder()
{
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
  TIM2->CR1 = 0;
  TIM2->CR2 = 0;
  TIM2->SMCR = 0;
  TIM2->CCMR1 = 0;
  TIM2->CCER = 0;

  TIM2->PSC = 0;
  TIM2->ARR = 0xFFFFFFFF; // TIM2 is 32-bit

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

void setupRightEncoder()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  // PA6 PA7 Alternate Function (TIM3 CH1 and CH2)
  GPIOA->MODER &= ~(3 << (6 * 2));
  GPIOA->MODER &= ~(3 << (7 * 2));
  GPIOA->MODER |= (2 << (6 * 2));
  GPIOA->MODER |= (2 << (7 * 2));

  // AF2 for TIM3
  GPIOA->AFR[0] &= ~(0xF << (6 * 4));
  GPIOA->AFR[0] &= ~(0xF << (7 * 4));
  GPIOA->AFR[0] |= (2 << (6 * 4)); 
  GPIOA->AFR[0] |= (2 << (7 * 4)); 

  // Reset timer
  TIM3->CR1 = 0;
  TIM3->CR2 = 0;
  TIM3->SMCR = 0;
  TIM3->CCMR1 = 0;
  TIM3->CCER = 0;

  TIM3->PSC = 0;
  TIM3->ARR = 0xFFFF; // TIM3 is 16-bit

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

// --- CORE UPDATE LOGIC ---

int32_t getLeftDelta()
{
  int32_t current = TIM2->CNT;
  int32_t delta = current - lastLeftCount;
  lastLeftCount = current;
  return delta;
}

int32_t getRightDelta()
{
  // 16-bit math ensures delta is correct even when timer rolls over
  int16_t current = (int16_t)TIM3->CNT;
  int16_t delta = current - lastRightCount; 
  lastRightCount = current;
  return (int32_t)delta; 
}

void updateSpeed(float dtSeconds)
{
  int32_t leftDelta = getLeftDelta();
  int32_t rightDelta = getRightDelta();

  // Accumulate totals safely for infinite distance tracking
  totalLeftCount += leftDelta;
  totalRightCount += rightDelta;

  // Calculate speeds based on actual elapsed time (dtSeconds)
  leftCPS = (float)leftDelta / dtSeconds;
  rightCPS = (float)rightDelta / dtSeconds;

  leftRPS = leftCPS / COUNTS_PER_REV;
  rightRPS = rightCPS / COUNTS_PER_REV;

  leftRPM = leftRPS * 60.0f;
  rightRPM = rightRPS * 60.0f;

  leftRad = leftRPS * TWO_PI;
  rightRad = rightRPS * TWO_PI;

  leftSpeed = leftRPS * WHEEL_CIRCUMFERENCE;
  rightSpeed = rightRPS * WHEEL_CIRCUMFERENCE;
}

// --- UTILITY FUNCTIONS ---

void resetEncoders()
{
  TIM2->CNT = 0;
  TIM3->CNT = 0;
  lastLeftCount = 0;
  lastRightCount = 0;
  totalLeftCount = 0;
  totalRightCount = 0;
}

float getLeftDistance()
{
  return (float)totalLeftCount * MM_PER_COUNT;
}

float getRightDistance()
{
  return (float)totalRightCount * MM_PER_COUNT; 
}

float getRobotDistance()
{
  return (getLeftDistance() + getRightDistance()) / 2.0f;
}


// --- MAIN ARDUINO LOOP ---

void setup() {
  Serial.begin(115200);
  setupLeftEncoder();
  setupRightEncoder(); 
}

void loop() {
  unsigned long now = millis();
  unsigned long deltaMs = now - lastUpdate;

  // Run update roughly every 10ms
  if (deltaMs >= 10) 
  {
    // Convert actual elapsed ms to seconds to ensure accurate RPM
    float dtSeconds = (float)deltaMs / 1000.0f; 
    
    updateSpeed(dtSeconds);
    lastUpdate = now;

    // Optional: Print output inside the timing loop so it doesn't flood the serial monitor
    Serial.print("Left RPM: ");
    Serial.print(leftRPM);
    Serial.print("   Right RPM: ");
    Serial.print(rightRPM);
    Serial.print("   Left Speed: ");
    Serial.print(leftSpeed);
    Serial.print(" mm/s");
    Serial.print("   Right Speed: ");
    Serial.println(rightSpeed);
  }
}