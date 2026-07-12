#include <Arduino.h>

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

const float MM_PER_COUNT = 0.178f;

float getLeftDistance() {
  return getLeftCount() * MM_PER_COUNT;
}

float getRightDistance() {
  // If you travel long distances, base this on an accumulated counter variable 
  // instead of getRightCount() directly, to bypass the 16-bit limit.
  return getRightCount() * MM_PER_COUNT; 
}

float getRobotDistance() {
  return (getLeftDistance() + getRightDistance()) / 2.0f;
}

void setup() {
  Serial.begin(115200);
  setupLeftEncoder();
  setupRightEncoder(); // Added right encoder setup
}

void loop() {
  Serial.print("Left : ");
  Serial.print(getLeftCount());
  Serial.print(" Right : ");
  Serial.println(getRightCount());
  delay(100);
}