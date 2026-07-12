#include <Arduino.h>

// --- PIN DEFINITIONS ---
#define PWMA  PA8
#define AIN1  PB12
#define AIN2  PB13

#define PWMB  PA9
#define BIN1  PB15
#define BIN2  PA10

#define STBY  PB14

// --- SYSTEM CONSTANTS ---
const float WHEEL_DIAMETER = 34.0f;       // mm
const float COUNTS_PER_REV = 600.0f;
const float MM_PER_COUNT = 0.178f;
const float SAMPLE_TIME = 0.01f;          // 10 ms control loop

// --- GLOBAL VARIABLES ---
float targetLeftSpeed = 0.0f;
float targetRightSpeed = 0.0f;

float leftSpeed = 0.0f;  
float rightSpeed = 0.0f; 

float leftPWM = 0;
float rightPWM = 0;

int32_t lastLeftCount = 0;
int16_t lastRightCount = 0;

unsigned long lastPID = 0;

// --- PID STRUCTURE ---
struct PIDController
{
    float kp;
    float ki;
    float kd;

    float error;
    float previousError;

    float integral;
    float derivative;
};

PIDController leftPID;
PIDController rightPID;

// ==========================================
// MOTOR DRIVER FUNCTIONS
// ==========================================
void setupMotorDriver()
{
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
}

void motorAForward()  { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  }
void motorAReverse()  { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); }
void motorBForward()  { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  }
void motorBReverse()  { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }

void motorAStop()
{
  analogWrite(PWMA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

void motorBStop()
{
  analogWrite(PWMB, 0);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void setMotorPWM(uint16_t pwmLeft, uint16_t pwmRight)
{
  // Now safely applies 0-4999 values because we set the resolution in setup()
  analogWrite(PWMA, pwmLeft);
  analogWrite(PWMB, pwmRight);
}

// ==========================================
// ENCODER FUNCTIONS
// ==========================================
void setupLeftEncoder()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  GPIOA->MODER &= ~(3<<(0*2)); GPIOA->MODER &= ~(3<<(1*2));
  GPIOA->MODER |= (2<<(0*2));  GPIOA->MODER |= (2<<(1*2));

  GPIOA->AFR[0] &= ~(0xF<<0);  GPIOA->AFR[0] &= ~(0xF<<4);
  GPIOA->AFR[0] |= (1<<0);     GPIOA->AFR[0] |= (1<<4);

  TIM2->CR1=0; TIM2->CR2=0; TIM2->SMCR=0; TIM2->CCMR1=0; TIM2->CCER=0;
  TIM2->PSC=0; TIM2->ARR=0xFFFFFFFF; 

  TIM2->SMCR |= TIM_SMCR_SMS_0; TIM2->SMCR |= TIM_SMCR_SMS_1;
  TIM2->CCMR1 |= TIM_CCMR1_CC1S_0; TIM2->CCMR1 |= TIM_CCMR1_CC2S_0;
  TIM2->CCMR1 |= (3<<4); TIM2->CCMR1 |= (3<<12);
  
  TIM2->CNT=0;
  TIM2->CR1 |= TIM_CR1_CEN;
}

void setupRightEncoder()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  GPIOA->MODER &= ~(3<<(6*2)); GPIOA->MODER &= ~(3<<(7*2));
  GPIOA->MODER |= (2<<(6*2));  GPIOA->MODER |= (2<<(7*2));

  GPIOA->AFR[0] &= ~(0xF<<(6*4)); GPIOA->AFR[0] &= ~(0xF<<(7*4));
  GPIOA->AFR[0] |= (2<<(6*4));    GPIOA->AFR[0] |= (2<<(7*4)); 

  TIM3->CR1=0; TIM3->CR2=0; TIM3->SMCR=0; TIM3->CCMR1=0; TIM3->CCER=0;
  TIM3->PSC=0; TIM3->ARR=0xFFFF; 

  TIM3->SMCR |= TIM_SMCR_SMS_0; TIM3->SMCR |= TIM_SMCR_SMS_1;
  TIM3->CCMR1 |= TIM_CCMR1_CC1S_0; TIM3->CCMR1 |= TIM_CCMR1_CC2S_0;
  TIM3->CCMR1 |= (3<<4); TIM3->CCMR1 |= (3<<12);
  
  TIM3->CNT=0;
  TIM3->CR1 |= TIM_CR1_CEN;
}

int32_t getLeftDelta()
{
  int32_t current = TIM2->CNT;
  int32_t delta = current - lastLeftCount;
  lastLeftCount = current;
  return delta;
}

int32_t getRightDelta()
{
  int16_t current = (int16_t)TIM3->CNT;
  int16_t delta = current - lastRightCount;
  lastRightCount = current;
  return (int32_t)delta;
}

void updateSpeed()
{
    int32_t leftDelta = getLeftDelta();
    
    // Reverse the right encoder data so that moving "forward" 
    // reads as a positive speed for the PID controller.
    int32_t rightDelta = -getRightDelta(); 

    leftSpeed = (leftDelta * MM_PER_COUNT) / SAMPLE_TIME;
    rightSpeed = (rightDelta * MM_PER_COUNT) / SAMPLE_TIME;
}

// ==========================================
// PID & CONTROL FUNCTIONS
// ==========================================
void setupPID()
{
    leftPID.kp = 1.8f;
    leftPID.ki = 0.8f;
    leftPID.kd = 0.02f;

    leftPID.error = 0;
    leftPID.previousError = 0;
    leftPID.integral = 0;
    leftPID.derivative = 0;

    rightPID = leftPID;
}

float calculatePID(PIDController &pid, float target, float actual)
{
    pid.error = target - actual;

    pid.integral += pid.error * SAMPLE_TIME;

    if(pid.integral > 500)  pid.integral = 500;
    if(pid.integral < -500) pid.integral = -500;

    pid.derivative = (pid.error - pid.previousError) / SAMPLE_TIME;

    float output = 
        pid.kp * pid.error + 
        pid.ki * pid.integral + 
        pid.kd * pid.derivative;

    pid.previousError = pid.error;

    return output;
}

void setTargetSpeed(float left, float right)
{
    targetLeftSpeed = left;
    targetRightSpeed = right;
}

void updateMotorPID()
{
    updateSpeed();

    leftPWM += calculatePID(leftPID, targetLeftSpeed, leftSpeed);
    rightPWM += calculatePID(rightPID, targetRightSpeed, rightSpeed);

    leftPWM = constrain(leftPWM, 0, 4999);
    rightPWM = constrain(rightPWM, 0, 4999);

    setMotorPWM((uint16_t)leftPWM, (uint16_t)rightPWM);
}

// ==========================================
// HIGH-LEVEL MOVEMENT COMMANDS
// ==========================================
void robotForward(float speed)
{
    motorAForward();
    motorBForward();
    setTargetSpeed(speed, speed);
}

// ==========================================
// MAIN LOOP
// ==========================================
void setup() 
{
  Serial.begin(115200);
  
  // Set STM32 hardware PWM to 13-bit resolution (values from 0 to 8191)
  // This allows your PID 0-4999 range to work natively without overflowing
  analogWriteResolution(13);
  
  setupMotorDriver();
  setupLeftEncoder();
  setupRightEncoder();
  setupPID();
  
  delay(2000); 
  robotForward(300);
}

void loop() 
{
    if(millis() - lastPID >= 10)
    {
        lastPID = millis();
        updateMotorPID();
        
        Serial.print("Target:"); Serial.print(targetLeftSpeed); Serial.print(",");
        Serial.print("Left_Actual:"); Serial.print(leftSpeed); Serial.print(",");
        Serial.print("Right_Actual:"); Serial.println(rightSpeed);
    }
}