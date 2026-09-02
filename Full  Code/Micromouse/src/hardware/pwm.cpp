/**
 * @file pwm.cpp
 * @brief TIM1 PWM implementation using register-level access.
 *
 * @see pwm.h for public API documentation
 */

#include "pwm.h"
#include <Arduino.h>

void pwm_init(void) {
    // 1. Enable peripheral clocks for GPIOA and TIM1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // 2. Configure PA8 (TIM1_CH1) and PA9 (TIM1_CH2) for Alternate Function mode (binary 10 = 2)
    GPIOA->MODER &= ~(3 << (8 * 2));
    GPIOA->MODER &= ~(3 << (9 * 2));
    GPIOA->MODER |= (2 << (8 * 2));
    GPIOA->MODER |= (2 << (9 * 2));

    // 3. Set AF1 (TIM1) in High Alternate Function Register (AFR[1] controls PA8..PA15)
    GPIOA->AFR[1] &= ~(0xF << ((8 - 8) * 4));
    GPIOA->AFR[1] &= ~(0xF << ((9 - 8) * 4));
    GPIOA->AFR[1] |= (1 << ((8 - 8) * 4));
    GPIOA->AFR[1] |= (1 << ((9 - 8) * 4));

    // 4. Reset TIM1 configuration registers to clean state
    TIM1->CR1 = 0;
    TIM1->CR2 = 0;
    TIM1->SMCR = 0;
    TIM1->DIER = 0;
    TIM1->CCER = 0;
    TIM1->CCMR1 = 0;
    TIM1->CCMR2 = 0;

    // 5. Set Prescaler = 0, ARR = PWM_MAX (4199) -> 84 MHz / 4200 = 20 kHz PWM
    TIM1->PSC = 0;
    TIM1->ARR = PWM_MAX;
    TIM1->CNT = 0;

    // 6. Set initial duty cycle to 0%
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;

    // 7. Configure PWM Mode 1 (binary 110 = 6) on CH1 and CH2
    TIM1->CCMR1 |= (6 << 4);
    TIM1->CCMR1 |= (6 << 12);

    // 8. Enable preload registers on CH1 and CH2
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM1->CCMR1 |= TIM_CCMR1_OC2PE;

    // 9. Enable output channels CH1 and CH2
    TIM1->CCER |= TIM_CCER_CC1E;
    TIM1->CCER |= TIM_CCER_CC2E;

    // 10. Enable Main Output (BDTR->MOE is mandatory for advanced timers like TIM1 to output PWM)
    TIM1->BDTR |= TIM_BDTR_MOE;

    // 11. Force update event to load shadow registers
    TIM1->EGR = TIM_EGR_UG;

    // 12. Enable auto-reload preload
    TIM1->CR1 |= TIM_CR1_ARPE;

    // 13. Start TIM1 counter
    TIM1->CR1 |= TIM_CR1_CEN;
}

void pwm_set_left(uint16_t duty) {
    if (duty > PWM_MAX) duty = PWM_MAX;
    /* LEFT motor  = PA9 = TIM1_CH2 → CCR2 (was wrongly CCR1) */
    TIM1->CCR2 = duty;
}

void pwm_set_right(uint16_t duty) {
    if (duty > PWM_MAX) duty = PWM_MAX;
    /* RIGHT motor = PA8 = TIM1_CH1 → CCR1 (was wrongly CCR2) */
    TIM1->CCR1 = duty;
}

void pwm_set_both(uint16_t left_duty, uint16_t right_duty) {
    pwm_set_left(left_duty);
    pwm_set_right(right_duty);
}
