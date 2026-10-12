#include "pwm_out.h"
#include "register.h"

static inline void set_ccr(uint8_t m, uint16_t us)
{
    switch (m) {
    case 0: TIM1->CCR3 = us; break;
    case 1: TIM1->CCR1 = us; break;
    case 2: TIM2->CCR4 = us; break;
    case 3: TIM2->CCR3 = us; break;
    default: break;
    }
}

static void gpio_af(GPIO_TypeDef *port, uint32_t pin, uint32_t af)
{
    port->MODER   &= ~(3U << (pin * 2));
    port->MODER   |=  (2U << (pin * 2));
    port->OSPEEDR |=  (3U << (pin * 2));
    port->OTYPER  &= ~(1U << pin);
    port->PUPDR   &= ~(3U << (pin * 2));
    if (pin < 8U) {
        port->AFR[0] &= ~(0xFU << (pin * 4));
        port->AFR[0] |=  (af   << (pin * 4));
    } else {
        port->AFR[1] &= ~(0xFU << ((pin - 8U) * 4));
        port->AFR[1] |=  (af   << ((pin - 8U) * 4));
    }
}

void Motor_Init(uint32_t tim1_clk_hz, uint32_t tim2_clk_hz)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB2ENR;
    (void)RCC->APB1ENR;

    gpio_af(GPIOB, 15U, 1U);
    gpio_af(GPIOA, 8U,  1U);
    gpio_af(GPIOB, 11U, 1U);
    gpio_af(GPIOB, 10U, 1U);

    uint32_t arr = (1000000U / MOTOR_PWM_HZ) - 1U;

    TIM1->CR1   = 0;
    TIM1->PSC   = (tim1_clk_hz / 1000000U) - 1U;
    TIM1->ARR   = arr;
    TIM1->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
    TIM1->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;
    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC3NE;
    TIM1->BDTR |= TIM_BDTR_MOE;

    TIM2->CR1   = 0;
    TIM2->DIER  = 0;
    TIM2->PSC   = (tim2_clk_hz / 1000000U) - 1U;
    TIM2->ARR   = arr;
    TIM2->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE
                | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;
    TIM2->CCER  = TIM_CCER_CC3E | TIM_CCER_CC4E;

    for (uint8_t m = 0; m < MOTOR_COUNT; m++)
        set_ccr(m, MOTOR_MIN_US);

    TIM1->CR1 |= TIM_CR1_ARPE;  TIM2->CR1 |= TIM_CR1_ARPE;
    TIM1->EGR |= TIM_EGR_UG;    TIM2->EGR |= TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;   TIM2->CR1 |= TIM_CR1_CEN;
}

void Motor_SetUs(uint8_t motor, uint16_t us)
{
    if (motor >= MOTOR_COUNT)
        return;
    if (us < MOTOR_MIN_US) us = MOTOR_MIN_US;
    if (us > MOTOR_MAX_US) us = MOTOR_MAX_US;
    set_ccr(motor, us);
}

void Motor_StopAll(void)
{
    for (uint8_t m = 0; m < MOTOR_COUNT; m++)
        set_ccr(m, MOTOR_MIN_US);
}
