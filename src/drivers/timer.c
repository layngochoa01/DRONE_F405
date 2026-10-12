#include "timer.h"
#include "register.h"

void Timer7_InitHz(uint32_t timer_clk_hz, uint32_t freq_hz)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    TIM7->CR1 &= ~TIM_CR1_CEN;

    uint32_t tick_hz = 1000000U;
    uint32_t psc = (timer_clk_hz / tick_hz) - 1U;

    uint32_t arr = (tick_hz / freq_hz) - 1U;

    TIM7->PSC = psc;
    TIM7->ARR = arr;
    TIM7->CNT = 0U;

    TIM7->CR1 |= TIM_CR1_URS;

    TIM7->EGR |= TIM_EGR_UG;
    TIM7->SR  &= ~TIM_SR_UIF;

    TIM7->DIER |= TIM_DIER_UIE;

    NVIC->IP[NVIC_TIM7_IRQ] = (10U << 4);
    NVIC->ISER[NVIC_TIM7_IRQ / 32U] = (1U << (NVIC_TIM7_IRQ % 32U));
}

void Timer7_Start(void)
{
    TIM7->CR1 |= TIM_CR1_CEN;
}

void Timer5_Init(uint32_t timer_clk_hz)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    TIM5->CR1  = 0U;
    TIM5->DIER = 0U;
    TIM5->PSC  = (timer_clk_hz / 10000U) - 1U;
    TIM5->ARR  = 0xFFFFFFFFU;
    TIM5->CR1  = TIM_CR1_OPM | TIM_CR1_URS;
    TIM5->EGR  = TIM_EGR_UG;
    TIM5->SR   = 0U;
}

void Timer5_StartOneShot(uint32_t ms)
{
    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->ARR  = ms * 10U - 1U;
    TIM5->CNT  = 0U;
    TIM5->SR   = 0U;
    TIM5->CR1 |= TIM_CR1_CEN;
}

void Timer5_Stop(void)
{
    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->SR   = 0U;
}

bool Timer5_IsExpired(void)
{
    return (TIM5->SR & TIM_SR_UIF) != 0U;
}
