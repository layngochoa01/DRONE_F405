#include "timer.h"
#include "register.h"

/* =========================================================
 * Timer2_InitHz
 *
 * Công thức tần số timer (RM0090 Section 13.4):
 *   f_update = timer_clk_hz / ((PSC + 1) * (ARR + 1))
 *
 * Chiến lược chọn PSC/ARR:
 *   - Chọn PSC sao cho (timer_clk_hz / (PSC+1)) là số nguyên "đẹp"
 *     (ví dụ ra đúng 1MHz hoặc 1kHz) để ARR tính ra số nguyên,
 *     giảm sai số làm tròn.
 *   - Ở đây: PSC sao cho tick = 1MHz (1us/tick), sau đó
 *     ARR = (1000000 / freq_hz) - 1
 *
 * Ví dụ: timer_clk_hz = 84,000,000Hz (APB1*2), freq_hz = 100Hz
 *   PSC = (84,000,000 / 1,000,000) - 1 = 83
 *   ARR = (1,000,000 / 100) - 1       = 9999
 *   → f_update = 84,000,000 / (84 * 10000) = 100Hz  (đúng)
 * ========================================================= */
void Timer2_InitHz(uint32_t timer_clk_hz, uint32_t freq_hz)
{
    /* 1. Bật clock cho TIM2 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    /* 2. Disable timer trước khi cấu hình */
    TIM2->CR1 &= ~TIM_CR1_CEN;

    /* 3. Tính PSC để ra tick 1MHz (1us/tick) */
    uint32_t tick_hz = 1000000U;
    uint32_t psc = (timer_clk_hz / tick_hz) - 1U;

    /* 4. Tính ARR để ra đúng freq_hz từ tick 1MHz */
    uint32_t arr = (tick_hz / freq_hz) - 1U;

    TIM2->PSC = psc;
    TIM2->ARR = arr;
    TIM2->CNT = 0U;

    /* 5. URS=1: update event chỉ sinh ra do overflow/UG, không do
     *    ghi nhầm vào các thanh ghi khác → tránh ngắt giả khi cấu hình */
    TIM2->CR1 |= TIM_CR1_URS;

    /* 6. Force load PSC/ARR vào shadow register ngay (EGR.UG)
     *    rồi clear cờ UIF do UG vừa sinh ra, tránh ngắt giả ngay khi bật */
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->SR  &= ~TIM_SR_UIF;

    /* 7. Bật update interrupt */
    TIM2->DIER |= TIM_DIER_UIE;

    /* 8. Bật NVIC cho TIM2 (IRQ28 → ISER[0] bit 28) */
    NVIC->ISER[0] |= (1U << NVIC_TIM2_IRQ);
    NVIC->IP[NVIC_TIM2_IRQ] = (10U << 4);  /* priority thấp hơn DMA (DMA = 12) */
}

void Timer2_Start(void)
{
    TIM2->CR1 |= TIM_CR1_CEN;
}

void Timer2_Stop(void)
{
    TIM2->CR1 &= ~TIM_CR1_CEN;
}