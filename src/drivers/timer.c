#include "timer.h"
#include "register.h"

/* =========================================================
 * Timer7_InitHz (kích đọc IMU 100 Hz — TIM2 nhường cho PWM5/6)
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
void Timer7_InitHz(uint32_t timer_clk_hz, uint32_t freq_hz)
{
    /* 1. Bật clock cho TIM7 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    /* 2. Disable timer trước khi cấu hình */
    TIM7->CR1 &= ~TIM_CR1_CEN;

    /* 3. Tính PSC để ra tick 1MHz (1us/tick) */
    uint32_t tick_hz = 1000000U;
    uint32_t psc = (timer_clk_hz / tick_hz) - 1U;

    /* 4. Tính ARR để ra đúng freq_hz từ tick 1MHz */
    uint32_t arr = (tick_hz / freq_hz) - 1U;

    TIM7->PSC = psc;
    TIM7->ARR = arr;
    TIM7->CNT = 0U;

    /* 5. URS=1: update event chỉ sinh ra do overflow/UG, không do
     *    ghi nhầm vào các thanh ghi khác → tránh ngắt giả khi cấu hình */
    TIM7->CR1 |= TIM_CR1_URS;

    /* 6. Force load PSC/ARR vào shadow register ngay (EGR.UG)
     *    rồi clear cờ UIF do UG vừa sinh ra, tránh ngắt giả ngay khi bật */
    TIM7->EGR |= TIM_EGR_UG;
    TIM7->SR  &= ~TIM_SR_UIF;

    /* 7. Bật update interrupt */
    TIM7->DIER |= TIM_DIER_UIE;

    /* 8. Bật NVIC cho TIM7 (IRQ55 → ISER[1] bit 23) */
    NVIC->IP[NVIC_TIM7_IRQ] = (10U << 4);  /* thấp hơn DMA SPI (8), cao hơn UART5 (13) */
    NVIC->ISER[NVIC_TIM7_IRQ / 32U] = (1U << (NVIC_TIM7_IRQ % 32U));
}

void Timer7_Start(void)
{
    TIM7->CR1 |= TIM_CR1_CEN;
}

void Timer7_Stop(void)
{
    TIM7->CR1 &= ~TIM_CR1_CEN;
}

/* =========================================================
 * TIM5 — one-shot timer cho chế độ MOVE_FORWARD / MOVE_LEFT
 *   tick = 10 kHz (0.1 ms), OPM: tràn 1 lần thì tự tắt CEN
 * ========================================================= */
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
    TIM5->EGR  = TIM_EGR_UG;          /* nạp PSC vào shadow */
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
