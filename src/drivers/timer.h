#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

/* =========================================================
 * TIMER (TIM2) - Trigger định kỳ cho IMU DMA read
 *
 * Mục đích: TIM2 chạy ở APB1, tạo interrupt định kỳ (mặc định 100Hz)
 * Trong ISR TIM2, KHÔNG đọc trực tiếp SPI (blocking) mà chỉ
 * khởi động một DMA transfer non-blocking (xem icm42605.c).
 *
 * TIM2 là 32-bit timer (RM0090 Section 13), nằm trên APB1.
 * Lưu ý clock: nếu APB1 prescaler != 1 (ở đây PPRE1_DIV4),
 * clock cấp cho timer APB1 sẽ tự nhân đôi theo RM0090 Section 6.2:
 *   APB1 = 168MHz / 4 = 42MHz
 *   TIM2 clock = APB1 * 2 = 84MHz  (vì PPRE1 != 1)
 * ========================================================= */

/* --- TIM2 base address & register map (RM0090 Section 13.4) --- */
#define TIM2_BASE           0x40000000UL   /* APB1_BASE + 0x0000 */

typedef struct {
    volatile uint32_t CR1;     /* 0x00 - Control register 1 */
    volatile uint32_t CR2;     /* 0x04 - Control register 2 */
    volatile uint32_t SMCR;    /* 0x08 - Slave mode control */
    volatile uint32_t DIER;    /* 0x0C - DMA/Interrupt enable */
    volatile uint32_t SR;      /* 0x10 - Status register */
    volatile uint32_t EGR;     /* 0x14 - Event generation */
    volatile uint32_t CCMR1;   /* 0x18 - Capture/compare mode 1 */
    volatile uint32_t CCMR2;   /* 0x1C - Capture/compare mode 2 */
    volatile uint32_t CCER;    /* 0x20 - Capture/compare enable */
    volatile uint32_t CNT;     /* 0x24 - Counter */
    volatile uint32_t PSC;     /* 0x28 - Prescaler */
    volatile uint32_t ARR;     /* 0x2C - Auto-reload */
} TIM_TypeDef;

#define TIM2    ((TIM_TypeDef *) TIM2_BASE)

/* RCC APB1ENR bit - TIM2 clock enable (RM0090 6.3.13, bit 0) */
#define RCC_APB1ENR_TIM2EN      (1U << 0)

/* TIM CR1 bits (RM0090 13.4.1) */
#define TIM_CR1_CEN             (1U << 0)   /* Counter enable */
#define TIM_CR1_URS             (1U << 2)   /* Update request source */

/* TIM DIER bits (RM0090 13.4.4) */
#define TIM_DIER_UIE            (1U << 0)   /* Update interrupt enable */

/* TIM SR bits (RM0090 13.4.5) */
#define TIM_SR_UIF              (1U << 0)   /* Update interrupt flag */

/* TIM EGR bits (RM0090 13.4.3) */
#define TIM_EGR_UG              (1U << 0)   /* Update generation (force reload) */

/* NVIC IRQ number cho TIM2 (RM0090 Table 61: IRQ28) */
#define NVIC_TIM2_IRQ           28U

/* =========================================================
 * PUBLIC API
 * ========================================================= */

/**
 * Timer2_InitHz - Khởi tạo TIM2 chạy interrupt định kỳ ở freq_hz
 * timer_clk_hz: clock thực sự cấp vào timer (xem ghi chú clock ở trên,
 *               KHÔNG phải APB1 clock thô, mà là APB1 * 2 nếu prescaler != 1)
 * freq_hz     : tần số interrupt mong muốn (ví dụ 100 cho 100Hz)
 *
 * ISR thực tế người dùng tự định nghĩa hàm "TIM2_IRQHandler(void)"
 * (đặt trong icm42605.c hoặc main.c, khai báo trong startup.s)
 */
void Timer2_InitHz(uint32_t timer_clk_hz, uint32_t freq_hz);

/* Bật / tắt timer (không cần re-init lại toàn bộ cấu hình) */
void Timer2_Start(void);
void Timer2_Stop(void);

/* IRQ handler - định nghĩa thực tế nằm ở icm42605.c (trigger DMA read) */
void TIM2_IRQHandler(void);

#endif /* TIMER_H */