#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdint.h>

/* Định nghĩa trong main.c — SysTick 1 ms */
uint32_t get_tick(void);
void     delay_ms(uint32_t ms);

/* Bộ đếm chu kỳ CPU (DWT) — đo thời gian thực của vòng nhanh */
#define CPU_HZ        168000000U
#define DWT_CTRL      (*(volatile uint32_t *)0xE0001000UL)
#define DWT_CYCCNT    (*(volatile uint32_t *)0xE0001004UL)
#define SCB_DEMCR     (*(volatile uint32_t *)0xE000EDFCUL)

static inline void cycle_counter_init(void)
{
    SCB_DEMCR |= (1UL << 24);   /* TRCENA */
    DWT_CYCCNT = 0U;
    DWT_CTRL  |= 1UL;           /* CYCCNTENA */
}

static inline uint32_t cycle_now(void)
{
    return DWT_CYCCNT;
}

/* Vùng găng: chép dữ liệu dùng chung giữa ngắt và main loop */
#define CRITICAL_BEGIN()  __asm volatile ("cpsid i" ::: "memory")
#define CRITICAL_END()    __asm volatile ("cpsie i" ::: "memory")

#endif
