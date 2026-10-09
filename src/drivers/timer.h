#ifndef TIMER_H
#define TIMER_H
#include"../../include/register.h"
#include <stdint.h>
#include <stdbool.h>

/* TIM7 (timer cơ bản, không có chân): ngắt update kích đọc IMU */
void Timer7_InitHz(uint32_t timer_clk_hz, uint32_t freq_hz);
void Timer7_Start(void);
void Timer7_Stop(void);
void TIM7_IRQHandler(void);

/* TIM5 one-shot: đếm thời gian cho các chế độ di chuyển (tiến / sang trái).
 * Không dùng ngắt — poll cờ UIF. Tối đa ~429 s. */
void Timer5_Init(uint32_t timer_clk_hz);
void Timer5_StartOneShot(uint32_t ms);
void Timer5_Stop(void);
bool Timer5_IsExpired(void);    /* true khi đã đếm hết thời gian */

#endif 