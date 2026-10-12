#ifndef TIMER_H
#define TIMER_H
#include"../../include/register.h"
#include <stdint.h>
#include <stdbool.h>

void Timer7_InitHz(uint32_t timer_clk_hz, uint32_t freq_hz);
void Timer7_Start(void);
void TIM7_IRQHandler(void);

void Timer5_Init(uint32_t timer_clk_hz);
void Timer5_StartOneShot(uint32_t ms);
void Timer5_Stop(void);
bool Timer5_IsExpired(void);

#endif
