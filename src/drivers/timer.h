#ifndef TIMER_H
#define TIMER_H
#include"../../include/register.h"
#include <stdint.h>

void Timer2_InitHz(uint32_t timer_clk_hz, uint32_t freq_hz);
void Timer2_Start(void);
void Timer2_Stop(void);
void TIM2_IRQHandler(void);

#endif 