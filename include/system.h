#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdint.h>

/* Định nghĩa trong main.c — SysTick 1 ms */
uint32_t get_tick(void);
void     delay_ms(uint32_t ms);

#endif
