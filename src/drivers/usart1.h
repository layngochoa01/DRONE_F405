#ifndef UART1_H
#define UART1_H

#include <stdint.h>
#include <stdbool.h>

void UART1_Init(uint32_t apb2_clk, uint32_t baud);
void UART1_IRQ_Handler(void);  

bool     UART1_RxAvailable(void);
char     UART1_RxRead(void);
uint32_t UART1_GetDroppedCount(void);
void     UART1_SetBaud(uint32_t apb2_clk, uint32_t baud);   /* byte bị bỏ do bộ đệm nhận đầy */

#endif