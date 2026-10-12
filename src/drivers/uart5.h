#ifndef UART5_H
#define UART5_H

#include <stdint.h>
#include <stddef.h>

#define UART5_RX_BUF_SIZE 256U

void UART5_Init(uint32_t apb1_clk, uint32_t baudrate);

void UART5_WriteChar(char c);

void UART5_WriteString(const char* str);

void UART5_WriteF(const char *fmt, ...);

uint16_t UART5_RxAvailable(void);
char     UART5_RxRead(void);
void     UART5_RxFlush(void);

void     UART5_IRQHandler(void);

#endif
