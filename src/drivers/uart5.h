#ifndef UART5_H
#define UART5_H

#include <stdint.h>
#include <stddef.h>

#define UART5_RX_BUF_SIZE 256U

/* Khởi tạo UART5
 * apb1_clk: Tần số xung nhịp hiện tại của bus APB1 (Hz)
 * baudrate: Tốc độ baud 115200
 */
void UART5_Init(uint32_t apb1_clk, uint32_t baudrate);


/* ── TX  ──────────────────────── */
/* Gửi 1 byte (ký tự) */
void UART5_WriteChar(char c);

/* Gửi một chuỗi ký tự (string) */
void UART5_WriteString(const char* str);

void UART5_WriteF(const char *fmt, ...);

/* ── RX - Ring buffer ────────────────────────────── */
uint16_t UART5_RxAvailable(void);
char     UART5_RxRead(void);
uint16_t UART5_RxReadLine(char *buf, uint16_t max_len);  /* đọc đến '\n' */
void     UART5_RxFlush(void);

/* ── IRQ (gọi trong vector table) ───────────────── */
void     UART5_IRQHandler(void);

#endif /* UART5_H */