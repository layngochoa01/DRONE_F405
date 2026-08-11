#ifndef SPI_H
#define SPI_H

#include <stdint.h>
/* =========================================================

 * Giao tiếp: SPI1 (Mode 3: CPOL=1, CPHA=1)
 * CS: PC14
 * SCK: PA5 | MOSI: PA7 | MISO: PB4
 * ========================================================= */

#define SPI_ERROR 0xFF

typedef void (*SPI1_DMA_Callback)(void);

/* Khởi tạo SPI1 (Chân PA5, PB4, PA7) */
void SPI1_Init(void);

/* Truyền và nhận 1 byte đồng thời qua SPI1 (có cơ chế chống treo) */
uint8_t SPI1_TransmitReceive(uint8_t tx_data);


/* ── Non-blocking (DMA) ──────────────────────────────── */
void    SPI1_DMA_Init(void);
 
/* Bắt đầu transfer DMA:
 * tx_buf : buffer gửi đi (15 bytes: 1 addr + 14 data)
 * rx_buf : buffer nhận về
 * len    : số bytes
 * cb     : callback gọi khi xong (từ IRQ)
 */
void    SPI1_DMA_Transfer(const uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, SPI1_DMA_Callback cb);
 
/* IRQ handlers - đặt trong startup.s */
void    DMA2_Stream0_IRQHandler(void);  /* RX complete */
void    DMA2_Stream3_IRQHandler(void);  /* TX complete */
 
#endif