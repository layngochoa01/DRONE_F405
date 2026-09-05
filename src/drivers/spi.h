#ifndef SPI_H
#define SPI_H

#include <stdint.h>

#define SPI_ERROR 0xFF

typedef void (*SPI1_DMA_Callback)(void);

void SPI1_Init(void);
void    SPI1_DMA_Init(void);

uint8_t SPI1_TransmitReceive(uint8_t tx_data);
void    SPI1_DMA_Transfer(const uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, SPI1_DMA_Callback cb);

void    DMA2_Stream0_IRQHandler(void); 
void    DMA2_Stream3_IRQHandler(void);  
#endif