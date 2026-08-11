#include "spi.h"
#include "register.h"


/* =========================================================
 * PRIVATE STATE
 * ========================================================= */
extern void CS_Low(void);
extern void CS_High(void);
static volatile SPI1_DMA_Callback dma_callback = 0;

/* =========================================================
 * BLOCKING SPI
 * APB2 clock = 84MHz (168MHz / 2)
 * BR_DIV16 → SPI clock = 84/16 = 5.25MHz (ICM-42605 max 24MHz)
 * ========================================================= */

void SPI1_Init(void)
{
    /* 1. Bật clock cho GPIOA, GPIOB SPI1 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    __asm volatile ("nop");
    __asm volatile ("nop");

    /* 2. Cấu hình PA5 (SCK) - AF5, High speed */
    GPIOA->MODER   &= ~(0x3U << (SPI1_SCK_PIN * 2));
    GPIOA->MODER   |=  (GPIO_MODE_AF << (SPI1_SCK_PIN * 2));
    GPIOA->OSPEEDR |=  (GPIO_SPEED_VERY_HIGH << (SPI1_SCK_PIN * 2));
    GPIOA->PUPDR   &= ~(0x3U << (SPI1_SCK_PIN * 2));
    GPIOA->AFR[0]  &= ~(0xFU << (SPI1_SCK_PIN * 4));
    GPIOA->AFR[0]  |=  (GPIO_AF5_SPI1 << (SPI1_SCK_PIN * 4));

    /* 3. Cấu hình PA7 (MOSI) - AF5, High speed */
    GPIOA->MODER   &= ~(0x3U << (SPI1_MOSI_PIN * 2));
    GPIOA->MODER   |=  (GPIO_MODE_AF << (SPI1_MOSI_PIN * 2));
    GPIOA->OSPEEDR |=  (GPIO_SPEED_VERY_HIGH << (SPI1_MOSI_PIN * 2));
    GPIOA->PUPDR   &= ~(0x3U << (SPI1_MOSI_PIN * 2));
    GPIOA->AFR[0]  &= ~(0xFU << (SPI1_MOSI_PIN * 4));
    GPIOA->AFR[0]  |=  (GPIO_AF5_SPI1 << (SPI1_MOSI_PIN * 4));

    /* 4. Cấu hình PB4 (MISO) - AF5
     * PB4 nằm ở AFR[0] vì pin 4 < 8 */
    GPIOB->MODER   &= ~(0x3U << (SPI1_MISO_PIN * 2));
    GPIOB->MODER   |=  (GPIO_MODE_AF << (SPI1_MISO_PIN * 2));
    GPIOB->OSPEEDR |=  (GPIO_SPEED_VERY_HIGH << (SPI1_MISO_PIN * 2));
    GPIOB->PUPDR   &= ~(0x3U << (SPI1_MISO_PIN * 2));
    GPIOB->AFR[0]  &= ~(0xFU << (SPI1_MISO_PIN * 4));
    GPIOB->AFR[0]  |=  (GPIO_AF5_SPI1 << (SPI1_MISO_PIN * 4));

    

    /* 6. Cấu hình SPI1
     * - Master mode
     * - CPOL=1, CPHA=1 (Mode 3) - ICM-42605 yêu cầu
     * - 8-bit data frame
     * - Software NSS (SSM=1, SSI=1)
     * - BR = fPCLK/16 = 84/16 ≈ 5.25MHz
     */
    SPI1->CR1 = 0; /* Reset trước */
    SPI1->CR1 = SPI_CR1_MSTR
              | SPI_CR1_CPOL       /* CPOL = 1 */
              | SPI_CR1_CPHA       /* CPHA = 1 */
              | SPI_CR1_SSM        /* Software NSS */
              | SPI_CR1_SSI        /* SSI = 1 (NSS internal high) */
              | SPI_CR1_BR_DIV16;  /* 84MHz / 16 = 5.25MHz */

    /* 7. Bật SPI */
    SPI1->CR1 |= SPI_CR1_SPE;
}

uint8_t SPI1_TransmitReceive(uint8_t tx_data)
{
    uint16_t timeout;

    timeout = 10000;
    while (!(SPI1->SR & SPI_SR_TXE)) {
        if (--timeout == 0) return SPI_ERROR;
    }

    *((volatile uint8_t *)&SPI1->DR) = tx_data;

    timeout = 10000;
    while (!(SPI1->SR & SPI_SR_RXNE)) {
        if (--timeout == 0) return SPI_ERROR;
    }

    return (uint8_t)SPI1->DR;
}


/* =========================================================
 * DMA INIT
 * ========================================================= */
void SPI1_DMA_Init(void)
{
    /* Bật clock DMA2 */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    __asm volatile ("nop");
    __asm volatile ("nop");
 
    /* ── DMA2 Stream0: SPI1_RX (Channel 3) ─────────────
     * Peripheral → Memory, byte, memory increment, high priority
     */
    DMA2->S[0].CR = 0;                      /* disable trước khi config */
    while (DMA2->S[0].CR & DMA_CR_EN);      /* chờ disable xong */
 
    DMA2->S[0].CR  = DMA_CR_CHSEL_3_RX     /* channel 3 = SPI1 RX */
                   | DMA_CR_PL_HIGH
                   | DMA_CR_MSIZE_8
                   | DMA_CR_PSIZE_8
                   | DMA_CR_MINC            /* tăng memory address */
                   | DMA_CR_DIR_P2M         /* peripheral → memory */
                   | DMA_CR_TCIE;           /* interrupt khi xong */
    DMA2->S[0].PAR = (uint32_t)&SPI1->DR;  /* nguồn: SPI1 DR */
 
    /* ── DMA2 Stream3: SPI1_TX (Channel 3) ─────────────
     * Memory → Peripheral, byte, memory increment, high priority
     */
    DMA2->S[3].CR = 0;
    while (DMA2->S[3].CR & DMA_CR_EN);
 
    DMA2->S[3].CR  = DMA_CR_CHSEL_3        /* channel 3 = SPI1 TX */
                   | DMA_CR_PL_HIGH
                   | DMA_CR_MSIZE_8
                   | DMA_CR_PSIZE_8
                   | DMA_CR_MINC
                   | DMA_CR_DIR_M2P         /* memory → peripheral */
                   | DMA_CR_TCIE;
    DMA2->S[3].PAR = (uint32_t)&SPI1->DR;
 
    /* Bật NVIC cho DMA2 Stream0 (IRQ56) và Stream3 (IRQ59) */
    NVIC->ISER[1] |= (1U << (NVIC_DMA2_S0_IRQ - 32U)); /* bit 24 */
    NVIC->ISER[1] |= (1U << (NVIC_DMA2_S3_IRQ - 32U)); /* bit 27 */
    NVIC->IP[NVIC_DMA2_S0_IRQ] = (8U << 4);  /* priority cao hơn UART5 */
    NVIC->IP[NVIC_DMA2_S3_IRQ] = (8U << 4);
 
    /* Bật DMA mode cho SPI1 */
    SPI1->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
}
 
/* =========================================================
 * DMA TRANSFER
 * ========================================================= */
void SPI1_DMA_Transfer(const uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, SPI1_DMA_Callback cb)
{
    /* Lưu callback */
    dma_callback = cb;
     
    /* Clear interrupt flags */
    DMA2->LIFCR = DMA2_S0_TCIF | DMA2_S0_TEIF | DMA2_S0_FEIF | DMA2_S0_DMEIF;
    DMA2->LIFCR = DMA2_S3_TCIF | DMA2_S3_TEIF | DMA2_S3_FEIF | DMA2_S3_DMEIF;
 
    /* Config RX stream */
    DMA2->S[0].M0AR = (uint32_t)rx_buf;
    DMA2->S[0].NDTR = len;
 
    /* Config TX stream */
    DMA2->S[3].M0AR = (uint32_t)tx_buf;
    DMA2->S[3].NDTR = len;
    
    CS_Low();
    /* Bật RX trước, TX sau (quan trọng!) */
    DMA2->S[0].CR |= DMA_CR_EN;
    DMA2->S[3].CR |= DMA_CR_EN;
}
 
/* =========================================================
 * IRQ HANDLERS
 * ========================================================= */
 
/* Stream0 RX complete - đây là IRQ chính báo "xong rồi" */
void DMA2_Stream0_IRQHandler(void)
{
    if (DMA2->LISR & DMA2_S0_TCIF) {
        DMA2->LIFCR = DMA2_S0_TCIF;    /* clear flag */
 
        /* Disable cả 2 stream */
        DMA2->S[0].CR &= ~DMA_CR_EN;
        DMA2->S[3].CR &= ~DMA_CR_EN;
        
         CS_High();
         
        /* Gọi callback (thường là ICM42605_DMA_Callback) */
        if (dma_callback) {
            dma_callback();
        }
    }
 
    /* Clear error flags nếu có */
    DMA2->LIFCR = DMA2_S0_TEIF | DMA2_S0_FEIF | DMA2_S0_DMEIF;
}
 
/* Stream3 TX complete - chỉ clear flag, không cần xử lý thêm */
void DMA2_Stream3_IRQHandler(void)
{
    if (DMA2->LISR & DMA2_S3_TCIF) {
        DMA2->LIFCR = DMA2_S3_TCIF;
    }
    DMA2->LIFCR = DMA2_S3_TEIF | DMA2_S3_FEIF | DMA2_S3_DMEIF;
}