#include "spi.h"
#include "register.h"

extern void CS_Low(void);
extern void CS_High(void);
static volatile SPI1_DMA_Callback dma_callback = 0;

void SPI1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    __asm volatile ("nop");
    __asm volatile ("nop");

    GPIOA->MODER   &= ~(0x3U << (SPI1_SCK_PIN * 2));
    GPIOA->MODER   |=  (GPIO_MODE_AF << (SPI1_SCK_PIN * 2));
    GPIOA->OSPEEDR |=  (GPIO_SPEED_VERY_HIGH << (SPI1_SCK_PIN * 2));
    GPIOA->PUPDR   &= ~(0x3U << (SPI1_SCK_PIN * 2));
    GPIOA->AFR[0]  &= ~(0xFU << (SPI1_SCK_PIN * 4));
    GPIOA->AFR[0]  |=  (GPIO_AF5_SPI1 << (SPI1_SCK_PIN * 4));

    GPIOA->MODER   &= ~(0x3U << (SPI1_MOSI_PIN * 2));
    GPIOA->MODER   |=  (GPIO_MODE_AF << (SPI1_MOSI_PIN * 2));
    GPIOA->OSPEEDR |=  (GPIO_SPEED_VERY_HIGH << (SPI1_MOSI_PIN * 2));
    GPIOA->PUPDR   &= ~(0x3U << (SPI1_MOSI_PIN * 2));
    GPIOA->AFR[0]  &= ~(0xFU << (SPI1_MOSI_PIN * 4));
    GPIOA->AFR[0]  |=  (GPIO_AF5_SPI1 << (SPI1_MOSI_PIN * 4));

    GPIOB->MODER   &= ~(0x3U << (SPI1_MISO_PIN * 2));
    GPIOB->MODER   |=  (GPIO_MODE_AF << (SPI1_MISO_PIN * 2));
    GPIOB->OSPEEDR |=  (GPIO_SPEED_VERY_HIGH << (SPI1_MISO_PIN * 2));
    GPIOB->PUPDR   &= ~(0x3U << (SPI1_MISO_PIN * 2));
    GPIOB->AFR[0]  &= ~(0xFU << (SPI1_MISO_PIN * 4));
    GPIOB->AFR[0]  |=  (GPIO_AF5_SPI1 << (SPI1_MISO_PIN * 4));

    SPI1->CR1 = 0; 
    SPI1->CR1 = SPI_CR1_MSTR
              | SPI_CR1_CPOL       /* CPOL = 1 */
              | SPI_CR1_CPHA       /* CPHA = 1 */
              | SPI_CR1_SSM        /* Software NSS */
              | SPI_CR1_SSI        /* SSI = 1 (NSS internal high) */
              | SPI_CR1_BR_DIV16;  /* 84MHz / 16 = 5.25MHz */

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

void SPI1_DMA_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    __asm volatile ("nop");
    __asm volatile ("nop");
 
    DMA2->S[0].CR = 0;                     
    while (DMA2->S[0].CR & DMA_CR_EN);     
 
    DMA2->S[0].CR  = DMA_CR_CHSEL_3_RX     
                   | DMA_CR_PL_HIGH
                   | DMA_CR_MSIZE_8
                   | DMA_CR_PSIZE_8
                   | DMA_CR_MINC           
                   | DMA_CR_DIR_P2M        
                   | DMA_CR_TCIE;          
    DMA2->S[0].PAR = (uint32_t)&SPI1->DR;  
 
    DMA2->S[3].CR = 0;
    while (DMA2->S[3].CR & DMA_CR_EN);
 
    DMA2->S[3].CR  = DMA_CR_CHSEL_3        
                   | DMA_CR_PL_HIGH
                   | DMA_CR_MSIZE_8
                   | DMA_CR_PSIZE_8
                   | DMA_CR_MINC
                   | DMA_CR_DIR_M2P         
                   | DMA_CR_TCIE;
    DMA2->S[3].PAR = (uint32_t)&SPI1->DR;
 
   
    NVIC->ISER[1] |= (1U << (NVIC_DMA2_S0_IRQ - 32U)); 
    NVIC->ISER[1] |= (1U << (NVIC_DMA2_S3_IRQ - 32U)); 
    NVIC->IP[NVIC_DMA2_S0_IRQ] = (8U << 4);  
    NVIC->IP[NVIC_DMA2_S3_IRQ] = (8U << 4);
 
    SPI1->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
}
 
void SPI1_DMA_Transfer(const uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, SPI1_DMA_Callback cb)
{
    dma_callback = cb;
     
    DMA2->LIFCR = DMA2_S0_TCIF | DMA2_S0_TEIF | DMA2_S0_FEIF | DMA2_S0_DMEIF;
    DMA2->LIFCR = DMA2_S3_TCIF | DMA2_S3_TEIF | DMA2_S3_FEIF | DMA2_S3_DMEIF;

    DMA2->S[0].M0AR = (uint32_t)rx_buf;
    DMA2->S[0].NDTR = len;
 
    DMA2->S[3].M0AR = (uint32_t)tx_buf;
    DMA2->S[3].NDTR = len;
    
    CS_Low();

    DMA2->S[0].CR |= DMA_CR_EN;
    DMA2->S[3].CR |= DMA_CR_EN;
}
 
void DMA2_Stream0_IRQHandler(void)
{
    if (DMA2->LISR & DMA2_S0_TCIF) {
        DMA2->LIFCR = DMA2_S0_TCIF;    

        DMA2->S[0].CR &= ~DMA_CR_EN;
        DMA2->S[3].CR &= ~DMA_CR_EN;
        
         CS_High();
         
        if (dma_callback) {
            dma_callback();
        }
    }
 
    DMA2->LIFCR = DMA2_S0_TEIF | DMA2_S0_FEIF | DMA2_S0_DMEIF;
}
 
void DMA2_Stream3_IRQHandler(void)
{
    if (DMA2->LISR & DMA2_S3_TCIF) {
        DMA2->LIFCR = DMA2_S3_TCIF;
    }
    DMA2->LIFCR = DMA2_S3_TEIF | DMA2_S3_FEIF | DMA2_S3_DMEIF;
}