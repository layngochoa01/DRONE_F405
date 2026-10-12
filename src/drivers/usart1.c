#include "usart1.h"
#include "register.h"

#define UART1_RX_BUF_SIZE 512U

static volatile char     s_rxBuf[UART1_RX_BUF_SIZE];
static volatile uint16_t s_rxHead = 0;
static volatile uint16_t s_rxTail = 0;
static volatile uint32_t s_rxDropped = 0;

static uint32_t computeBRR(uint32_t pclk, uint32_t baud)
{
    uint32_t usartdiv_x100 = (pclk * 25U) / (4U * baud);
    uint32_t mantissa = usartdiv_x100 / 100U;
    uint32_t fraction = ((usartdiv_x100 - mantissa * 100U) * 16U + 50U) / 100U;
    if (fraction >= 16U) { mantissa += 1U; fraction = 0U; }
    return (mantissa << 4) | (fraction & 0x0FU);
}

void UART1_Init(uint32_t apb2_clk, uint32_t baud)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    GPIOA->MODER &= ~(3U << (10U * 2));
    GPIOA->MODER |=  (GPIO_MODE_AF << (10U * 2));
    GPIOA->AFR[1] &= ~(0xFU << ((10U - 8U) * 4));
    GPIOA->AFR[1] |=  (GPIO_AF7_USART1 << ((10U - 8U) * 4));
    GPIOA->PUPDR  &= ~(3U << (10U * 2));
    GPIOA->PUPDR  |=  (GPIO_PUPD_UP << (10U * 2));

    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;

    USART1->BRR = computeBRR(apb2_clk, baud);
    USART1->CR1 = USART_CR1_RE | USART_CR1_RXNEIE;
    USART1->CR1 |= USART_CR1_UE;

    NVIC->ISER[1] |= (1U << (37 - 32));
}

void UART1_IRQ_Handler(void)
{
    if (!(USART1->SR & USART_SR_RXNE)) return;

    char c = (char)USART1->DR;
    uint16_t next = (uint16_t)((s_rxHead + 1U) % UART1_RX_BUF_SIZE);

    if (next != s_rxTail) {
        s_rxBuf[s_rxHead] = c;
        s_rxHead = next;
    } else {
        s_rxDropped++;
    }
}

bool UART1_RxAvailable(void)
{
    return s_rxHead != s_rxTail;
}

char UART1_RxRead(void)
{
    char c = s_rxBuf[s_rxTail];
    s_rxTail = (uint16_t)((s_rxTail + 1U) % UART1_RX_BUF_SIZE);
    return c;
}

uint32_t UART1_GetDroppedCount(void)
{
    return s_rxDropped;
}
