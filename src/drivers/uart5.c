#include "uart5.h"
#include "register.h"
#include <stdarg.h>
#include<stdio.h>

/* =========================================================
 * RING BUFFER
 * ========================================================= */
#define RB_MASK     (UART5_RX_BUF_SIZE - 1U)

static volatile uint8_t  rb_buf[UART5_RX_BUF_SIZE];
static volatile uint16_t rb_head = 0;   /* IRQ ghi */
static volatile uint16_t rb_tail = 0;   /* app đọc */

static inline void rb_push(uint8_t byte)
{
    uint16_t next = (rb_head + 1U) & RB_MASK;
    if (next != rb_tail) {          /* bỏ qua khi full */
        rb_buf[rb_head] = byte;
        rb_head = next;
    }
}

/* =========================================================
 * INIT
 * ========================================================= */
void UART5_Init(uint32_t apb1_clk, uint32_t baudrate)
{
    /* 1. Bật clock GPIOC, GPIOD, UART5 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    /* 2. PC12 → AF8 (TX) */
    GPIOC->MODER  &= ~(0x3U << (12 * 2));
    GPIOC->MODER  |=  (GPIO_MODE_AF << (12 * 2));
    GPIOC->OTYPER &= ~(1U << 12);
    GPIOC->OSPEEDR|=  (GPIO_SPEED_VERY_HIGH << (12 * 2));
    GPIOC->PUPDR  &= ~(0x3U << (12 * 2));
    GPIOC->AFR[1] &= ~(0xFU << ((12 - 8) * 4));
    GPIOC->AFR[1] |=  (8U   << ((12 - 8) * 4));    /* AF8 */

    /* 3. PD2 → AF8 (RX), pull-up */
    GPIOD->MODER  &= ~(0x3U << (2 * 2));
    GPIOD->MODER  |=  (GPIO_MODE_AF << (2 * 2));
    GPIOD->PUPDR  &= ~(0x3U << (2 * 2));
    GPIOD->PUPDR  |=  (GPIO_PUPD_UP << (2 * 2));
    GPIOD->AFR[0] &= ~(0xFU << (2 * 4));
    GPIOD->AFR[0] |=  (8U   << (2 * 4));           /* AF8 */

    /* 4. Cấu hình UART5 */
    UART5->CR1 = 0;
    UART5->CR2 = 0;
    UART5->CR3 = 0;
    UART5->BRR = (apb1_clk + baudrate / 2U) / baudrate;  /* làm tròn */

    UART5->CR1 |= USART_CR1_RXNEIE     /* bật RX interrupt */
               |  USART_CR1_TE
               |  USART_CR1_RE
               |  USART_CR1_UE;

    /* 5. NVIC: IRQ53, priority 13 */
    NVIC->ISER[1] |= (1U << (53U - 32U));
    NVIC->IP[53]   =  (13U << 4);
}

/* =========================================================
 * TX
 * ========================================================= */
void UART5_WriteChar(char c)
{
    while (!(UART5->SR & USART_SR_TXE));
    UART5->DR = (uint8_t)c;
}

void UART5_WriteString(const char *str)
{
    while (*str) UART5_WriteChar(*str++);
}

void UART5_WriteF(const char *fmt, ...)
{
    char tmp[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);
    UART5_WriteString(tmp);
}

/* =========================================================
 * RX
 * ========================================================= */
uint16_t UART5_RxAvailable(void)
{
    return (rb_head - rb_tail) & RB_MASK;
}

char UART5_RxRead(void)
{
    char c = (char)rb_buf[rb_tail];
    rb_tail = (rb_tail + 1U) & RB_MASK;
    return c;
}

uint16_t UART5_RxReadLine(char *buf, uint16_t max_len)
{
    uint16_t count = 0;
    while (UART5_RxAvailable() && count < (max_len - 1U)) {
        char c = UART5_RxRead();
        if (c == '\r') continue;
        buf[count++] = c;
        if (c == '\n') break;
    }
    buf[count] = '\0';
    return count;
}

void UART5_RxFlush(void)
{
    rb_head = rb_tail = 0;
}

/* =========================================================
 * IRQ HANDLER
 * ========================================================= */
void UART5_IRQHandler(void)
{
    uint32_t sr = UART5->SR;

    if (sr & USART_SR_RXNE) {
        rb_push((uint8_t)(UART5->DR & 0xFFU));
    }

    /* Clear error flags: đọc DR để xóa ORE/FE/NE */
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        (void)UART5->DR;
    }
}