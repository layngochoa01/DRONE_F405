#include "ibus.h"
#include "register.h"

#define IBUS_TIMEOUT_MS 500U

#define IBUS_FRAME_LEN     32
#define IBUS_HEADER_LEN    0x20
#define IBUS_HEADER_CMD    0x40

static volatile uint8_t  s_frameBuf[IBUS_FRAME_LEN];
static volatile uint8_t  s_frameIndex = 0;
static volatile uint16_t s_channels[IBUS_CHANNEL_COUNT];
static volatile bool     s_frameReady = false;
static volatile bool     s_hasValidFrame = false;
static volatile uint32_t s_lastFrameTick = 0;

volatile uint8_t  g_dbgBuf[64];
volatile uint8_t  g_dbgIdx = 0;
volatile uint8_t  g_dbgDone = 0;
volatile uint32_t g_errORE = 0, g_errFE = 0, g_errNE = 0;
volatile uint32_t g_hdrCount = 0;   /* số lần thấy 0x20 rồi 0x40 */
static uint8_t s_prev = 0;
static uint32_t computeBRR(uint32_t pclk, uint32_t baud)
{
    uint32_t usartdiv_x100 = (pclk * 25U) / (4U * baud);   /* USARTDIV × 100, tránh mất phần thập phân */
    uint32_t mantissa = usartdiv_x100 / 100U;
    uint32_t fraction = ((usartdiv_x100 - mantissa * 100U) * 16U + 50U) / 100U;

    if (fraction >= 16U) {
        mantissa += 1U;
        fraction = 0U;
    }

    return (mantissa << 4) | (fraction & 0x0FU);
}

void IBUS_Init(uint32_t apb1_clk)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER &= ~(3U << (USART2_RX_PIN * 2));
    GPIOA->MODER |=  (GPIO_MODE_AF << (USART2_RX_PIN * 2));

    GPIOA->AFR[0] &= ~(0xFU << (USART2_RX_PIN * 4));
    GPIOA->AFR[0] |=  (GPIO_AF7_USART2 << (USART2_RX_PIN * 4));

    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    USART2->BRR = computeBRR(apb1_clk, 115200U);

    USART2->CR1 = USART_CR1_RE | USART_CR1_RXNEIE;  
    USART2->CR1 |= USART_CR1_UE;

    NVIC->ISER[1] |= (1U << (38 - 32));   
}

static uint16_t decodeChannel(uint8_t lo, uint8_t hi)
{
    return ((uint16_t)hi << 8) | lo;
}

void IBUS_ISR_Handler(void)
{
    uint32_t sr = USART2->SR;

    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        if (sr & USART_SR_ORE) g_errORE++;
        if (sr & USART_SR_FE)  g_errFE++;
        if (sr & USART_SR_NE)  g_errNE++;
    }
    if (!(sr & USART_SR_RXNE) && !(sr & USART_SR_ORE)) return;

    uint8_t c = (uint8_t)USART2->DR;   /* đọc DR sẽ xóa cả ORE */

    if (s_prev == 0x20 && c == 0x40) g_hdrCount++;
    s_prev = c;

    if (!g_dbgDone) {
        g_dbgBuf[g_dbgIdx++] = c;
        if (g_dbgIdx >= 64) g_dbgDone = 1;
    }

    if (s_frameIndex == 0)
    {
        if (c != IBUS_HEADER_LEN)
            return;

        s_frameBuf[0] = c;
        s_frameIndex = 1;
        return;
    }
    
    s_frameBuf[s_frameIndex++] = c;
    
    if (s_frameIndex >= IBUS_FRAME_LEN)
    {
        s_frameIndex = 0;

        if (s_frameBuf[1] != IBUS_HEADER_CMD)
            return;

        uint16_t checksum = 0xFFFF;

        for (int i = 0; i < IBUS_FRAME_LEN - 2; i++)
            checksum -= s_frameBuf[i];

        uint16_t recvChecksum =
            decodeChannel(s_frameBuf[30], s_frameBuf[31]);

        if (checksum != recvChecksum)
            return;

        for (int ch = 0; ch < IBUS_CHANNEL_COUNT; ch++)
        {
            s_channels[ch] =
                decodeChannel(
                    s_frameBuf[2 + ch * 2],
                    s_frameBuf[3 + ch * 2]
                );
        }

        s_lastFrameTick = get_tick();
        s_hasValidFrame = true;
        s_frameReady = true;
    }
}

bool IBUS_IsFrameReady(void)
{
    if (s_frameReady) {
        s_frameReady = false;
        return true;
    }
    return false;
}

bool IBUS_IsConnected(void)
{
    if (!s_hasValidFrame)
        return false;

    return ((get_tick() - s_lastFrameTick) < IBUS_TIMEOUT_MS);
}

uint16_t IBUS_GetChannel(uint8_t index)
{
    if (index >= IBUS_CHANNEL_COUNT) return 1500;  
    return s_channels[index];
}