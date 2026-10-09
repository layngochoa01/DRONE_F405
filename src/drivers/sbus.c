#include "sbus.h"
#include "register.h"
#include "system.h"

#define SBUS_FRAME_LEN      25
#define SBUS_START_BYTE     0x0F
#define SBUS_FOOTER_BYTE    0x00

#define SBUS_FLAG_FRAME_LOST     (1U << 2)
#define SBUS_FLAG_FAILSAFE       (1U << 3)

#define SBUS_TIMEOUT_MS     500U
#define SBUS_BAUD  100000U

#define SBUS_GAP_MS 2U

static volatile uint8_t  s_frameBuf[SBUS_FRAME_LEN];
static volatile uint8_t  s_frameIndex = 0;

static volatile uint16_t s_channelsRaw[SBUS_CHANNEL_COUNT];

static volatile bool     s_frameReady = false;
static volatile bool     s_hasValidFrame = false;
static volatile bool     s_failsafe = false;
static volatile bool     s_frameLost = false;

static volatile uint32_t s_lastFrameTick = 0;
static volatile uint32_t s_lastByteTick = 0;

static bool s_frameBad = false;

// volatile uint8_t  g_sbusLastRaw = 0;
// volatile uint32_t g_sbusRawCount = 0;

// volatile uint32_t g_sbusBytes     = 0;
// volatile uint32_t g_sbusFrames    = 0;
// volatile uint32_t g_sbusGood      = 0;
// volatile uint32_t g_sbusBadFooter = 0;
// volatile uint32_t g_sbusBadUart   = 0;

// volatile uint32_t g_sbusErrFE     = 0;
// volatile uint32_t g_sbusErrNE     = 0;
// volatile uint32_t g_sbusErrPE     = 0;
// volatile uint32_t g_sbusErrORE    = 0;

static uint32_t computeBRR(uint32_t pclk, uint32_t baud)
{
    uint32_t usartdiv_x100 = (pclk * 25U) / (4U * baud);
    uint32_t mantissa = usartdiv_x100 / 100U;
    uint32_t fraction = ((usartdiv_x100 - mantissa * 100U) * 16U + 50U) / 100U;

    if (fraction >= 16U) {
        mantissa += 1U;
        fraction = 0U;
    }
    return (mantissa << 4) | (fraction & 0x0FU);
}

static void sbus_decode(const uint8_t *b)
{
    uint32_t acc = 0;
    uint32_t bits = 0;
    uint32_t idx = 1;

    for (uint32_t ch = 0; ch < SBUS_CHANNEL_COUNT; ch++)
    {
        while (bits < 11U)
        {
            acc |= ((uint32_t)b[idx++]) << bits;
            bits += 8U;
        }

        s_channelsRaw[ch] =
            (uint16_t)(acc & 0x07FFU);

        acc >>= 11;
        bits -= 11U;
    }
}

void SBUS_Init(uint32_t apb1_clk)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER &= ~(3U << (3U * 2));
    GPIOA->MODER |=  (GPIO_MODE_AF << (3U * 2));

    GPIOA->PUPDR &= ~(3U << (3U * 2));
    GPIOA->PUPDR |=  (1U << (3U * 2));

    GPIOA->AFR[0] &= ~(0xFU << (3U * 4));
    GPIOA->AFR[0] |=  (GPIO_AF7_USART2 << (3U * 4));

    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    USART2->BRR = computeBRR(apb1_clk, SBUS_BAUD);
    USART2->CR2 = USART_CR2_STOP_2BIT;
    USART2->CR1 = USART_CR1_M | USART_CR1_PCE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART2->CR1 |= USART_CR1_UE;

    NVIC->ISER[1] |= (1U << (38 - 32));

    s_frameIndex = 0;
    s_frameBad = false;
    s_frameReady = false;
    s_hasValidFrame = false;
}

void SBUS_ISR_Handler(void)
{
    if (!(USART2->SR & USART_SR_RXNE))
        return;

    uint8_t c = (uint8_t)USART2->DR;

    // g_sbusRawCount++;
    // g_sbusLastRaw = c;

    if (s_frameIndex == 0)
    {
        if (c != SBUS_START_BYTE)
            return;
    }

    s_frameBuf[s_frameIndex++] = c;

    if (s_frameIndex >= SBUS_FRAME_LEN)
    {
        s_frameIndex = 0;

        if (s_frameBuf[24] != SBUS_FOOTER_BYTE)
            return;

        uint8_t *d = (uint8_t *)&s_frameBuf[1];

        s_channelsRaw[0]  = (d[0] | d[1] << 8) & 0x07FF;
        s_channelsRaw[1]  = (d[1] >> 3 | d[2] << 5) & 0x07FF;
        s_channelsRaw[2]  = (d[2] >> 6 | d[3] << 2 | d[4] << 10) & 0x07FF;
        s_channelsRaw[3]  = (d[4] >> 1 | d[5] << 7) & 0x07FF;

        s_channelsRaw[4]  = (d[5] >> 4 | d[6] << 4) & 0x07FF;
        s_channelsRaw[5]  = (d[6] >> 7 | d[7] << 1 | d[8] << 9) & 0x07FF;
        s_channelsRaw[6]  = (d[8] >> 2 | d[9] << 6) & 0x07FF;
        s_channelsRaw[7]  = (d[9] >> 5 | d[10] << 3) & 0x07FF;

        s_channelsRaw[8]  = (d[11] | d[12] << 8) & 0x07FF;
        s_channelsRaw[9]  = (d[12] >> 3 | d[13] << 5) & 0x07FF;
        s_channelsRaw[10] = (d[13] >> 6 | d[14] << 2 | d[15] << 10) & 0x07FF;
        s_channelsRaw[11] = (d[15] >> 1 | d[16] << 7) & 0x07FF;

        s_channelsRaw[12] = (d[16] >> 4 | d[17] << 4) & 0x07FF;
        s_channelsRaw[13] = (d[17] >> 7 | d[18] << 1 | d[19] << 9) & 0x07FF;
        s_channelsRaw[14] = (d[19] >> 2 | d[20] << 6) & 0x07FF;
        s_channelsRaw[15] = (d[20] >> 5 | d[21] << 3) & 0x07FF;

        uint8_t flags = s_frameBuf[23];

        s_failsafe = (flags & SBUS_FLAG_FAILSAFE) != 0;

        s_lastFrameTick = get_tick();
        s_hasValidFrame = true;
        s_frameReady = true;

        // g_sbusFrames++;
        // g_sbusGood++;
    }
}

bool SBUS_IsFrameReady(void)
{
    if (s_frameReady) {
        s_frameReady = false;
        return true;
    }
    return false;
}

bool SBUS_IsConnected(void)
{
    if (!s_hasValidFrame) return false;
    return ((get_tick() - s_lastFrameTick) < SBUS_TIMEOUT_MS);
}

bool SBUS_IsFailsafe(void)
{
    return s_failsafe;
}

uint16_t SBUS_GetChannel(uint8_t index)
{
    if (index >= SBUS_CHANNEL_COUNT) return 1500;

    float us = 1000.0f + ((float)s_channelsRaw[index] - 172.0f) * (1000.0f / 1639.0f);
    if (us < 1000.0f) us = 1000.0f;
    if (us > 2000.0f) us = 2000.0f;
    return (uint16_t)us;
}