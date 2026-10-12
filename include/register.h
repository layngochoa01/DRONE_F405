#pragma once

#include <stdint.h>

#define REG32(addr)   (*(volatile uint32_t *)(addr))
#define REG16(addr)   (*(volatile uint16_t *)(addr))
#define REG8(addr)    (*(volatile uint8_t  *)(addr))

#define PERIPH_BASE         0x40000000UL
#define AHB1_BASE           (PERIPH_BASE + 0x00020000UL)
#define APB2_BASE           (PERIPH_BASE + 0x00010000UL)
#define APB1_BASE           (PERIPH_BASE + 0x00000000UL)

#define GPIOA_BASE          (AHB1_BASE + 0x0000UL)
#define GPIOB_BASE          (AHB1_BASE + 0x0400UL)
#define GPIOC_BASE          (AHB1_BASE + 0x0800UL)
#define GPIOD_BASE          (AHB1_BASE + 0x0C00UL)

#define RCC_BASE            (AHB1_BASE + 0x3800UL)
#define SPI1_BASE           (APB2_BASE + 0x3000UL)
#define I2C1_BASE           (APB1_BASE + 0x5400UL)

#define USART1_BASE         (APB2_BASE + 0x1000UL)
#define USART2_BASE         (APB1_BASE + 0x4400UL)
#define UART5_BASE          (APB1_BASE + 0x5000UL)

#define DMA2_BASE           (AHB1_BASE + 0x6400UL)

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    volatile uint32_t RESERVED0;
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t RESERVED1;
    volatile uint32_t RESERVED2;
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    volatile uint32_t RESERVED3;
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t RESERVED4;
    volatile uint32_t RESERVED5;
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
    volatile uint32_t AHB3LPENR;
    volatile uint32_t RESERVED6;
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    volatile uint32_t RESERVED7;
    volatile uint32_t RESERVED8;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    volatile uint32_t RESERVED9;
    volatile uint32_t RESERVED10;
    volatile uint32_t SSCGR;
    volatile uint32_t PLLI2SCFGR;
} RCC_TypeDef;

#define RCC     ((RCC_TypeDef *) RCC_BASE)

#define RCC_AHB1ENR_GPIOAEN     (1U << 0)
#define RCC_AHB1ENR_GPIOBEN     (1U << 1)
#define RCC_AHB1ENR_GPIOCEN     (1U << 2)
#define RCC_AHB1ENR_GPIODEN     (1U << 3)

#define RCC_APB2ENR_SPI1EN      (1U << 12)
#define RCC_APB1ENR_I2C1EN      (1U << 21)
#define RCC_CR_HSION            (1U << 0)
#define RCC_CR_HSIRDY           (1U << 1)
#define RCC_CR_HSEON            (1U << 16)
#define RCC_CR_HSERDY           (1U << 17)
#define RCC_CR_PLLON            (1U << 24)
#define RCC_CR_PLLRDY           (1U << 25)

#define RCC_CFGR_SW_PLL         (0x2U << 0)
#define RCC_CFGR_SWS_PLL        (0x2U << 2)
#define RCC_CFGR_HPRE_DIV1      (0x0U << 4)
#define RCC_CFGR_PPRE1_DIV4     (0x5U << 10)
#define RCC_CFGR_PPRE2_DIV2     (0x4U << 13)

#define RCC_PLLCFGR_PLLM        (16U << 0)
#define RCC_PLLCFGR_PLLN        (336U << 6)
#define RCC_PLLCFGR_PLLP_DIV2  (0x0U << 16)
#define RCC_PLLCFGR_PLLSRC_HSI (0U << 22)
#define RCC_PLLCFGR_PLLQ       (7U << 24)

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIO_TypeDef;

#define GPIOA   ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB   ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC   ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD   ((GPIO_TypeDef *) GPIOD_BASE)

#define GPIO_MODE_INPUT     0x0U
#define GPIO_MODE_OUTPUT    0x1U
#define GPIO_MODE_AF        0x2U
#define GPIO_MODE_ANALOG    0x3U

#define GPIO_SPEED_LOW      0x0U
#define GPIO_SPEED_MEDIUM   0x1U
#define GPIO_SPEED_HIGH     0x2U
#define GPIO_SPEED_VERY_HIGH 0x3U

#define GPIO_PUPD_NONE      0x0U
#define GPIO_PUPD_UP        0x1U
#define GPIO_PUPD_DOWN      0x2U

#define GPIO_BSRR_SET(pin)      (1U << (pin))
#define GPIO_BSRR_RESET(pin)    (1U << ((pin) + 16))

#define GPIO_AF5_SPI1   5U
#define GPIO_AF4_I2C1   4U

#define RCC_APB2ENR_USART1EN    (1U << 4)
#define RCC_APB1ENR_UART5EN     (1U << 20)
#define RCC_APB1ENR_USART2EN   (1U << 17)

#define GPIO_AF8_UART5      8U
#define GPIO_AF7_USART2   7U
#define GPIO_AF7_USART1   7U

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;
    volatile uint32_t I2SCFGR;
    volatile uint32_t I2SPR;
} SPI_TypeDef;

#define SPI1    ((SPI_TypeDef *) SPI1_BASE)

#define SPI_CR1_CPHA        (1U << 0)
#define SPI_CR1_CPOL        (1U << 1)
#define SPI_CR1_MSTR        (1U << 2)
#define SPI_CR1_BR_DIV2     (0x0U << 3)
#define SPI_CR1_BR_DIV4     (0x1U << 3)
#define SPI_CR1_BR_DIV8     (0x2U << 3)
#define SPI_CR1_BR_DIV16    (0x3U << 3)
#define SPI_CR1_BR_DIV32    (0x4U << 3)
#define SPI_CR1_BR_DIV64    (0x5U << 3)
#define SPI_CR1_BR_DIV128   (0x6U << 3)
#define SPI_CR1_BR_DIV256   (0x7U << 3)
#define SPI_CR1_SPE         (1U << 6)
#define SPI_CR1_SSI         (1U << 8)
#define SPI_CR1_SSM         (1U << 9)
#define SPI_CR1_DFF         (1U << 11)

#define SPI_SR_RXNE         (1U << 0)
#define SPI_SR_TXE          (1U << 1)
#define SPI_SR_BSY          (1U << 7)

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
    volatile uint32_t FLTR;
} I2C_TypeDef;

#define I2C1    ((I2C_TypeDef *) I2C1_BASE)
#define I2C_CR1_PE          (1U << 0)
#define I2C_CR1_START       (1U << 8)
#define I2C_CR1_STOP        (1U << 9)
#define I2C_CR1_ACK         (1U << 10)
#define I2C_CR1_SWRST       (1U << 15)

#define I2C_SR1_SB          (1U << 0)
#define I2C_SR1_ADDR        (1U << 1)
#define I2C_SR1_BTF         (1U << 2)
#define I2C_SR1_RXNE        (1U << 6)
#define I2C_SR1_TXE         (1U << 7)
#define I2C_SR1_AF          (1U << 10)

#define I2C1_SCL_PIN        8U
#define I2C1_SDA_PIN        7U

typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_TypeDef;

#define UART5   ((USART_TypeDef *) UART5_BASE)
#define USART2   ((USART_TypeDef *) USART2_BASE)
#define USART1   ((USART_TypeDef *) USART1_BASE)

#define USART_CR1_RE        (1U << 2)
#define USART_CR1_TE        (1U << 3)
#define USART_CR1_RXNEIE    (1U << 5)
#define USART_CR1_TXEIE     (1U << 7) ontrol
#define USART_CR1_UE        (1U << 13)

#define USART_SR_PE         (1U << 0)
#define USART_SR_FE         (1U << 1)
#define USART_SR_NE         (1U << 2)
#define USART_SR_ORE        (1U << 3)
#define USART_SR_TC         (1U << 6)
#define USART_SR_RXNE       (1U << 5)
#define USART_SR_TXE        (1U << 7)

#define USART_CR1_PS         (1U << 9)
#define USART_CR1_PCE        (1U << 10)
#define USART_CR1_M          (1U << 12)

#define USART_CR2_STOP_2BIT  (2U << 12)

#define USART2_TX_PIN   2U
#define USART2_RX_PIN   3U

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t NDTR;
    volatile uint32_t PAR;
    volatile uint32_t M0AR;
    volatile uint32_t M1AR;
    volatile uint32_t FCR;
} DMA_Stream_TypeDef;

typedef struct {
    volatile uint32_t LISR;
    volatile uint32_t HISR;
    volatile uint32_t LIFCR;
    volatile uint32_t HIFCR;
    DMA_Stream_TypeDef S[8];
} DMA_TypeDef;

#define DMA2    ((DMA_TypeDef *) DMA2_BASE)

#define DMA_CR_EN              (1U << 0)
#define DMA_CR_DMEIE           (1U << 1)
#define DMA_CR_TEIE            (1U << 2)
#define DMA_CR_HTIE            (1U << 3)
#define DMA_CR_TCIE            (1U << 4)
#define DMA_CR_PFCTRL          (1U << 5)

#define DMA_CR_DIR_P2M         (0x0U << 6)
#define DMA_CR_DIR_M2P         (0x1U << 6)

#define DMA_CR_CIRC            (1U << 8)
#define DMA_CR_PINC            (1U << 9)
#define DMA_CR_MINC            (1U << 10)

#define DMA_CR_PSIZE_8      (0x0U << 11)
#define DMA_CR_MSIZE_8      (0x0U << 13)

#define DMA_CR_PL_LOW       (0x0U << 16)
#define DMA_CR_PL_MEDIUM    (0x1U << 16)
#define DMA_CR_PL_HIGH      (0x2U << 16)
#define DMA_CR_PL_VHIGH     (0x3U << 16)

#define DMA_CR_CHSEL_3      (0x3U << 25)
#define DMA_CR_CHSEL_3_RX   (0x3U << 25)

#define DMA2_S0_TCIF        (1U << 5)
#define DMA2_S0_HTIF        (1U << 4)
#define DMA2_S0_TEIF        (1U << 3)
#define DMA2_S0_DMEIF       (1U << 2)
#define DMA2_S0_FEIF        (1U << 0)

#define DMA2_S3_TCIF        (1U << 27)
#define DMA2_S3_HTIF        (1U << 26)
#define DMA2_S3_TEIF        (1U << 25)
#define DMA2_S3_DMEIF       (1U << 24)
#define DMA2_S3_FEIF        (1U << 22)

#define RCC_AHB1ENR_DMA2EN  (1U << 22)

#define SPI_CR2_RXDMAEN     (1U << 0)
#define SPI_CR2_TXDMAEN     (1U << 1)

#define NVIC_DMA2_S0_IRQ    56U
#define NVIC_DMA2_S3_IRQ    59U

#define SCB_BASE            0xE000ED00UL

typedef struct {
    volatile uint32_t CPUID;
    volatile uint32_t ICSR;
    volatile uint32_t VTOR;
    volatile uint32_t AIRCR;
    volatile uint32_t SCR;
    volatile uint32_t CCR;
    volatile uint8_t  SHP[12];
    volatile uint32_t SHCSR;
    volatile uint32_t CFSR;
    volatile uint32_t HFSR;
    volatile uint32_t RESERVED;
    volatile uint32_t MMFAR;
    volatile uint32_t BFAR;
    volatile uint32_t AFSR;
    volatile uint32_t RESERVED2[19];
    volatile uint32_t CPACR;
} SCB_TypeDef;

#define SCB     ((SCB_TypeDef *) SCB_BASE)

#define FLASH_BASE          0x40023C00UL
#define FLASH_ACR           REG32(FLASH_BASE + 0x00)

#define FLASH_ACR_LATENCY_5WS   (5U << 0)
#define FLASH_ACR_PRFTEN        (1U << 8)
#define FLASH_ACR_ICEN          (1U << 9)
#define FLASH_ACR_DCEN          (1U << 10)

#define SYSTICK_BASE        0xE000E010UL

typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile uint32_t CALIB;
} SysTick_TypeDef;

#define SYSTICK     ((SysTick_TypeDef *) SYSTICK_BASE)

#define SYSTICK_CTRL_ENABLE     (1U << 0)
#define SYSTICK_CTRL_TICKINT    (1U << 1)
#define SYSTICK_CTRL_CLKSRC     (1U << 2)
#define SYSTICK_CTRL_COUNTFLAG  (1U << 16)

#define NVIC_BASE       0xE000E100UL

typedef struct {
    volatile uint32_t ISER[8];
    volatile uint32_t RESERVED0[24];
    volatile uint32_t ICER[8];
    volatile uint32_t RESERVED1[24];
    volatile uint32_t ISPR[8];
    volatile uint32_t RESERVED2[24];
    volatile uint32_t ICPR[8];
    volatile uint32_t RESERVED3[24];
    volatile uint32_t IABR[8];
    volatile uint32_t RESERVED4[56];
    volatile uint8_t  IP[240];
} NVIC_TypeDef;

#define NVIC    ((NVIC_TypeDef *) NVIC_BASE)

#define TIM2_BASE   (APB1_BASE + 0x0000UL)
#define TIM1_BASE   (APB2_BASE + 0x0000UL)
#define TIM5_BASE   (APB1_BASE + 0x0C00UL)
#define TIM7_BASE   (APB1_BASE + 0x1400UL)
#define TIM8_BASE   (APB2_BASE + 0x0400UL)

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t BDTR;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
    volatile uint32_t OR;
} TIM_TypeDef;

#define TIM2   ((TIM_TypeDef *) TIM2_BASE)
#define TIM1   ((TIM_TypeDef *) TIM1_BASE)
#define TIM5   ((TIM_TypeDef *) TIM5_BASE)
#define TIM7   ((TIM_TypeDef *) TIM7_BASE)
#define TIM8   ((TIM_TypeDef *) TIM8_BASE)

#define RCC_APB1ENR_TIM2EN      (1U << 0)
#define RCC_APB1ENR_TIM5EN      (1U << 3)
#define RCC_APB1ENR_TIM7EN      (1U << 5)

#define TIM_CR1_CEN             (1U << 0)
#define TIM_CR1_URS             (1U << 2)
#define TIM_CR1_OPM             (1U << 3)
#define TIM_DIER_UIE            (1U << 0)
#define TIM_SR_UIF              (1U << 0)
#define TIM_EGR_UG              (1U << 0)
#define NVIC_TIM2_IRQ           28U
#define NVIC_TIM7_IRQ           55U

#define RCC_APB2ENR_TIM1EN   (1U << 0)
#define RCC_APB2ENR_TIM8EN   (1U << 1)

#define TIM_CR1_ARPE         (1U << 7)

#define TIM_CCMR1_OC1PE      (1U << 3)
#define TIM_CCMR1_OC1M_1     (1U << 5)
#define TIM_CCMR1_OC1M_2     (1U << 6)

#define TIM_CCMR2_OC3PE      (1U << 3)
#define TIM_CCMR2_OC3M_1     (1U << 5)
#define TIM_CCMR2_OC3M_2     (1U << 6)
#define TIM_CCMR2_OC4PE      (1U << 11)
#define TIM_CCMR2_OC4M_1     (1U << 13)
#define TIM_CCMR2_OC4M_2     (1U << 14)

#define TIM_CCER_CC1E        (1U << 0)
#define TIM_CCER_CC3E        (1U << 8)
#define TIM_CCER_CC3NE       (1U << 10)
#define TIM_CCER_CC4E        (1U << 12)

#define TIM_BDTR_MOE         (1U << 15)

#define LED_BLUE_PIN    14U
#define LED_RED_PIN   13U

#define SPI1_SCK_PIN    5U
#define SPI1_MISO_PIN   4U
#define SPI1_MOSI_PIN   7U

#define ICM42605_CS_PIN 14U
