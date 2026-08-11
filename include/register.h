#pragma once

#include <stdint.h>

/*
 * register.h - STM32F405 peripheral register map
 * Tự định nghĩa địa chỉ thanh ghi, không dùng stm32f4xx.h
 * Tham chiếu: STM32F405 Reference Manual (RM0090)
 */

/* =========================================================
 * KIỂU DỮ LIỆU TRUY CẬP THANH GHI
 * volatile: báo compiler không cache giá trị, phải đọc RAM/register thật
 * ========================================================= */
#define REG32(addr)   (*(volatile uint32_t *)(addr))
#define REG16(addr)   (*(volatile uint16_t *)(addr))
#define REG8(addr)    (*(volatile uint8_t  *)(addr))

/* =========================================================
 * BASE ADDRESS - RM0090 Table 1 (Memory Map)
 * ========================================================= */

/* AHB1 Bus - GPIO, DMA, RCC */
#define PERIPH_BASE         0x40000000UL
#define AHB1_BASE           (PERIPH_BASE + 0x00020000UL)
#define APB2_BASE           (PERIPH_BASE + 0x00010000UL)
#define APB1_BASE           (PERIPH_BASE + 0x00000000UL)

/* GPIO base addresses - RM0090 Table 1 */
#define GPIOA_BASE          (AHB1_BASE + 0x0000UL)   /* 0x40020000 */
#define GPIOB_BASE          (AHB1_BASE + 0x0400UL)   /* 0x40020400 */
#define GPIOC_BASE          (AHB1_BASE + 0x0800UL)   /* 0x40020800 */
#define GPIOD_BASE          (AHB1_BASE + 0x0C00UL)   /* 0x40020C00 */

/* RCC - Reset and Clock Control */
#define RCC_BASE            (AHB1_BASE + 0x3800UL)   /* 0x40023800 */

/* SPI1 - APB2 bus */
#define SPI1_BASE           (APB2_BASE + 0x3000UL)   /* 0x40013000 */

/* UART5 - APB1 bus */
#define UART5_BASE          (APB1_BASE + 0x5000UL)   /* 0x40005000 */

#define DMA2_BASE           (AHB1_BASE + 0x6400UL)   /* 0x40026400 */

/* =========================================================
 * RCC REGISTERS - RM0090 Section 6.3
 * ========================================================= */
typedef struct {
    volatile uint32_t CR;           /* 0x00 - Clock control */
    volatile uint32_t PLLCFGR;     /* 0x04 - PLL configuration */
    volatile uint32_t CFGR;        /* 0x08 - Clock configuration */
    volatile uint32_t CIR;         /* 0x0C - Clock interrupt */
    volatile uint32_t AHB1RSTR;    /* 0x10 - AHB1 peripheral reset */
    volatile uint32_t AHB2RSTR;    /* 0x14 */
    volatile uint32_t AHB3RSTR;    /* 0x18 */
    volatile uint32_t RESERVED0;   /* 0x1C */
    volatile uint32_t APB1RSTR;    /* 0x20 - APB1 peripheral reset */
    volatile uint32_t APB2RSTR;    /* 0x24 - APB2 peripheral reset */
    volatile uint32_t RESERVED1;   /* 0x28 */
    volatile uint32_t RESERVED2;   /* 0x2C */
    volatile uint32_t AHB1ENR;     /* 0x30 - AHB1 clock enable ← GPIO clock */
    volatile uint32_t AHB2ENR;     /* 0x34 */
    volatile uint32_t AHB3ENR;     /* 0x38 */
    volatile uint32_t RESERVED3;   /* 0x3C */
    volatile uint32_t APB1ENR;     /* 0x40 - APB1 clock enable */
    volatile uint32_t APB2ENR;     /* 0x44 - APB2 clock enable ← SPI1 clock */
    volatile uint32_t RESERVED4;   /* 0x48 */
    volatile uint32_t RESERVED5;   /* 0x4C */
    volatile uint32_t AHB1LPENR;   /* 0x50 */
    volatile uint32_t AHB2LPENR;   /* 0x54 */
    volatile uint32_t AHB3LPENR;   /* 0x58 */
    volatile uint32_t RESERVED6;   /* 0x5C */
    volatile uint32_t APB1LPENR;   /* 0x60 */
    volatile uint32_t APB2LPENR;   /* 0x64 */
    volatile uint32_t RESERVED7;   /* 0x68 */
    volatile uint32_t RESERVED8;   /* 0x6C */
    volatile uint32_t BDCR;        /* 0x70 - Backup domain control */
    volatile uint32_t CSR;         /* 0x74 - Clock control & status */
    volatile uint32_t RESERVED9;   /* 0x78 */
    volatile uint32_t RESERVED10;  /* 0x7C */
    volatile uint32_t SSCGR;       /* 0x80 */
    volatile uint32_t PLLI2SCFGR;  /* 0x84 */
} RCC_TypeDef;

#define RCC     ((RCC_TypeDef *) RCC_BASE)

/* RCC AHB1ENR bits - RM0090 6.3.10 */
#define RCC_AHB1ENR_GPIOAEN     (1U << 0)   /* GPIOA clock enable */
#define RCC_AHB1ENR_GPIOBEN     (1U << 1)   /* GPIOB clock enable */
#define RCC_AHB1ENR_GPIOCEN     (1U << 2)   /* GPIOC clock enable */
#define RCC_AHB1ENR_GPIODEN     (1U << 3)   /* GPIOD clock enable */

/* RCC APB2ENR bits - RM0090 6.3.14 */
#define RCC_APB2ENR_SPI1EN      (1U << 12)  /* SPI1 clock enable */

/* RCC CR bits */
#define RCC_CR_HSION            (1U << 0)   /* HSI oscillator ON */
#define RCC_CR_HSIRDY           (1U << 1)   /* HSI ready */
#define RCC_CR_HSEON            (1U << 16)  /* HSE ON */
#define RCC_CR_HSERDY           (1U << 17)  /* HSE ready */
#define RCC_CR_PLLON            (1U << 24)  /* PLL ON */
#define RCC_CR_PLLRDY           (1U << 25)  /* PLL ready */

/* RCC CFGR bits */
#define RCC_CFGR_SW_PLL         (0x2U << 0) /* PLL as system clock */
#define RCC_CFGR_SWS_PLL        (0x2U << 2) /* PLL used as system clock (status) */
#define RCC_CFGR_HPRE_DIV1      (0x0U << 4) /* AHB = SYSCLK / 1 */
#define RCC_CFGR_PPRE1_DIV4     (0x5U << 10)/* APB1 = AHB / 4 */
#define RCC_CFGR_PPRE2_DIV2     (0x4U << 13)/* APB2 = AHB / 2 */

/* RCC PLLCFGR - RM0090 6.3.2
 * BLUEBERRY dùng HSI 16MHz làm nguồn PLL
 * Công thức: SYSCLK = (HSI/M) * N / P
 * Target: 168MHz
 * M=16, N=336, P=2 → (16/16)*336/2 = 168MHz
 */
#define RCC_PLLCFGR_PLLM        (16U << 0)  /* M = 16 */
#define RCC_PLLCFGR_PLLN        (336U << 6) /* N = 336 */
#define RCC_PLLCFGR_PLLP_DIV2  (0x0U << 16)/* P = 2 */
#define RCC_PLLCFGR_PLLSRC_HSI (0U << 22)  /* HSI làm nguồn PLL */
#define RCC_PLLCFGR_PLLQ       (7U << 24)  /* Q = 7 cho USB (48MHz) */

/* =========================================================
 * GPIO REGISTERS - RM0090 Section 8.4
 * ========================================================= */
typedef struct {
    volatile uint32_t MODER;    /* 0x00 - Mode register */
    volatile uint32_t OTYPER;   /* 0x04 - Output type register */
    volatile uint32_t OSPEEDR;  /* 0x08 - Output speed register */
    volatile uint32_t PUPDR;    /* 0x0C - Pull-up/pull-down register */
    volatile uint32_t IDR;      /* 0x10 - Input data register */
    volatile uint32_t ODR;      /* 0x14 - Output data register */
    volatile uint32_t BSRR;     /* 0x18 - Bit set/reset register */
    volatile uint32_t LCKR;     /* 0x1C - Configuration lock register */
    volatile uint32_t AFR[2];   /* 0x20-0x24 - Alternate function low/high */
} GPIO_TypeDef;

#define GPIOA   ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB   ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC   ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD   ((GPIO_TypeDef *) GPIOD_BASE)

/* GPIO MODER values (2 bits per pin) - RM0090 8.4.1 */
#define GPIO_MODE_INPUT     0x0U    /* 00: Input */
#define GPIO_MODE_OUTPUT    0x1U    /* 01: Output */
#define GPIO_MODE_AF        0x2U    /* 10: Alternate function */
#define GPIO_MODE_ANALOG    0x3U    /* 11: Analog */

/* GPIO OSPEEDR values - RM0090 8.4.3 */
#define GPIO_SPEED_LOW      0x0U
#define GPIO_SPEED_MEDIUM   0x1U
#define GPIO_SPEED_HIGH     0x2U
#define GPIO_SPEED_VERY_HIGH 0x3U

/* GPIO PUPDR values - RM0090 8.4.4 */
#define GPIO_PUPD_NONE      0x0U    /* No pull */
#define GPIO_PUPD_UP        0x1U    /* Pull-up */
#define GPIO_PUPD_DOWN      0x2U    /* Pull-down */

/* GPIO BSRR - Bit Set/Reset Register
 * Bit [15:0]  = BS (Set)   → ghi 1 để SET pin HIGH
 * Bit [31:16] = BR (Reset) → ghi 1 để SET pin LOW
 * Đây là atomic operation, an toàn hơn đọc-sửa-ghi ODR
 */
#define GPIO_BSRR_SET(pin)      (1U << (pin))
#define GPIO_BSRR_RESET(pin)    (1U << ((pin) + 16))

/* Alternate Function numbers - RM0090 Table 9 */
#define GPIO_AF5_SPI1   5U  /* SPI1 dùng AF5 */

/* RCC APB1ENR bits - RM0090 6.3.13 */
#define RCC_APB1ENR_UART5EN     (1U << 20)  /* UART5 clock enable */

/* Alternate Function numbers - RM0090 Table 9 */
#define GPIO_AF8_UART5      8U  /* UART5 dùng AF8 */

/* =========================================================
 * SPI REGISTERS - RM0090 Section 28.5
 * ========================================================= */
typedef struct {
    volatile uint32_t CR1;      /* 0x00 - Control register 1 */
    volatile uint32_t CR2;      /* 0x04 - Control register 2 */
    volatile uint32_t SR;       /* 0x08 - Status register */
    volatile uint32_t DR;       /* 0x0C - Data register */
    volatile uint32_t CRCPR;    /* 0x10 - CRC polynomial */
    volatile uint32_t RXCRCR;   /* 0x14 - RX CRC register */
    volatile uint32_t TXCRCR;   /* 0x18 - TX CRC register */
    volatile uint32_t I2SCFGR;  /* 0x1C - I2S configuration */
    volatile uint32_t I2SPR;    /* 0x20 - I2S prescaler */
} SPI_TypeDef;

#define SPI1    ((SPI_TypeDef *) SPI1_BASE)

/* SPI CR1 bits - RM0090 28.5.1 */
#define SPI_CR1_CPHA        (1U << 0)   /* Clock phase */
#define SPI_CR1_CPOL        (1U << 1)   /* Clock polarity */
#define SPI_CR1_MSTR        (1U << 2)   /* Master mode */
#define SPI_CR1_BR_DIV2     (0x0U << 3) /* Baud = fPCLK/2 */
#define SPI_CR1_BR_DIV4     (0x1U << 3) /* Baud = fPCLK/4 */
#define SPI_CR1_BR_DIV8     (0x2U << 3) /* Baud = fPCLK/8 */
#define SPI_CR1_BR_DIV16    (0x3U << 3) /* Baud = fPCLK/16 */
#define SPI_CR1_BR_DIV32    (0x4U << 3) /* Baud = fPCLK/32 */
#define SPI_CR1_BR_DIV64    (0x5U << 3) /* Baud = fPCLK/64 */
#define SPI_CR1_BR_DIV128   (0x6U << 3) /* Baud = fPCLK/128 */
#define SPI_CR1_BR_DIV256   (0x7U << 3) /* Baud = fPCLK/256 */
#define SPI_CR1_SPE         (1U << 6)   /* SPI enable */
#define SPI_CR1_SSI         (1U << 8)   /* Internal slave select */
#define SPI_CR1_SSM         (1U << 9)   /* Software slave management */
#define SPI_CR1_DFF         (1U << 11)  /* Data frame format (0=8bit, 1=16bit) */

/* SPI SR bits - RM0090 28.5.3 */
#define SPI_SR_RXNE         (1U << 0)   /* Receive buffer not empty */
#define SPI_SR_TXE          (1U << 1)   /* Transmit buffer empty */
#define SPI_SR_BSY          (1U << 7)   /* Busy flag */

/* =========================================================
 * USART/UART REGISTERS - RM0090 Section 30.6
 * ========================================================= */
typedef struct {
    volatile uint32_t SR;       /* 0x00 - Status register */
    volatile uint32_t DR;       /* 0x04 - Data register */
    volatile uint32_t BRR;      /* 0x08 - Baud rate register */
    volatile uint32_t CR1;      /* 0x0C - Control register 1 */
    volatile uint32_t CR2;      /* 0x10 - Control register 2 */
    volatile uint32_t CR3;      /* 0x14 - Control register 3 */
    volatile uint32_t GTPR;     /* 0x18 - Guard time and prescaler */
} USART_TypeDef;

#define UART5   ((USART_TypeDef *) UART5_BASE)

/* USART CR1 bits - RM0090 30.6.4 */
#define USART_CR1_RE        (1U << 2)   /* Receiver enable */
#define USART_CR1_TE        (1U << 3)   /* Transmitter enable */
#define USART_CR1_RXNEIE    (1U << 5)   /* RXNE interrupt enable */
#define USART_CR1_TXEIE     (1U << 7)   /* TXE interrupt enable */
#define USART_CR1_UE        (1U << 13)  /* USART enable */

/* USART SR bits - RM0090 30.6.1 */
#define USART_SR_PE         (1U << 0)   /* Parity error    */
#define USART_SR_FE         (1U << 1)   /* Framing error   */
#define USART_SR_NE         (1U << 2)   /* Noise error     */
#define USART_SR_ORE        (1U << 3)   /* Overrun error   */
#define USART_SR_TC         (1U << 6)   /* TX complete     */
#define USART_SR_RXNE       (1U << 5)   /* Read data register not empty */
#define USART_SR_TXE        (1U << 7)   /* Transmit data register empty */

/* =========================================================
 * DMA STREAM REGISTERS
 * ========================================================= */
typedef struct {
    volatile uint32_t CR;       /* 0x00 - Configuration register */
    volatile uint32_t NDTR;     /* 0x04 - Number of data register */
    volatile uint32_t PAR;      /* 0x08 - Peripheral address */
    volatile uint32_t M0AR;     /* 0x0C - Memory 0 address */
    volatile uint32_t M1AR;     /* 0x10 - Memory 1 address */
    volatile uint32_t FCR;      /* 0x14 - FIFO control */
} DMA_Stream_TypeDef;
 
/* DMA global registers */
typedef struct {
    volatile uint32_t LISR;     /* 0x00 - Low interrupt status  (stream 0-3) */
    volatile uint32_t HISR;     /* 0x04 - High interrupt status (stream 4-7) */
    volatile uint32_t LIFCR;    /* 0x08 - Low interrupt flag clear  */
    volatile uint32_t HIFCR;    /* 0x0C - High interrupt flag clear */
    DMA_Stream_TypeDef S[8];    /* 0x10 - Stream 0..7 */
} DMA_TypeDef;
 
#define DMA2    ((DMA_TypeDef *) DMA2_BASE)


/* DMA Stream CR bits - RM0090 10.5.5 */
#define DMA_CR_EN           (1U << 0)    /* Stream enable */
#define DMA_CR_DMEIE        (1U << 1)    /* Direct mode error interrupt */
#define DMA_CR_TEIE         (1U << 2)    /* Transfer error interrupt */
#define DMA_CR_HTIE         (1U << 3)    /* Half transfer interrupt */
#define DMA_CR_TCIE         (1U << 4)    /* Transfer complete interrupt */
#define DMA_CR_PFCTRL       (1U << 5)    /* Peripheral flow control */
 
/* Direction */
#define DMA_CR_DIR_P2M      (0x0U << 6)  /* Peripheral → Memory */
#define DMA_CR_DIR_M2P      (0x1U << 6)  /* Memory → Peripheral */
 
#define DMA_CR_CIRC         (1U << 8)    /* Circular mode */
#define DMA_CR_PINC         (1U << 9)    /* Peripheral increment */
#define DMA_CR_MINC         (1U << 10)   /* Memory increment */
 
/* Data size */
#define DMA_CR_PSIZE_8      (0x0U << 11) /* Peripheral size: byte */
#define DMA_CR_MSIZE_8      (0x0U << 13) /* Memory size: byte */
 
/* Priority */
#define DMA_CR_PL_LOW       (0x0U << 16)
#define DMA_CR_PL_MEDIUM    (0x1U << 16)
#define DMA_CR_PL_HIGH      (0x2U << 16)
#define DMA_CR_PL_VHIGH     (0x3U << 16)
 
/* Channel select (bits [27:25]) */
#define DMA_CR_CHSEL_3      (0x3U << 25) /* Channel 3 = SPI1 TX (Stream3) */
#define DMA_CR_CHSEL_3_RX   (0x3U << 25) /* Channel 3 = SPI1 RX (Stream0) */
 
/* DMA2 LISR/LIFCR flags cho Stream0 (RX) - RM0090 10.5.2 */
#define DMA2_S0_TCIF        (1U << 5)    /* Stream0 transfer complete */
#define DMA2_S0_HTIF        (1U << 4)    /* Stream0 half transfer */
#define DMA2_S0_TEIF        (1U << 3)    /* Stream0 transfer error */
#define DMA2_S0_DMEIF       (1U << 2)    /* Stream0 direct mode error */
#define DMA2_S0_FEIF        (1U << 0)    /* Stream0 FIFO error */
 
/* DMA2 LISR/LIFCR flags cho Stream3 (TX) */
#define DMA2_S3_TCIF        (1U << 27)
#define DMA2_S3_HTIF        (1U << 26)
#define DMA2_S3_TEIF        (1U << 25)
#define DMA2_S3_DMEIF       (1U << 24)
#define DMA2_S3_FEIF        (1U << 22)


/* RCC AHB1ENR - DMA2 clock enable */
#define RCC_AHB1ENR_DMA2EN  (1U << 22)
 
/* SPI CR2 - DMA enable bits */
#define SPI_CR2_RXDMAEN     (1U << 0)    /* RX DMA enable */
#define SPI_CR2_TXDMAEN     (1U << 1)    /* TX DMA enable */
 
/* NVIC IRQ numbers cho DMA2 */
/* DMA2_Stream0 = IRQ56, DMA2_Stream3 = IRQ59 */
/* ISER[1]: IRQ56 → bit 24, IRQ59 → bit 27   */

#define NVIC_DMA2_S0_IRQ    56U
#define NVIC_DMA2_S3_IRQ    59U
 
/* SCB - System Control Block (cho FPU, VTOR) */
#define SCB_BASE            0xE000ED00UL
typedef struct {
    volatile uint32_t CPUID;    /* 0x00 */
    volatile uint32_t ICSR;     /* 0x04 */
    volatile uint32_t VTOR;     /* 0x08 */
    volatile uint32_t AIRCR;    /* 0x0C */
    volatile uint32_t SCR;      /* 0x10 */
    volatile uint32_t CCR;      /* 0x14 */
    volatile uint8_t  SHP[12];  /* 0x18 */
    volatile uint32_t SHCSR;    /* 0x24 */
    volatile uint32_t CFSR;     /* 0x28 */
    volatile uint32_t HFSR;     /* 0x2C */
    volatile uint32_t RESERVED; /* 0x30 */
    volatile uint32_t MMFAR;    /* 0x34 */
    volatile uint32_t BFAR;     /* 0x38 */
    volatile uint32_t AFSR;     /* 0x3C */
    volatile uint32_t RESERVED2[19];
    volatile uint32_t CPACR;    /* 0x88 - FPU enable */
} SCB_TypeDef;
#define SCB     ((SCB_TypeDef *) SCB_BASE)
/* =========================================================
 * FLASH - cần set latency trước khi tăng clock
 * RM0090 Section 3.4
 * ========================================================= */
#define FLASH_BASE          0x40023C00UL
#define FLASH_ACR           REG32(FLASH_BASE + 0x00)

#define FLASH_ACR_LATENCY_5WS   (5U << 0)  /* 5 wait states cho 168MHz */
#define FLASH_ACR_PRFTEN        (1U << 8)   /* Prefetch enable */
#define FLASH_ACR_ICEN          (1U << 9)   /* Instruction cache enable */
#define FLASH_ACR_DCEN          (1U << 10)  /* Data cache enable */

/* =========================================================
 * SYSTICK - ARM Cortex-M4 core peripheral
 * ARM v7-M Architecture Reference Manual B3.3
 * ========================================================= */
#define SYSTICK_BASE        0xE000E010UL

typedef struct {
    volatile uint32_t CTRL;     /* 0x00 - Control and status */
    volatile uint32_t LOAD;     /* 0x04 - Reload value */
    volatile uint32_t VAL;      /* 0x08 - Current value */
    volatile uint32_t CALIB;    /* 0x0C - Calibration */
} SysTick_TypeDef;

#define SYSTICK     ((SysTick_TypeDef *) SYSTICK_BASE)

#define SYSTICK_CTRL_ENABLE     (1U << 0)   /* Counter enable */
#define SYSTICK_CTRL_TICKINT    (1U << 1)   /* Exception request enable */
#define SYSTICK_CTRL_CLKSRC     (1U << 2)   /* Clock source: 1=AHB, 0=AHB/8 */
#define SYSTICK_CTRL_COUNTFLAG  (1U << 16)  /* Counted to 0 since last read */

/* =========================================================
 * NVIC - Nested Vectored Interrupt Controller
 * ARM v7-M Architecture Reference Manual B3.4
 * ========================================================= */
#define NVIC_BASE       0xE000E100UL

typedef struct {
    volatile uint32_t ISER[8];      /* 0x000 - Interrupt Set Enable    */
    volatile uint32_t RESERVED0[24];
    volatile uint32_t ICER[8];      /* 0x080 - Interrupt Clear Enable  */
    volatile uint32_t RESERVED1[24];
    volatile uint32_t ISPR[8];      /* 0x100 - Interrupt Set Pending   */
    volatile uint32_t RESERVED2[24];
    volatile uint32_t ICPR[8];      /* 0x180 - Interrupt Clear Pending */
    volatile uint32_t RESERVED3[24];
    volatile uint32_t IABR[8];      /* 0x200 - Interrupt Active Bit    */
    volatile uint32_t RESERVED4[56];
    volatile uint8_t  IP[240];      /* 0x300 - Interrupt Priority      */
} NVIC_TypeDef;


#define NVIC    ((NVIC_TypeDef *) NVIC_BASE)


/* =========================================================
 * PIN DEFINITIONS - Theo target.h của BLUEBERRY F405
 * ========================================================= */

/* LED pins */
#define LED_BLUE_PIN    14U   /* PA14 - LED xanh dương */
#define LED_RED_PIN   13U   /* PA13 - LED đỏ */

/* SPI1 pins - RM0090 Table 9 AF mapping */
#define SPI1_SCK_PIN    5U    /* PA5  - SPI1_SCK  (AF5) */
#define SPI1_MISO_PIN   4U    /* PB4  - SPI1_MISO (AF5) */  /* BLUEBERRY dùng PB4, không phải PA6 */
#define SPI1_MOSI_PIN   7U    /* PA7  - SPI1_MOSI (AF5) */

/* ICM42605 CS pin */
#define ICM42605_CS_PIN 14U   /* PC14 - Chip Select (GPIO output) */