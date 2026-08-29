#include "i2c.h"
#include "stm32f4xx.h"   // header CMSIS bạn đang dùng cho project — đổi tên nếu khác

#define I2C_TIMEOUT 100000UL

static bool I2C1_WaitFlag(volatile uint32_t *reg, uint32_t flag, uint8_t setState)
{
    uint32_t timeout = I2C_TIMEOUT;
    while (((*reg & flag) ? 1 : 0) != setState) {
        if (--timeout == 0) return false;
    }
    return true;
}

void I2C1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER &= ~((3U << (8*2)) | (3U << (7*2)));
    GPIOB->MODER |=  ((2U << (8*2)) | (2U << (7*2)));   

    GPIOB->OTYPER |= (1U << 8) | (1U << 7);             

    GPIOB->OSPEEDR |= (3U << (8*2)) | (3U << (7*2));    

    GPIOB->PUPDR &= ~((3U << (8*2)) | (3U << (7*2)));
    GPIOB->PUPDR |=  ((1U << (8*2)) | (1U << (7*2)));   

    GPIOB->AFR[1] &= ~((0xF << ((8-8)*4)) | (0xF << ((7-8)*4)));
    GPIOB->AFR[1] |=  ((4U  << ((8-8)*4)) | (4U  << ((7-8)*4)));

    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    I2C1->CR2 = 42;                
    I2C1->CCR = 210;                
    I2C1->TRISE = 43;               

    I2C1->CR1 |= I2C_CR1_PE;
}

static bool I2C1_Start(uint8_t devAddr, uint8_t rw)
{
    I2C1->CR1 |= I2C_CR1_START;
    if (!I2C1_WaitFlag(&I2C1->SR1, I2C_SR1_SB, 1)) return false;

    I2C1->DR = (devAddr << 1) | rw;
    if (!I2C1_WaitFlag(&I2C1->SR1, I2C_SR1_ADDR, 1)) return false;

    (void)I2C1->SR1;
    (void)I2C1->SR2;   

    return true;
}

static void I2C1_Stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
}

bool I2C1_WriteReg(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
    if (!I2C1_Start(devAddr, 0)) { I2C1_Stop(); return false; }

    I2C1->DR = regAddr;
    if (!I2C1_WaitFlag(&I2C1->SR1, I2C_SR1_TXE, 1)) { I2C1_Stop(); return false; }

    I2C1->DR = data;
    if (!I2C1_WaitFlag(&I2C1->SR1, I2C_SR1_BTF, 1)) { I2C1_Stop(); return false; }

    I2C1_Stop();
    return true;
}

bool I2C1_ReadReg(uint8_t devAddr, uint8_t regAddr, uint8_t *data)
{
    return I2C1_ReadRegs(devAddr, regAddr, data, 1);
}

bool I2C1_ReadRegs(uint8_t devAddr, uint8_t regAddr, uint8_t *buf, uint16_t len)
{
    if (!I2C1_Start(devAddr, 0)) { I2C1_Stop(); return false; }

    I2C1->DR = regAddr;
    if (!I2C1_WaitFlag(&I2C1->SR1, I2C_SR1_TXE, 1)) { I2C1_Stop(); return false; }

    I2C1->CR1 |= I2C_CR1_ACK;
    if (!I2C1_Start(devAddr, 1)) { I2C1_Stop(); return false; }

    for (uint16_t i = 0; i < len; i++) {
        if (i == len - 1) {
            I2C1->CR1 &= ~I2C_CR1_ACK;  
            I2C1_Stop();
        }
        if (!I2C1_WaitFlag(&I2C1->SR1, I2C_SR1_RXNE, 1)) return false;
        buf[i] = I2C1->DR;
    }

    return true;
}