#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>

void I2C1_Init(void);
bool I2C1_WriteReg(uint8_t devAddr, uint8_t regAddr, uint8_t data);
bool I2C1_ReadReg(uint8_t devAddr, uint8_t regAddr, uint8_t *data);
bool I2C1_ReadRegs(uint8_t devAddr, uint8_t regAddr, uint8_t *buf, uint16_t len);

#endif