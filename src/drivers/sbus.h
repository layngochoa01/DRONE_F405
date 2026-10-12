#ifndef SBUS_H
#define SBUS_H

#include <stdint.h>
#include <stdbool.h>

#define SBUS_CHANNEL_COUNT 16

void SBUS_Init(uint32_t apb1_clk);
void SBUS_ISR_Handler(void);
bool SBUS_IsConnected(void);
bool SBUS_IsFailsafe(void);
uint16_t SBUS_GetChannel(uint8_t index);

#endif
