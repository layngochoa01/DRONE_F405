#ifndef SBUS_H
#define SBUS_H

#include <stdint.h>
#include <stdbool.h>

#define SBUS_CHANNEL_COUNT 16

void SBUS_Init(uint32_t apb1_clk);
void SBUS_ISR_Handler(void);
bool SBUS_IsFrameReady(void);
bool SBUS_IsConnected(void);
bool SBUS_IsFailsafe(void);
uint16_t SBUS_GetChannel(uint8_t index);   

extern volatile uint8_t  g_sbusLastRaw;
extern volatile uint32_t g_sbusRawCount;

extern volatile uint32_t g_sbusBytes;
extern volatile uint32_t g_sbusFrames;
extern volatile uint32_t g_sbusGood;
extern volatile uint32_t g_sbusBadFooter;
extern volatile uint32_t g_sbusBadUart;

extern volatile uint32_t g_sbusErrFE;
extern volatile uint32_t g_sbusErrNE;
extern volatile uint32_t g_sbusErrPE;
extern volatile uint32_t g_sbusErrORE;

#endif