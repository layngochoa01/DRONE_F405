#ifndef IBUS_H
#define IBUS_H

#include <stdint.h>
#include <stdbool.h>

#define IBUS_CHANNEL_COUNT 14

extern volatile uint8_t  g_dbgBuf[64];
extern volatile uint8_t  g_dbgIdx;
extern volatile uint8_t  g_dbgDone;
extern volatile uint32_t g_errORE, g_errFE, g_errNE;
extern volatile uint32_t g_hdrCount;
extern volatile uint32_t g_frames, g_badCmd, g_badSum, g_goodFrames;

void IBUS_Init(uint32_t apb1_clk);
void IBUS_ISR_Handler(void);           
bool IBUS_IsFrameReady(void);
bool IBUS_IsConnected(void);
uint16_t IBUS_GetChannel(uint8_t index); 

extern volatile uint8_t  g_ibusLastRaw;
extern volatile uint32_t g_ibusRawCount;
#endif