#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <stdbool.h>

#define GPS_BAUD                 115200U

#define GPS_HEALTH_TIMEOUT_MS    500U
#define GPS_HEALTH_MIN_SATS      6U
#define GPS_HEALTH_MAX_HACC_M    3.0f

typedef struct {
    bool     fixValid;
    uint8_t  satellites;
    float    hdop;
    int32_t  latE7;
    int32_t  lonE7;
    float    latitude;
    float    longitude;
    float    altitudeMsl;
    float    speedKmh;
    float    courseDeg;
    bool     velValid;
    float    velN;
    float    velE;

    uint32_t iTOW;
    uint8_t  fixType;
    uint8_t  flags;
    float    hAccM;
    float    vAccM;
    float    sAccMs;

    int32_t  hMSLmm;
    uint32_t hAccMm, vAccMm, sAccMms;
    int32_t  velNmms, velEmms, velDmms, gSpeedMms;
    uint16_t pDOP;
} GPS_Data_t;

typedef struct {
    uint32_t bytes;
    uint32_t packets;
    uint32_t pvt;
    uint32_t badChecksum;
    uint32_t resync;
    uint32_t lastPvtTick;
} GPS_Stats_t;

void GPS_Init(void);
void GPS_Process(void);
const GPS_Data_t  *GPS_GetData(void);
const GPS_Stats_t *GPS_GetStats(void);
bool GPS_IsNewData(void);
bool GPS_IsHealthy(uint32_t nowTick);

#endif
