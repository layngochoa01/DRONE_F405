#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <stdbool.h>

#define GPS_BAUD   115200U   

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
} GPS_Data_t;

void GPS_Init(void);
void GPS_Process(void);
const GPS_Data_t *GPS_GetData(void);
bool GPS_HasFix(void);
bool GPS_IsNewData(void);

typedef struct {
    uint32_t bytes;       
    uint32_t sentences;   
    uint32_t gga;         
    uint8_t  fixQuality;  
} GPS_Stats_t;
const GPS_Stats_t *GPS_GetStats(void);

#endif
