#ifndef POSITION_H
#define POSITION_H

#include <stdint.h>
#include <stdbool.h>
#include "../sensors/gps.h"

#define POS_GPS_TIMEOUT_MS   1500U

typedef struct {
    float posN, posW;
    float velN, velW;
} PositionEstimate_t;

void Position_Init(void);
void Position_Predict(float accN, float accW, float dT);
void Position_UpdateGps(const GPS_Data_t *gps, uint32_t nowMs);
bool Position_IsValid(uint32_t nowMs);
const PositionEstimate_t *Position_Get(void);

#endif
