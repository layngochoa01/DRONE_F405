#ifndef ALTITUDE_H
#define ALTITUDE_H

#include "../sensors/icm42605.h"

typedef struct {
    float altitude;          
    float verticalVelocity;  
} AltitudeState_t;

void  Altitude_Init(void);
void  Altitude_SetBaseline(float currentPressurePa);
float  Altitude_PressureToMeters(float pressurePa);
void Altitude_Update(float baroAltitude, float verticalAccel, float dT);

const AltitudeState_t * Altitude_Get(void);
float  Altitude_ComputeVerticalAccel(const float rMat[3][3], float accelX, float accelY, float accelZ);
#endif