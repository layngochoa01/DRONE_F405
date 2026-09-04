#ifndef ALTITUDE_H
#define ALTITUDE_H

#include "../sensors/icm42605.h"

typedef struct {
    float altitude;          
    float verticalVelocity;  
} AltitudeState_t;

void  init(void);
void  setBaseline(float currentPressurePa);
float  pressureToMeters(float pressurePa);
void update(float baroAltitude, float verticalAccel, float dT);

const AltitudeState_t * get(void);
float  computeVerticalAccel(const float rMat[3][3], float accelX, float accelY, float accelZ);
#endif