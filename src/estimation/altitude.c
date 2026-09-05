#include "altitude.h"
#include <math.h>
#include "imu.h"

/* Hệ số complementary filter - P cho vị trí, P cho vận tốc.*/
#define K_ALT   1.0f
#define K_VEL   0.3f

/* cm/s^2 -> m/s^2*/
#define CM_TO_M 0.01f

static AltitudeState_t s_state;
static float s_baselinePressure = 101325.0f;  

void  Altitude_Init(void)
{
    s_state.altitude = 0.0f;
    s_state.verticalVelocity = 0.0f;
}

void Altitude_SetBaseline(float currentPressurePa)
{
    s_baselinePressure = currentPressurePa;
}

float Altitude_PressureToMeters(float pressurePa)
{
    return 44330.0f * (1.0f - powf(pressurePa / s_baselinePressure, 0.1903f));
}

void Altitude_Update(float baroAltitude, float verticalAccel, float dT)
{
    float verticalAccel_m = verticalAccel * CM_TO_M;   
    s_state.verticalVelocity += verticalAccel_m * dT;
    s_state.altitude += s_state.verticalVelocity * dT;

    float error = baroAltitude - s_state.altitude;
    s_state.altitude += error * K_ALT * dT;
    s_state.verticalVelocity += error * K_VEL * dT;
}

const AltitudeState_t *Altitude_Get(void)
{
    return &s_state;
}

float Altitude_ComputeVerticalAccel(const float rMat[3][3], float accelX, float accelY, float accelZ)
{
    float accelZ_world = rMat[2][0]*accelX + rMat[2][1]*accelY + rMat[2][2]*accelZ;
    return accelZ_world - GRAVITY_CMSS;
}