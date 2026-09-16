#ifndef ALTITUDE_H
#define ALTITUDE_H

#define GRAVITY_MS2 9.80665f

typedef struct {
    float altitude;
    float velocity;
    float accelBias;
} AltitudeKF_State_t;

typedef struct {
    float baselinePressure  ;
} AltitudeBaro_t;

void AltitudeBaro_Init(float baselinePressure);
void AltitudeKF_Init(float processNoiseAlt, float processNoiseVel, float processNoiseBias, float measurementNoiseBaro);

void  Altitude_SetBaseline(float currentPressurePa);

float Altitude_PressureToMeters(float pressurePa);
float Altitude_ComputeVerticalAccel(const float rMat[3][3], float accelX, float accelY, float accelZ);

void AltitudeKF_Predict(float verticalAccel_ms2, float dT);
void AltitudeKF_UpdateBaro(float baroAltitude);

const AltitudeKF_State_t *AltitudeKF_Get(void);
const float GetBaseLinePress(void);
#endif