#ifndef ALTITUDE_H
#define ALTITUDE_H

#include <stdint.h>
#include <stdbool.h>

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
/* Trả về true nếu mẫu được nhận (hoặc đã reset theo baro sau ~1 s bị loại liên tục) */
bool AltitudeKF_UpdateBaro(float baroAltitude, uint32_t nowMs);
/* false nếu > 500 ms không có mẫu baro nào được nhận */
bool AltitudeKF_IsHealthy(uint32_t nowMs);
/* Gọi khi drone ở dưới đất (DISARMED / IDLE): neo h = 0, v = 0, học bias accel */
void AltitudeKF_GroundAnchor(float verticalAccel_ms2);
/* Kéo dần áp suất gốc P0 về áp suất đo được (bù thời tiết khi đứng dưới đất) */
void Altitude_NudgeBaseline(float pressurePa, float alpha);

const AltitudeKF_State_t *AltitudeKF_Get(void);
float GetBaseLinePress(void);

float Baro_MovingAverage(float newPressure);
#endif