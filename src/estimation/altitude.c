#include "altitude.h"
#include "../drivers/uart5.h"
#include <math.h>
#include <stdbool.h>
#define BIAS_FREEZE_VELOCITY_THRESHOLD 0.15f
#define VELOCITY_DAMPING 0.998f   /* mỗi lần predict (100 Hz) */
#define BARO_INNOVATION_GATE          3.0f// 0.5f 
#define BARO_MA_WINDOW 4

#define BARO_REJECT_RESET_COUNT  8      /* bị cổng 3σ loại liên tiếp ~1 s (8 Hz) → reset theo baro */
#define BARO_HEALTH_TIMEOUT_MS   500U   /* quá lâu không có mẫu baro được nhận → không khỏe */
#define GROUND_BIAS_ALPHA        0.02f  /* EMA bias khi neo dưới đất (gọi 100 Hz, τ ≈ 0.5 s) */
#define GROUND_P_ALT_VEL         1e-3f  /* hiệp phương sai h, v khi đang neo */

static float x[3]; 
static float P[3][3];
static float Q[3];  //noise
static float R;     //measurement noise R

static float s_pressBuf[BARO_MA_WINDOW];
static uint8_t s_pressIdx = 0;
static bool s_pressBufFull = false;

static AltitudeBaro_t s_baro;  

static uint32_t s_lastAcceptMs = 0;
static bool     s_everAccepted = false;
static uint8_t  s_rejectCount  = 0;

/* Đặt lại h, v về giá trị cho trước, xóa tương quan giữa (h, v) và các trạng thái khác */
static void resetAltVel(float h, float pAltVel)
{
    x[0] = h;
    x[1] = 0.0f;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if ((i < 2 || j < 2) && i != j) P[i][j] = 0.0f;
        }
    }
    P[0][0] = pAltVel;
    P[1][1] = pAltVel;
}

static void mat3Mul(float out[3][3], const float a[3][3], const float b[3][3])
{
    float tmp[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            tmp[i][j] = a[i][0]*b[0][j] + a[i][1]*b[1][j] + a[i][2]*b[2][j];
        }
    }
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            out[i][j] = tmp[i][j];
}

static void mat3Transpose(float out[3][3], const float a[3][3])
{
    float tmp[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            tmp[j][i] = a[i][j];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            out[i][j] = tmp[i][j];
}

void AltitudeBaro_Init(float baselinePressure)
{
    s_baro.baselinePressure = baselinePressure;
}

void Altitude_SetBaseline(float currentPressurePa)
{
    s_baro.baselinePressure = currentPressurePa;
}

float Altitude_PressureToMeters(float pressurePa)
{
    // UART5_WriteF("PTM P: %.3f \n", s_baro.baselinePressure);
    return 44330.0f * (1.0f - powf(pressurePa / s_baro.baselinePressure, 1.0f / 5.255f));
}

float Altitude_ComputeVerticalAccel(const float rMat[3][3], float accelX, float accelY, float accelZ)
{
    float accelZ_world = rMat[2][0]*accelX + rMat[2][1]*accelY + rMat[2][2]*accelZ;
    return accelZ_world - GRAVITY_MS2;
}

void AltitudeKF_Init(float processNoiseAlt, float processNoiseVel, float processNoiseBias, float measurementNoiseBaro)
{
    x[0] = 0.0f;
    x[1] = 0.0f;
    x[2] = 0.0f;

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            P[i][j] = (i == j) ? 1.0f : 0.0f;

    Q[0] = processNoiseAlt;
    Q[1] = processNoiseVel;
    Q[2] = processNoiseBias;

    R = measurementNoiseBaro;
}

void AltitudeKF_Predict(float verticalAccel_ms2, float dT)
{
    float dT2 = dT * dT;
    float accelCorrected = verticalAccel_ms2 - x[2];

    float altNew  = x[0] + x[1] * dT + 0.5f * accelCorrected * dT2;
    float velNew  = (x[1] + accelCorrected*dT) * VELOCITY_DAMPING;
    float biasNew = x[2];

    x[0] = altNew;
    x[1] = velNew;
    x[2] = biasNew;

    float F[3][3] = {
        { 1.0f, dT,  -0.5f*dT2 },
        { 0.0f, 1.0f, -dT      },
        { 0.0f, 0.0f, 1.0f     }
    };
    float Ft[3][3];
    mat3Transpose(Ft, F);

    float FP[3][3];
    mat3Mul(FP, F, P);

    float FPFt[3][3];
    mat3Mul(FPFt, FP, Ft);

    float biasProcessNoise = (fabsf(x[1]) < BIAS_FREEZE_VELOCITY_THRESHOLD) ? Q[2] : 0.0f;

    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            float qAdd = 0.0f;
            if(i == j) qAdd = ( i == 2 ) ? biasProcessNoise : Q[i];
            P[i][j] = FPFt[i][j] + qAdd;
        }
    }
}

bool AltitudeKF_UpdateBaro(float baroAltitude, uint32_t nowMs)
{
    float y = baroAltitude - x[0];
    float S = P[0][0] + R;

    float innovationLimit = BARO_INNOVATION_GATE * sqrtf(S);
    if (fabsf(y) > innovationLimit) {
        /* Loại mẫu bất thường; bị loại liên tục ~1 s thì tin baro, đặt lại ước lượng */
        if (++s_rejectCount < BARO_REJECT_RESET_COUNT) return false;
        resetAltVel(baroAltitude, 1.0f);
        s_rejectCount  = 0;
        s_lastAcceptMs = nowMs;
        s_everAccepted = true;
        return true;
    }
    s_rejectCount = 0;
    float K[3];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    K[2] = (fabsf(x[1]) < BIAS_FREEZE_VELOCITY_THRESHOLD) ? (P[2][0] / S) : 0.0f;
   
    x[0] += K[0] * y;
    x[1] += K[1] * y;
    x[2] += K[2] * y;

    // UART5_WriteF( "BARO: %.3f | ALT: %.3f | VEL: %.3f | K1: %.5f\r\n", baroAltitude, x[0], x[1], K[1] );

    float newP[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            newP[i][j] = P[i][j] - K[i] * P[0][j];
        }
    }
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            P[i][j] = newP[i][j];

    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 3; j++) {
            float avg = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = avg;
            P[j][i] = avg;
        }
    }

    s_lastAcceptMs = nowMs;
    s_everAccepted = true;
    return true;
}

bool AltitudeKF_IsHealthy(uint32_t nowMs)
{
    return s_everAccepted && ((nowMs - s_lastAcceptMs) <= BARO_HEALTH_TIMEOUT_MS);
}

void AltitudeKF_GroundAnchor(float verticalAccel_ms2)
{
    /* Dưới đất đứng yên: gia tốc đứng (đã trừ g) chính là bias */
    x[2] += GROUND_BIAS_ALPHA * (verticalAccel_ms2 - x[2]);
    resetAltVel(0.0f, GROUND_P_ALT_VEL);
}

void Altitude_NudgeBaseline(float pressurePa, float alpha)
{
    s_baro.baselinePressure += alpha * (pressurePa - s_baro.baselinePressure);
}

const AltitudeKF_State_t *AltitudeKF_Get(void)
{
    static AltitudeKF_State_t out;
    out.altitude  = x[0];
    out.velocity  = x[1];
    out.accelBias = x[2];
    return &out;
}

float GetBaseLinePress()
{
    return s_baro.baselinePressure;
}

float Baro_MovingAverage(float newPressure)
{
    s_pressBuf[s_pressIdx] = newPressure;
    s_pressIdx = (s_pressIdx + 1) % BARO_MA_WINDOW;
    if (s_pressIdx == 0) s_pressBufFull = true;

    int n = s_pressBufFull ? BARO_MA_WINDOW : s_pressIdx;
    float sum = 0;
    for (int i = 0; i < n; i++) sum += s_pressBuf[i];
    return sum / (float)n;
}