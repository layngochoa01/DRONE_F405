#include "position.h"
#include <math.h>

#define EARTH_RADIUS_M     6371000.0f
#define DEG_E7_TO_RAD      (3.14159265f / 180.0f * 1e-7f)

/* Hệ số hiệu chỉnh mỗi lần có fix GPS (0..1) */
#define K_POS              0.30f   /* kéo vị trí về GPS */
#define K_VEL              0.30f   /* kéo vận tốc về vận tốc GPS */
#define K_VEL_FROM_POS     0.10f   /* sửa vận tốc từ sai số vị trí (khi không có vận tốc GPS) */

#define MAX_GPS_JUMP_M     50.0f   /* reset estimator nếu GPS nhảy xa hơn mức này */

static PositionEstimate_t s_est;
static bool     s_homeSet = false;
static int32_t  s_homeLatE7, s_homeLonE7;
static float    s_lonScale = 1.0f;     /* cos(lat home) */
static uint32_t s_lastGpsMs = 0;

void Position_Init(void)
{
    s_est = (PositionEstimate_t){ 0 };
    s_homeSet = false;
    s_lastGpsMs = 0;
}

void Position_Predict(float accN, float accW, float dT)
{
    if (!s_homeSet) return;

    s_est.posN += s_est.velN * dT + 0.5f * accN * dT * dT;
    s_est.posW += s_est.velW * dT + 0.5f * accW * dT * dT;
    s_est.velN += accN * dT;
    s_est.velW += accW * dT;
}

void Position_UpdateGps(const GPS_Data_t *gps, uint32_t nowMs)
{
    if (!gps->fixValid || gps->satellites < POS_GPS_MIN_SATS || gps->hdop > POS_GPS_MAX_HDOP) {
        return;
    }

    if (!s_homeSet) {
        s_homeLatE7 = gps->latE7;
        s_homeLonE7 = gps->lonE7;
        s_lonScale  = cosf((float)gps->latE7 * DEG_E7_TO_RAD);
        s_homeSet   = true;
        s_est = (PositionEstimate_t){ 0 };
    }

    /* Hiệu E7 là số nguyên → không mất chính xác khi đổi sang mét */
    float gpsN =  (float)(gps->latE7 - s_homeLatE7) * DEG_E7_TO_RAD * EARTH_RADIUS_M;
    float gpsE =  (float)(gps->lonE7 - s_homeLonE7) * DEG_E7_TO_RAD * EARTH_RADIUS_M * s_lonScale;
    float gpsW = -gpsE;

    float errN = gpsN - s_est.posN;
    float errW = gpsW - s_est.posW;

    bool timedOut = (nowMs - s_lastGpsMs) > POS_GPS_TIMEOUT_MS;
    if (timedOut || fabsf(errN) > MAX_GPS_JUMP_M || fabsf(errW) > MAX_GPS_JUMP_M) {
        s_est.posN = gpsN;
        s_est.posW = gpsW;
        s_est.velN = gps->velValid ?  gps->velN : 0.0f;
        s_est.velW = gps->velValid ? -gps->velE : 0.0f;
        s_lastGpsMs = nowMs;
        return;
    }

    s_est.posN += K_POS * errN;
    s_est.posW += K_POS * errW;

    if (gps->velValid) {
        s_est.velN += K_VEL * ( gps->velN - s_est.velN);
        s_est.velW += K_VEL * (-gps->velE - s_est.velW);
    } else {
        s_est.velN += K_VEL_FROM_POS * errN;
        s_est.velW += K_VEL_FROM_POS * errW;
    }

    s_lastGpsMs = nowMs;
}

bool Position_IsValid(uint32_t nowMs)
{
    return s_homeSet && ((nowMs - s_lastGpsMs) <= POS_GPS_TIMEOUT_MS);
}

const PositionEstimate_t *Position_Get(void)
{
    return &s_est;
}
