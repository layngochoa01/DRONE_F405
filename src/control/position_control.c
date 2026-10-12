#include "position_control.h"
#include "pid.h"
#include <math.h>

#define GRAVITY      9.80665f
#define DEG2RAD      (3.14159265f / 180.0f)

static PID_t s_velPidN, s_velPidW;
static float s_targetN = 0.0f, s_targetW = 0.0f;

static float constrainf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void PositionControl_Reset(const PositionEstimate_t *est)
{
    PID_Init(&s_velPidN, POS_VEL_KP, POS_VEL_KI, POS_VEL_KD);
    PID_Init(&s_velPidW, POS_VEL_KP, POS_VEL_KI, POS_VEL_KD);
    s_velPidN.prevMeasurement = est->velN;
    s_velPidW.prevMeasurement = est->velW;
    s_targetN = est->posN;
    s_targetW = est->posW;
}

static void velocityToTilt(const PositionEstimate_t *est, float velSpN, float velSpW,
                           float yawRad, float dT, float *rollSpRad, float *pitchSpRad)
{
    float accN = PID_Update(&s_velPidN, velSpN - est->velN, est->velN, dT, -POS_MAX_ACCEL, POS_MAX_ACCEL);
    float accW = PID_Update(&s_velPidW, velSpW - est->velW, est->velW, dT, -POS_MAX_ACCEL, POS_MAX_ACCEL);

    float c = cosf(yawRad), s = sinf(yawRad);
    float accFwd  =  accN * c + accW * s;
    float accLeft = -accN * s + accW * c;

    float maxTilt = POS_MAX_TILT_DEG * DEG2RAD;
    *pitchSpRad = constrainf( atan2f(accFwd,  GRAVITY), -maxTilt, maxTilt);
    *rollSpRad  = constrainf(-atan2f(accLeft, GRAVITY), -maxTilt, maxTilt);
}

void PositionControl_Update(const PositionEstimate_t *est, float yawRad, float dT,
                            float *rollSpRad, float *pitchSpRad)
{
    float velSpN = POS_KP * (s_targetN - est->posN);
    float velSpW = POS_KP * (s_targetW - est->posW);
    float velMag = sqrtf(velSpN * velSpN + velSpW * velSpW);
    if (velMag > POS_MAX_VEL) {
        velSpN *= POS_MAX_VEL / velMag;
        velSpW *= POS_MAX_VEL / velMag;
    }

    velocityToTilt(est, velSpN, velSpW, yawRad, dT, rollSpRad, pitchSpRad);
}

void PositionControl_UpdateVelocity(const PositionEstimate_t *est, float velFwd, float velLeft,
                                    float yawRad, float dT, float *rollSpRad, float *pitchSpRad)
{
    float c = cosf(yawRad), s = sinf(yawRad);
    float velSpN = velFwd * c - velLeft * s;
    float velSpW = velFwd * s + velLeft * c;

    velocityToTilt(est, velSpN, velSpW, yawRad, dT, rollSpRad, pitchSpRad);
}
