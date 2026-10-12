#include "rate_control.h"
#include "pid.h"

static PID_t s_pidRoll, s_pidPitch, s_pidYaw;

void RateControl_Init(float loopDt)
{
    PID_Init(&s_pidRoll,  0.6f, 0.02f, 0.01f);
    PID_Init(&s_pidPitch, 0.6f, 0.02f, 0.01f);
    PID_Init(&s_pidYaw,   1.0f, 0.03f, 0.0f);
    PID_SetDFilter(&s_pidRoll,  RATE_D_LPF_HZ, loopDt);
    PID_SetDFilter(&s_pidPitch, RATE_D_LPF_HZ, loopDt);
}

void RateControl_Reset(const Vec3_t *gyroDegS)
{
    PID_Reset(&s_pidRoll);
    PID_Reset(&s_pidPitch);
    PID_Reset(&s_pidYaw);
    s_pidRoll.prevMeasurement  = gyroDegS->x;
    s_pidPitch.prevMeasurement = gyroDegS->y;
    s_pidYaw.prevMeasurement   = gyroDegS->z;
}

void RateControl_Update(Vec3_t *output, const Vec3_t *rateSetpointDegS, const Vec3_t *gyroDegS, float dT)
{
    float errRoll  = rateSetpointDegS->x - gyroDegS->x;
    float errPitch = rateSetpointDegS->y - gyroDegS->y;
    float errYaw   = rateSetpointDegS->z - gyroDegS->z;
    output->x = PID_Update(&s_pidRoll,  errRoll,  gyroDegS->x, dT, -RATE_OUT_LIMIT_RP,  RATE_OUT_LIMIT_RP);
    output->y = PID_Update(&s_pidPitch, errPitch, gyroDegS->y, dT, -RATE_OUT_LIMIT_RP,  RATE_OUT_LIMIT_RP);
    output->z = PID_Update(&s_pidYaw,   errYaw,   gyroDegS->z, dT, -RATE_OUT_LIMIT_YAW, RATE_OUT_LIMIT_YAW);
}
