#include "rate_control.h"
#include "pid.h"

static PID_t s_pidRoll, s_pidPitch, s_pidYaw;

void RateControl_Init(void)
{
    PID_Init(&s_pidRoll,  0.6f, 0.02f, 0.01f);
    PID_Init(&s_pidPitch, 0.6f, 0.02f, 0.01f);
    PID_Init(&s_pidYaw,   1.0f, 0.03f, 0.0f);
}

void RateControl_Update(Vec3_t *output, const Vec3_t *rateSetpointDegS, const Vec3_t *gyroDegS, float dT)
{
    float errRoll  = rateSetpointDegS->x - gyroDegS->x;
    float errPitch = rateSetpointDegS->y - gyroDegS->y;
    float errYaw   = rateSetpointDegS->z - gyroDegS->z;
    output->x = PID_Update(&s_pidRoll,  errRoll,  gyroDegS->x, dT, -500.0f, 500.0f);
    output->y = PID_Update(&s_pidPitch, errPitch, gyroDegS->y, dT, -500.0f, 500.0f);
    output->z = PID_Update(&s_pidYaw,   errYaw,   gyroDegS->z, dT, -500.0f, 500.0f);
}