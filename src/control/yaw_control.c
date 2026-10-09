#include "yaw_control.h"

#define M_PIf_YAW   3.14159265358979f
#define DEG2RAD     (M_PIf_YAW / 180.0f)

static float s_targetYaw = 0.0f;

static float wrapPi(float a)
{
    while (a >  M_PIf_YAW) a -= 2.0f * M_PIf_YAW;
    while (a < -M_PIf_YAW) a += 2.0f * M_PIf_YAW;
    return a;
}

void YawControl_Reset(float currentYawRad)
{
    s_targetYaw = wrapPi(currentYawRad);
}

float YawControl_Update(float stickYaw, float currentYawRad, float dT)
{
    /* Gạt phải = quay phải = yaw giảm */
    s_targetYaw = wrapPi(s_targetYaw - stickYaw * YAW_MAX_RATE_DEGS * DEG2RAD * dT);

    /* Không để target chạy quá xa thực tế (tránh "tích lũy" khi drone quay không kịp) */
    float lead    = wrapPi(s_targetYaw - currentYawRad);
    float maxLead = YAW_MAX_LEAD_DEG * DEG2RAD;
    if (lead >  maxLead) s_targetYaw = wrapPi(currentYawRad + maxLead);
    if (lead < -maxLead) s_targetYaw = wrapPi(currentYawRad - maxLead);

    return s_targetYaw;
}

float YawControl_GetTarget(void)
{
    return s_targetYaw;
}
