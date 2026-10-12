#ifndef RATE_CONTROL_H
#define RATE_CONTROL_H

#include "../estimation/imu.h"

#define RATE_OUT_LIMIT_RP    400.0f
#define RATE_OUT_LIMIT_YAW   200.0f

#define RATE_D_LPF_HZ        80.0f

void RateControl_Init(float loopDt);
void RateControl_Reset(const Vec3_t *gyroDegS);
void RateControl_Update(Vec3_t *output, const Vec3_t *rateSetpointDegS, const Vec3_t *gyroDegS, float dT);

#endif
