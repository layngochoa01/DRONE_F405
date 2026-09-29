#ifndef RATE_CONTROL_H
#define RATE_CONTROL_H

#include "../estimation/imu.h"

void RateControl_Init(void);
void RateControl_Update(Vec3_t *output, const Vec3_t *rateSetpointDegS, const Vec3_t *gyroDegS, float dT);

#endif