#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "imu.h"
#include "../sensors/icm42605.h"

void Attitude_Init(float dT);

void Attitude_Update(const ICM42605_Data *data, float dT);

const Attitude_t *Attitude_Get(void);

const Quaternion_t *Quaternion_Get(void);
#endif