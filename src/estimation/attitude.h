#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "imu.h"
#include "../sensors/icm42605.h"

void Attitude_Init(float dT);
/* mag: body frame, NULL = chỉ dùng gyro + accel */
void Attitude_Update(const ICM42605_Data *data, const Vec3_t *mag, float dT);
const Attitude_t *Attitude_Get(void);
const Quaternion_t *Quaternion_Get(void);
void Attitude_GetRotationMatrix(float rMat[3][3]);
float Attitude_GetYawRad(void);
#endif