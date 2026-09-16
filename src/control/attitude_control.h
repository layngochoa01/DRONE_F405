#ifndef ATTITUDE_CONTROL_H
#define ATTITUDE_CONTROL_H

#include "../estimation/imu.h"   /* Quaternion_t, Vec3_t */

void AttitudeControl_ComputeError(Vec3_t *errorVec, const Quaternion_t *qSetpoint, const Quaternion_t *qCurrent);

void AttitudeControl_Update(Vec3_t *rateSetpoint, const Quaternion_t *qSetpoint, const Quaternion_t *qCurrent, float kpRoll, float kpPitch, float kpYaw);

#endif