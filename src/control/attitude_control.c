#include "attitude_control.h"
#include "../estimation/quaternion.h"

void AttitudeControl_ComputeError(Vec3_t *errorVec, const Quaternion_t *qSetpoint, const Quaternion_t *qCurrent)
{
    Quaternion_t qCurrentConj, qErr;

    Quaternion_Conjugate(&qCurrentConj, qCurrent);
    Quaternion_Multiply(&qErr, &qCurrentConj, qSetpoint);   
    if (qErr.q0 < 0.0f) {
        qErr.q0 = -qErr.q0;
        qErr.q1 = -qErr.q1;
        qErr.q2 = -qErr.q2;
        qErr.q3 = -qErr.q3;
    }

    errorVec->x = 2.0f * qErr.q1;
    errorVec->y = 2.0f * qErr.q2;
    errorVec->z = 2.0f * qErr.q3;
}

void AttitudeControl_Update(Vec3_t *rateSetpoint, const Quaternion_t *qSetpoint, const Quaternion_t *qCurrent, float kpRoll, float kpPitch, float kpYaw)
{
    Vec3_t errorVec;
    AttitudeControl_ComputeError(&errorVec, qSetpoint, qCurrent);

    rateSetpoint->x = kpRoll  * errorVec.x;
    rateSetpoint->y = kpPitch * errorVec.y;
    rateSetpoint->z = kpYaw   * errorVec.z;
}