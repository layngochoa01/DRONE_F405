#include "attitude_control.h"
#include "../estimation/quaternion.h"

void AttitudeControl_ComputeError(Vec3_t *errorVec, const Quaternion_t *qSetpoint, const Quaternion_t *qCurrent)
{
    Quaternion_t qCurrentConj, qErr;

    Quaternion_Conjugate(&qCurrentConj, qCurrent);
    Quaternion_Multiply(&qErr, &qCurrentConj, qSetpoint);   /* error tính trong body frame */

    /* shortest-path: quaternion q và -q biểu diễn cùng 1 attitude,
       nếu không xử lý PID có thể chọn quay đường vòng dài thay vì đường ngắn */
    if (qErr.q0 < 0.0f) {
        qErr.q0 = -qErr.q0;
        qErr.q1 = -qErr.q1;
        qErr.q2 = -qErr.q2;
        qErr.q3 = -qErr.q3;
    }

    /* với góc lỗi nhỏ, phần vector (q1,q2,q3) xấp xỉ tỉ lệ thuận với góc lỗi từng trục body-frame */
    errorVec->x = 2.0f * qErr.q1;
    errorVec->y = 2.0f * qErr.q2;
    errorVec->z = 2.0f * qErr.q3;
}

void AttitudeControl_Update(Vec3_t *rateSetpoint, const Quaternion_t *qSetpoint, const Quaternion_t *qCurrent,
                             float kpRoll, float kpPitch, float kpYaw)
{
    Vec3_t errorVec;
    AttitudeControl_ComputeError(&errorVec, qSetpoint, qCurrent);

    /* outer loop attitude thường chỉ cần P - rate loop bên dưới đã có I/D để triệt sai số */
    rateSetpoint->x = kpRoll  * errorVec.x;
    rateSetpoint->y = kpPitch * errorVec.y;
    rateSetpoint->z = kpYaw   * errorVec.z;
}