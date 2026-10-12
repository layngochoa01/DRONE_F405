#include "imu.h"
#include <math.h>
#include <stddef.h>
#include <stdbool.h>

static float vec3NormSq(const Vec3_t *v)
{
    return v->x*v->x + v->y*v->y + v->z*v->z;
}

static void vec3Normalize(Vec3_t *out, const Vec3_t *in)
{
    float n = sqrtf(vec3NormSq(in));
    if (n > 1e-6f) {
        out->x = in->x / n;
        out->y = in->y / n;
        out->z = in->z / n;
    }
}

static void vec3Cross(Vec3_t *out, const Vec3_t *a, const Vec3_t *b)
{
    out->x = a->y*b->z - a->z*b->y;
    out->y = a->z*b->x - a->x*b->z;
    out->z = a->x*b->y - a->y*b->x;
}

static float constrainf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void imuComputeRotationMatrix(IMU_t *imu)
{
    float q0 = imu->q.q0, q1 = imu->q.q1;
    float q2 = imu->q.q2, q3 = imu->q.q3;

    float q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;
    float q0q1 = q0*q1, q0q2 = q0*q2, q0q3 = q0*q3;
    float q1q2 = q1*q2, q1q3 = q1*q3, q2q3 = q2*q3;

    imu->rMat[0][0] = 1.0f - 2.0f*(q2q2 + q3q3);
    imu->rMat[0][1] = 2.0f*(q1q2 - q0q3);
    imu->rMat[0][2] = 2.0f*(q1q3 + q0q2);

    imu->rMat[1][0] = 2.0f*(q1q2 + q0q3);
    imu->rMat[1][1] = 1.0f - 2.0f*(q1q1 + q3q3);
    imu->rMat[1][2] = 2.0f*(q2q3 - q0q1);

    imu->rMat[2][0] = 2.0f*(q1q3 - q0q2);
    imu->rMat[2][1] = 2.0f*(q2q3 + q0q1);
    imu->rMat[2][2] = 1.0f - 2.0f*(q1q1 + q2q2);
}

static float imuComputeAccWeight(const IMU_t *imu, const Vec3_t *gyro, const Vec3_t *accel )
{
    (void)imu;
    float accMag = sqrtf(vec3NormSq(accel)) / GRAVITY_CMSS;
    float err    = fabsf(accMag - 1.0f);
    float w_near = (err < IMU_ACC_NEARNESS) ? (1.0f - err / IMU_ACC_NEARNESS) : 0.0f;

    float gyroMag = sqrtf(vec3NormSq(gyro));
    float rateLimit = (IMU_SPIN_RATE_LIMIT * M_PIf / 180.0f) * 4.0f;
    float w_rate  = 1.0f - constrainf(gyroMag / rateLimit, 0.0f, 1.0f);

    return w_near * w_rate;
}

static bool imuComputeMagError(const IMU_t *imu, const Vec3_t *mag, Vec3_t *vErr)
{
    if (vec3NormSq(mag) < 1e-12f) return false;

    Vec3_t m;
    vec3Normalize(&m, mag);

    float hx = imu->rMat[0][0]*m.x + imu->rMat[0][1]*m.y + imu->rMat[0][2]*m.z;
    float hy = imu->rMat[1][0]*m.x + imu->rMat[1][1]*m.y + imu->rMat[1][2]*m.z;
    float bx = sqrtf(hx*hx + hy*hy);
    if (bx < 1e-3f) return false;

    float ezEf = -(hy * bx);

    vErr->x = imu->rMat[2][0] * ezEf;
    vErr->y = imu->rMat[2][1] * ezEf;
    vErr->z = imu->rMat[2][2] * ezEf;
    return true;
}

static void imuMahonyUpdate(IMU_t *imu, const Vec3_t *gyro, const Vec3_t *accel, float accWeight, const Vec3_t *mag, float dT)
{
    Vec3_t vRotation = *gyro;

    Vec3_t vMagErr;
    if (mag && imuComputeMagError(imu, mag, &vMagErr)) {
        float spinLimit = (IMU_SPIN_RATE_LIMIT * M_PIf / 180.0f);
        if (imu->ki > 0.0f && vec3NormSq(gyro) < spinLimit * spinLimit) {
            imu->gyroDrift.z += vMagErr.z * imu->ki * dT;
        }
        vRotation.x += vMagErr.x * IMU_KP_MAG;
        vRotation.y += vMagErr.y * IMU_KP_MAG;
        vRotation.z += vMagErr.z * IMU_KP_MAG;
    }

    if (accel && accWeight > 0.001f) {
        Vec3_t vEstGravity = {imu->rMat[2][0], imu->rMat[2][1], imu->rMat[2][2] };

        Vec3_t vAcc= {0.0f, 0.0f, 0.0f};
        vec3Normalize(&vAcc, accel);

        Vec3_t vErr;
        vec3Cross(&vErr, &vAcc, &vEstGravity);
        float spinRateSq = vec3NormSq(gyro);
        float spinLimit  = (IMU_SPIN_RATE_LIMIT * M_PIf / 180.0f);

        if (imu->ki > 0.0f &&
            spinRateSq < spinLimit * spinLimit)
        {
            imu->gyroDrift.x += vErr.x * imu->ki * accWeight * dT;
            imu->gyroDrift.y += vErr.y * imu->ki * accWeight * dT;
            imu->gyroDrift.z += vErr.z * imu->ki * accWeight * dT;

            float i_limit = (M_PIf / 180.0f * 2.0f) * imu->kp * 0.5f;
            imu->gyroDrift.x = constrainf(imu->gyroDrift.x, -i_limit, i_limit);
            imu->gyroDrift.y = constrainf(imu->gyroDrift.y, -i_limit, i_limit);
            imu->gyroDrift.z = constrainf(imu->gyroDrift.z, -i_limit, i_limit);
        }

        vRotation.x += vErr.x * imu->kp * accWeight;
        vRotation.y += vErr.y * imu->kp * accWeight;
        vRotation.z += vErr.z * imu->kp * accWeight;
    }

    vRotation.x += imu->gyroDrift.x;
    vRotation.y += imu->gyroDrift.y;
    vRotation.z += imu->gyroDrift.z;

    Quaternion_IntegrateBodyRate(&imu->q, &vRotation, dT);

    imuComputeRotationMatrix(imu);
}

static void imuUpdateEulerAngles(IMU_t *imu)
{
    imu->attitude.roll  = atan2f(imu->rMat[2][1], imu->rMat[2][2]) * (180.0f / M_PIf);

    float sinPitch = constrainf(imu->rMat[2][0], -1.0f, 1.0f);
    imu->attitude.pitch = -asinf(sinPitch) * (180.0f / M_PIf);

    imu->attitude.yaw   = atan2f(imu->rMat[1][0], imu->rMat[0][0]) * (180.0f / M_PIf);

    if (imu->attitude.yaw < 0.0f)
        imu->attitude.yaw += 360.0f;

}

void IMU_Init(IMU_t *imu, float kp, float ki, float dT)
{
    (void)dT;
    imu->q = (Quaternion_t){ 1.0f, 0.0f, 0.0f, 0.0f };

    imu->gyroDrift = (Vec3_t){ 0.0f, 0.0f, 0.0f };

    imu->kp = kp;
    imu->ki = ki;

    imuComputeRotationMatrix(imu);
}

void IMU_Update(IMU_t *imu, const Vec3_t *gyro, const Vec3_t *accel, const Vec3_t *mag, float dT)
{
    float accWeight = imuComputeAccWeight(imu, gyro, accel);

    imuMahonyUpdate(imu, gyro, accel, accWeight, mag, dT);

    imuUpdateEulerAngles(imu);
}

const Attitude_t *IMU_GetAttitude(const IMU_t *imu)
{
    return &imu->attitude;
}

const Quaternion_t *IMU_GetQuaternion(const IMU_t *imu)
{
    return &imu->q;
}

void GetRotationMatrix(const IMU_t *imu, float rMat[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rMat[i][j] = imu->rMat[i][j];
        }
    }
}
