#include "imu.h"
#include <math.h>
#include <stddef.h>

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

static float imuComputeAccWeight(const IMU_t *imu, const Vec3_t *accel)
{
    /* --- Weight 1: Nearness ---
     * Bell curve quanh 1G, tolerance ±20%
     * |accel| = 1.0G → weight = 1.0
     * |accel| = 1.2G → weight = 0.0  */
    float accMag = sqrtf(vec3NormSq(accel)) / GRAVITY_CMSS;
    float err    = fabsf(accMag - 1.0f);
    float w_near = (err < IMU_ACC_NEARNESS)
                 ? (1.0f - err / IMU_ACC_NEARNESS)
                 : 0.0f;

    /* --- Weight 2: Rate ---
     * Dùng filtered gyro (PT1 3Hz) để tránh spike
     * Quay nhanh → lực ly tâm làm accel sai → giảm weight */
    float gyroMag = sqrtf(vec3NormSq(&imu->gyroFiltered));
    float rateLimit = (IMU_SPIN_RATE_LIMIT * M_PIf / 180.0f) * 4.0f;
    float w_rate  = 1.0f - constrainf(gyroMag / rateLimit, 0.0f, 1.0f);

    return w_near * w_rate;
}

/* =========================================================
 * PRIVATE - Mahony AHRS core
 *
 * Step 1: Tính error giữa gravity đo (accel) và gravity ước tính
 *         (rMat[2]) bằng cross product
 * Step 2: PI controller → correction vào gyro
 * Step 3: Tích phân quaternion
 * Step 4: Renormalize (Newton approximation)
 * Step 5: Cập nhật rMat
 * ========================================================= */
static void imuMahonyUpdate(IMU_t *imu, const Vec3_t *gyro, const Vec3_t *accel, float accWeight, float dT)
{
    Vec3_t vRotation = *gyro;   /* bắt đầu từ gyro raw */

    /* ── Accel correction (roll/pitch) ─────────────────── */
    if (accel && accWeight > 0.001f) {

        /* Gravity ước tính = hàng 3 của rMat
         * (tối ưu: không cần quaternionRotateVector) */
        Vec3_t vEstGravity = {
            imu->rMat[2][0],
            imu->rMat[2][1],
            imu->rMat[2][2]
        };

        /* Normalize accel đo được */
        Vec3_t vAcc= {0.0f, 0.0f, 0.0f};
        vec3Normalize(&vAcc, accel);

        /* Error = cross(accel_measured, gravity_estimated)
         * Cross product cho biết: lệch bao nhiêu, chiều nào
         * Magnitude ≈ sin(góc lệch) ≈ góc lệch khi nhỏ      */
        Vec3_t vErr;
        vec3Cross(&vErr, &vAcc, &vEstGravity);

        /* ── Integral (Ki) - bù drift dài hạn ─────────── */
        float spinRateSq = vec3NormSq(gyro);
        float spinLimit  = (IMU_SPIN_RATE_LIMIT * M_PIf / 180.0f);

        if (imu->ki > 0.0f &&
            spinRateSq < spinLimit * spinLimit)
        {
            /* Tích lũy drift estimate */
            imu->gyroDrift.x += vErr.x * imu->ki * accWeight * dT;
            imu->gyroDrift.y += vErr.y * imu->ki * accWeight * dT;
            imu->gyroDrift.z += vErr.z * imu->ki * accWeight * dT;

            /* Anti-windup: giới hạn drift */
            float i_limit = (M_PIf / 180.0f * 2.0f) * imu->kp * 0.5f;
            imu->gyroDrift.x = constrainf(imu->gyroDrift.x, -i_limit, i_limit);
            imu->gyroDrift.y = constrainf(imu->gyroDrift.y, -i_limit, i_limit);
            imu->gyroDrift.z = constrainf(imu->gyroDrift.z, -i_limit, i_limit);
        }

        /* ── Proportional (Kp) - correction tức thì ──── */
        vRotation.x += vErr.x * imu->kp * accWeight;
        vRotation.y += vErr.y * imu->kp * accWeight;
        vRotation.z += vErr.z * imu->kp * accWeight;
    }

    /* ── Áp dụng drift estimate (I term) ───────────────── */
    vRotation.x += imu->gyroDrift.x;
    vRotation.y += imu->gyroDrift.y;
    vRotation.z += imu->gyroDrift.z;

    /* ── Tích phân quaternion ───────────────────────────── */
    /* vTheta = vRotation × dt/2 (half-angle vector) */
    float tx = vRotation.x * 0.5f * dT;
    float ty = vRotation.y * 0.5f * dT;
    float tz = vRotation.z * 0.5f * dT;

    float thetaSq = tx*tx + ty*ty + tz*tz;

    float dq0, dq1, dq2, dq3;

    if (thetaSq * thetaSq < 24.0e-6f) {
        /* Góc nhỏ: Taylor series
         * sin(x)/x ≈ 1 - x²/6
         * cos(x)   ≈ 1 - x²/2
         * Tránh sin/cos tốn CPU, chính xác đến O(x⁴) */
        float s = 1.0f - thetaSq / 6.0f;
        dq0 = 1.0f - thetaSq / 2.0f;
        dq1 = tx * s;
        dq2 = ty * s;
        dq3 = tz * s;
    } else {
        /* Góc lớn: chính xác hoàn toàn */
        float theta = sqrtf(thetaSq);
        float s     = sinf(theta) / theta;
        dq0 = cosf(theta);
        dq1 = tx * s;
        dq2 = ty * s;
        dq3 = tz * s;
    }

    /* Nhân quaternion: orientation = orientation × deltaQ */
    float q0 = imu->q.q0, q1 = imu->q.q1;
    float q2 = imu->q.q2, q3 = imu->q.q3;

    imu->q.q0 = q0*dq0 - q1*dq1 - q2*dq2 - q3*dq3;
    imu->q.q1 = q0*dq1 + q1*dq0 + q2*dq3 - q3*dq2;
    imu->q.q2 = q0*dq2 - q1*dq3 + q2*dq0 + q3*dq1;
    imu->q.q3 = q0*dq3 + q1*dq2 - q2*dq1 + q3*dq0;

    /* ── Renormalize (Newton approximation) ─────────────── */
    /* 1/sqrt(x) ≈ (3-x)/2 khi x ≈ 1
     * Tránh sqrt() và division tốn CPU                      */
    float normSq = imu->q.q0*imu->q.q0 + imu->q.q1*imu->q.q1
                 + imu->q.q2*imu->q.q2 + imu->q.q3*imu->q.q3;
    float scale  = (3.0f - normSq) * 0.5f;
    imu->q.q0 *= scale;
    imu->q.q1 *= scale;
    imu->q.q2 *= scale;
    imu->q.q3 *= scale;

    /* ── Cập nhật rotation matrix ───────────────────────── */
    imuComputeRotationMatrix(imu);
}

/* =========================================================
 * PRIVATE - Euler angles từ rMat
 *
 * roll  = atan2(rMat[2][1], rMat[2][2])
 * pitch = 90° - acos(-rMat[2][0])
 * yaw   = -atan2(rMat[1][0], rMat[0][0])
 * ========================================================= */
static void imuUpdateEulerAngles(IMU_t *imu)
{
    imu->attitude.roll  = atan2f(imu->rMat[2][1], imu->rMat[2][2]) * (180.0f / M_PIf);

    imu->attitude.pitch = -asinf(imu->rMat[2][0]) * (180.0f / M_PIf);

    imu->attitude.yaw   = atan2f(imu->rMat[1][0], imu->rMat[0][0]) * (180.0f / M_PIf);

    /* Chuẩn hóa yaw về [0, 360) */
    if (imu->attitude.yaw < 0.0f)
        imu->attitude.yaw += 360.0f;
}

/* =========================================================
 * PUBLIC API
 * ========================================================= */
void IMU_Init(IMU_t *imu, float kp, float ki, float dT)
{
    /* Quaternion = identity (không nghiêng) */
    imu->q = (Quaternion_t){ 1.0f, 0.0f, 0.0f, 0.0f };

    /* Drift estimate = 0 */
    imu->gyroDrift = (Vec3_t){ 0.0f, 0.0f, 0.0f };

    /* Gains */
    imu->kp = kp;
    imu->ki = ki;

    /* PT1 filters @ 3Hz */
    pt1FilterInit(&imu->gyroFilterX,  IMU_ROTATION_LPF, dT);
    pt1FilterInit(&imu->gyroFilterY,  IMU_ROTATION_LPF, dT);
    pt1FilterInit(&imu->gyroFilterZ,  IMU_ROTATION_LPF, dT);
    pt1FilterInit(&imu->accelFilterX, IMU_ROTATION_LPF, dT);
    pt1FilterInit(&imu->accelFilterY, IMU_ROTATION_LPF, dT);
    pt1FilterInit(&imu->accelFilterZ, IMU_ROTATION_LPF, dT);

    /* Rotation matrix = identity */
    imuComputeRotationMatrix(imu);
}

void IMU_Update(IMU_t *imu, const Vec3_t *gyro, const Vec3_t *accel, float dT)
{
    imu->gyroFiltered.x = pt1FilterApply(&imu->gyroFilterX,  gyro->x);
    imu->gyroFiltered.y = pt1FilterApply(&imu->gyroFilterY,  gyro->y);
    imu->gyroFiltered.z = pt1FilterApply(&imu->gyroFilterZ,  gyro->z);

    imu->accelFiltered.x = pt1FilterApply(&imu->accelFilterX, accel->x);
    imu->accelFiltered.y = pt1FilterApply(&imu->accelFilterY, accel->y);
    imu->accelFiltered.z = pt1FilterApply(&imu->accelFilterZ, accel->z);

    float accWeight = imuComputeAccWeight(imu, accel);

    imuMahonyUpdate(imu, gyro, accel, accWeight, dT);

    imuUpdateEulerAngles(imu);
}

const Attitude_t *IMU_GetAttitude(const IMU_t *imu)
{
    return &imu->attitude;
}