#ifndef IMU_H
#define IMU_H

#include "quaternion.h"

typedef struct {
    float roll;
    float pitch;
    float yaw;
} Attitude_t;

typedef struct {
    Quaternion_t q;
    float rMat[3][3];
    Vec3_t gyroDrift;
    float kp;
    float ki;
    Attitude_t attitude;
} IMU_t;

#define IMU_KP_ACC          2.0f
#define IMU_KI_ACC          0.005f
#define IMU_KP_MAG          1.0f
#define IMU_SPIN_RATE_LIMIT 20.0f
#define IMU_ACC_NEARNESS    0.2f
#define GRAVITY_CMSS        981.0f
#define M_PIf               3.14159265358979f

void IMU_Init(IMU_t *imu, float kp, float ki, float dT);

void IMU_Update(IMU_t *imu, const Vec3_t *gyro, const Vec3_t *accel, const Vec3_t *mag, float dT);

const Attitude_t *IMU_GetAttitude(const IMU_t *imu);

const Quaternion_t *IMU_GetQuaternion(const IMU_t *imu);

void GetRotationMatrix(const IMU_t *imu, float rMat[3][3]);

#endif
