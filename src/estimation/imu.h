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

#define IMU_KP_ACC          2.0f   /* Kp accelerometer  */
#define IMU_KI_ACC          0.005f  /* Ki accelerometer  */
#define IMU_SPIN_RATE_LIMIT 20.0f   /* deg/s - giới hạn tích phân I */
#define IMU_ACC_NEARNESS    0.2f    /* 20% tolerance quanh 1G */
#define GRAVITY_CMSS        981.0f  
#define M_PIf               3.14159265358979f

void IMU_Init(IMU_t *imu, float kp, float ki, float dT);

void IMU_Update(IMU_t *imu, const Vec3_t *gyro, const Vec3_t *accel, float dT);

const Attitude_t *IMU_GetAttitude(const IMU_t *imu);

const Quaternion_t *IMU_GetQuaternion(const IMU_t *imu);

#endif