#ifndef IMU_H
#define IMU_H
#include <stdint.h>
#include "filter.h"


typedef struct {
    float q0, q1, q2, q3;   /* w, x, y, z */
} Quaternion_t;

typedef struct {
    float x, y, z;
} Vec3_t;

typedef struct {
    float roll;    /* độ, quanh trục X */
    float pitch;   /* độ, quanh trục Y */
    float yaw;     /* độ, quanh trục Z */
} Attitude_t;

typedef struct {
    Quaternion_t q;
    float rMat[3][3];
    Vec3_t gyroDrift;


    PT1Filter_t gyroFilterX, gyroFilterY, gyroFilterZ;
    PT1Filter_t accelFilterX, accelFilterY, accelFilterZ;


    Vec3_t gyroFiltered;
    Vec3_t accelFiltered;

    /* Mahony gains */
    float kp;   /* proportional - phản ứng nhanh  */
    float ki;   /* integral     - bù drift dài hạn */

    /* Output */
    Attitude_t attitude;
} IMU_t;

#define IMU_KP_ACC          2.0f   /* Kp accelerometer  */
#define IMU_KI_ACC          0.005f  /* Ki accelerometer  */
#define IMU_ROTATION_LPF    3.0f    /* Hz - PT1 cutoff   */
#define IMU_SPIN_RATE_LIMIT 20.0f   /* deg/s - giới hạn tích phân I */
#define IMU_ACC_NEARNESS    0.2f    /* 20% tolerance quanh 1G */
#define GRAVITY_CMSS        981.0f  
#define M_PIf               3.14159265358979f

void IMU_Init(IMU_t *imu, float kp, float ki, float dT);

void IMU_Update(IMU_t *imu, const Vec3_t *gyro, const Vec3_t *accel, float dT);

const Attitude_t *IMU_GetAttitude(const IMU_t *imu);

#endif