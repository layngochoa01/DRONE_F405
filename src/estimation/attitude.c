#include "attitude.h"
#include <math.h>

static IMU_t s_imu;

void Attitude_Init(float dT)
{
    IMU_Init(&s_imu, IMU_KP_ACC, IMU_KI_ACC, dT);
}

void Attitude_Update(const ICM42605_Data *data, float dT)
{
    Vec3_t gyro = {
        data->gyro_x * (M_PIf / 180.0f),
        data->gyro_y * (M_PIf / 180.0f),
        data->gyro_z * (M_PIf / 180.0f)
    };

    Vec3_t accel = {
        data->accel_x * 100.0f,   
        data->accel_y * 100.0f,
        data->accel_z * 100.0f
    };

    IMU_Update(&s_imu, &gyro, &accel, dT);
}

const Attitude_t *Attitude_Get(void)
{
    return IMU_GetAttitude(&s_imu);
}

const Quaternion_t *Quaternion_Get(void)
{
     return IMU_GetQuaternion(&s_imu);
}

void Attitude_GetRotationMatrix(float rMat[3][3])
{
    GetRotationMatrix(&s_imu, rMat);
}