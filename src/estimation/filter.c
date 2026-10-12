#include "filter.h"
#include <math.h>

#define M_PIf 3.14159265358979f

void pt1FilterInit(PT1Filter_t *f, float f_cut, float dT)
{
    f->RC    = 1.0f / (2.0f * M_PIf * f_cut);
    f->alpha = dT / (f->RC + dT);
    f->state = 0.0f;
}

float pt1FilterApply(PT1Filter_t *f, float input)
{
    f->state = f->state + f->alpha * (input - f->state);
    return f->state;
}

#define ACCEL_CUTOFF_HZ  20.0f
#define GYRO_CUTOFF_HZ   100.0f

void IMU_FilterInit(IMUFilter_t *f, float dt)
{
    pt1FilterInit(&f->ax, ACCEL_CUTOFF_HZ, dt);
    pt1FilterInit(&f->ay, ACCEL_CUTOFF_HZ, dt);
    pt1FilterInit(&f->az, ACCEL_CUTOFF_HZ, dt);

    pt1FilterInit(&f->gx, GYRO_CUTOFF_HZ, dt);
    pt1FilterInit(&f->gy, GYRO_CUTOFF_HZ, dt);
    pt1FilterInit(&f->gz, GYRO_CUTOFF_HZ, dt);
}

void IMU_FilterApply(IMUFilter_t *f, ICM42605_Data *data)
{
    data->accel_x = pt1FilterApply(&f->ax, data->accel_x);
    data->accel_y = pt1FilterApply(&f->ay, data->accel_y);
    data->accel_z = pt1FilterApply(&f->az, data->accel_z);

    data->gyro_x = pt1FilterApply(&f->gx, data->gyro_x);
    data->gyro_y = pt1FilterApply(&f->gy, data->gyro_y);
    data->gyro_z = pt1FilterApply(&f->gz, data->gyro_z);
}
