#ifndef FILTER_H
#define FILTER_H
#include "../sensors/icm42605.h"

typedef struct {
    float state;  
    float alpha;   
    float RC;      
} PT1Filter_t;

void  pt1FilterInit(PT1Filter_t *f, float f_cut, float dT);

void  pt1FilterUpdateCutoff(PT1Filter_t *f, float f_cut, float dT);

float pt1FilterApply(PT1Filter_t *f, float input);

void  pt1FilterReset(PT1Filter_t *f, float value);

typedef struct
{
    PT1Filter_t ax;
    PT1Filter_t ay;
    PT1Filter_t az;

    PT1Filter_t gx;
    PT1Filter_t gy;
    PT1Filter_t gz;
} IMUFilter_t;

void IMU_FilterInit(IMUFilter_t *f, float dt);

void IMU_FilterApply(IMUFilter_t *f, ICM42605_Data *data);

#endif