#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H
#include <stdbool.h> 
#include "../estimation/imu.h"  

#define MOTOR_COUNT 4

typedef struct {
    float pulseUs[MOTOR_COUNT];   /* 1000-2000 us */
} MotorOutput_t;

void MotorMixer_Init(void);
void MotorMixer_Update(MotorOutput_t *out, float throttle, const Vec3_t *rateOutput);
bool MotorMixer_IsArmed(void);
void MotorMixer_SetArmed(bool armed);

#endif