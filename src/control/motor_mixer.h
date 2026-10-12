#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include <stdbool.h>
#include "../estimation/imu.h"
#include "../drivers/pwm_out.h"

typedef struct {
    float pulseUs[MOTOR_COUNT];
} MotorOutput_t;

void MotorMixer_Init(void);
void MotorMixer_Update(MotorOutput_t *out, float throttle, const Vec3_t *torque);
void MotorMixer_SetArmed(bool armed);

#endif
