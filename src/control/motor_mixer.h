#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include <stdbool.h> 
#include "../estimation/imu.h"  
#include "../drivers/pwm_out.h"   /* MOTOR_COUNT, MOTOR_MIN_US, MOTOR_MAX_US */

typedef struct {
    float pulseUs[MOTOR_COUNT];   /* 1000-2000 us, theo thứ tự chân pwm_out */
} MotorOutput_t;

void MotorMixer_Init(void);
/* throttle: 0..1, torque: mô-men roll/pitch/yaw (µs) từ vòng rate */
void MotorMixer_Update(MotorOutput_t *out, float throttle, const Vec3_t *torque);
bool MotorMixer_IsArmed(void);
void MotorMixer_SetArmed(bool armed);

#endif
