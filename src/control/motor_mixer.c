#include "motor_mixer.h"
#include <math.h>

#define PWM_MIN      1000.0f
#define PWM_MAX      2000.0f
#define PWM_DISARMED 1000.0f
#define PWM_RANGE    (PWM_MAX - PWM_MIN) 

static bool s_armed = false;

static float constrainf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void MotorMixer_Init(void)
{
    s_armed = false;
}

void MotorMixer_SetArmed(bool armed)
{
    s_armed = armed;
}

bool MotorMixer_IsArmed(void)
{
    return s_armed;
}

void MotorMixer_Update(MotorOutput_t *out, float throttle, const Vec3_t *rateOutput)
{
    if (!s_armed) {
        for (int i = 0; i < MOTOR_COUNT; i++) out->pulseUs[i] = PWM_DISARMED;
        return;
    }

    float mix[MOTOR_COUNT];
    mix[0] =  rateOutput->y + rateOutput->x - rateOutput->z;   /* FL */
    mix[1] =  rateOutput->y - rateOutput->x + rateOutput->z;   /* FR */
    mix[2] = -rateOutput->y + rateOutput->x + rateOutput->z;   /* RL */
    mix[3] = -rateOutput->y - rateOutput->x - rateOutput->z;   /* RR */

    float mixMax = 0.0f, mixMin = 0.0f;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (mix[i] > mixMax) mixMax = mix[i];
        if (mix[i] < mixMin) mixMin = mix[i];
    }

    float throttleUs = PWM_MIN + throttle * PWM_RANGE;
    float headroomUp   = PWM_MAX - throttleUs;
    float headroomDown = throttleUs - PWM_MIN;

    float scale = 1.0f;
    if (mixMax * 1000.0f > headroomUp && mixMax > 0.0f) {
        scale = fminf(scale, headroomUp / (mixMax * 1000.0f));
    }
    if (mixMin * 1000.0f < -headroomDown && mixMin < 0.0f) {
        scale = fminf(scale, headroomDown / (-mixMin * 1000.0f));
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        float pulse = throttleUs + mix[i] * 1000.0f * scale;
        out->pulseUs[i] = constrainf(pulse, PWM_MIN, PWM_MAX);
    }
}