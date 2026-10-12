#include "motor_mixer.h"
#include <math.h>

#define PWM_MIN      ((float)MOTOR_MIN_US)
#define PWM_MAX      ((float)MOTOR_MAX_US)
#define PWM_DISARMED ((float)MOTOR_MIN_US)
#define PWM_RANGE    (PWM_MAX - PWM_MIN)

static const float s_mix[MOTOR_COUNT][3] = {
    { -1.0f,  -1.0f,  -1.0f },
    { -1.0f,  +1.0f,  +1.0f },
    { +1.0f,  +1.0f,  -1.0f },
    { +1.0f,  -1.0f,  +1.0f },
};

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

void MotorMixer_Update(MotorOutput_t *out, float throttle, const Vec3_t *torque)
{
    if (!s_armed) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) out->pulseUs[i] = PWM_DISARMED;
        return;
    }

    float mix[MOTOR_COUNT];
    float mixMax = 0.0f, mixMin = 0.0f;
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        mix[i] = s_mix[i][0] * torque->x + s_mix[i][1] * torque->y + s_mix[i][2] * torque->z;
        if (mix[i] > mixMax) mixMax = mix[i];
        if (mix[i] < mixMin) mixMin = mix[i];
    }

    float throttleUs   = PWM_MIN + constrainf(throttle, 0.0f, 1.0f) * PWM_RANGE;
    float headroomUp   = PWM_MAX - throttleUs;
    float headroomDown = throttleUs - PWM_MIN;

    float scale = 1.0f;
    if (mixMax > headroomUp)    scale = fminf(scale, headroomUp   / mixMax);
    if (-mixMin > headroomDown) scale = fminf(scale, headroomDown / -mixMin);

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        out->pulseUs[i] = constrainf(throttleUs + mix[i] * scale, PWM_MIN, PWM_MAX);
    }
}
