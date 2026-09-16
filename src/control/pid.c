#include "pid.h"

static float constrainf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float iLimit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    if (ki > 1e-6f && kp > 1e-6f) {
        float Ti = kp / ki;
        float Td = kd / kp;
        pid->kt = 2.0f / (Ti + Td);
    } else {
        pid->kt = 0.0f;
    }
    pid->integrator = 0.0f;
    pid->prevMeasurement = 0.0f;
}

float PID_Update(PID_t *pid, float error, float measurement, float dT, float outMin, float outMax)
{
    float p = pid->kp * error;
    float d = -pid->kd * (measurement - pid->prevMeasurement) / dT;
    pid->prevMeasurement = measurement;

    float outVal = p + pid->integrator + d;
    float outConstrained = constrainf(outVal, outMin, outMax);

    /* Back-calculation anti-windup */
    float backCalc = outConstrained - outVal;
    if ((backCalc > 0.0f) == (pid->integrator > 0.0f)) {
        backCalc = 0.0f;   /* chỉ cho co lại, không đẩy ngược hướng */
    }

    pid->integrator += (error * pid->ki * dT) + (backCalc * pid->kt * dT);

    return outConstrained;
}

void PID_Reset(PID_t *pid)
{
    pid->integrator = 0.0f;
    pid->prevMeasurement = 0.0f;
}