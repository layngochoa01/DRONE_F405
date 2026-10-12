#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float kt;
    float integrator;
    float prevMeasurement;
    float dAlpha;
    float dState;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd);
float PID_Update(PID_t *pid, float error, float measurement, float dT, float outMin, float outMax);
void PID_Reset(PID_t *pid);
void PID_SetDFilter(PID_t *pid, float cutoffHz, float dT);

#endif
