#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float integrator;
    float prevMeasurement;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float iLimit);
float PID_Update(PID_t *pid, float error, float measurement, float dT);
void PID_Reset(PID_t *pid);

#endif