#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float kt;             
    float integrator;
    float prevMeasurement;
    float dAlpha;         /* 0 = không lọc khâu D */
    float dState;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd);
float PID_Update(PID_t *pid, float error, float measurement, float dT, float outMin, float outMax);
void PID_Reset(PID_t *pid);
/* Bật lọc thông thấp PT1 cho khâu D (đạo hàm phép đo) */
void PID_SetDFilter(PID_t *pid, float cutoffHz, float dT);

#endif