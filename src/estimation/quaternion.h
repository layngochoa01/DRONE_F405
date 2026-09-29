#ifndef QUATERNION_H
#define QUATERNION_H

#include"types.h"

typedef struct {
    float q0, q1, q2, q3;  
} Quaternion_t;

void Quaternion_Multiply(Quaternion_t *out, const Quaternion_t *a, const Quaternion_t *b);
void Quaternion_Conjugate(Quaternion_t *out, const Quaternion_t *q);
void Quaternion_Normalize(Quaternion_t *q);
void Quaternion_FromEuler(Quaternion_t *q, float rollRad, float pitchRad, float yawRad);
void Quaternion_IntegrateBodyRate(Quaternion_t *q, const Vec3_t *rate, float dT);


#endif
