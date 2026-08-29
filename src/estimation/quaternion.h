#ifndef QUATERNION_H
#define QUATERNION_H

#include"types.h"

typedef struct {
    float q0, q1, q2, q3;  
} Quaternion_t;

void multiply(Quaternion_t *out, const Quaternion_t *a, const Quaternion_t *b);
void conjugate(Quaternion_t *out, const Quaternion_t *q);
void normalize(Quaternion_t *q);
void fromEuler(Quaternion_t *q, float rollRad, float pitchRad, float yawRad);
void integrateBodyRate(Quaternion_t *q, const Vec3_t *rate, float dT);


#endif
