#include "quaternion.h"
#include<math.h>

void multiply(Quaternion_t *out, const Quaternion_t *a, const Quaternion_t *b){
    Quaternion_t r;
    r.q0 = a->q0*b->q0 - a->q1*b->q1 - a->q2*b->q2 - a->q3*b->q3;
    r.q1 = a->q0*b->q1 + a->q1*b->q0 + a->q2*b->q3 - a->q3*b->q2;
    r.q2 = a->q0*b->q2 - a->q1*b->q3 + a->q2*b->q0 + a->q3*b->q1;
    r.q3 = a->q0*b->q3 + a->q1*b->q2 - a->q2*b->q1 + a->q3*b->q0;
    *out = r; 

}

void conjugate(Quaternion_t *out, const Quaternion_t *q){
    out->q0 =  q->q0;
    out->q1 = -q->q1;
    out->q2 = -q->q2;
    out->q3 = -q->q3;
}

void normalize(Quaternion_t *q){
    float normSq = q->q0*q->q0 + q->q1*q->q1 + q->q2*q->q2 + q->q3*q->q3;
    float scale  = (3.0f - normSq) * 0.5f;   // xấp xỉ 1/sqrt(normSq)
    q->q0 *= scale;
    q->q1 *= scale;
    q->q2 *= scale;
    q->q3 *= scale;
}

void fromEuler(Quaternion_t *q, float rollRad, float pitchRad, float yawRad){
    float cr = cosf(rollRad * 0.5f),  sr = sinf(rollRad * 0.5f);
    float cp = cosf(pitchRad * 0.5f), sp = sinf(pitchRad * 0.5f);
    float cy = cosf(yawRad * 0.5f),   sy = sinf(yawRad * 0.5f);

    q->q0 = cr*cp*cy + sr*sp*sy;
    q->q1 = sr*cp*cy - cr*sp*sy;
    q->q2 = cr*sp*cy + sr*cp*sy;
    q->q3 = cr*cp*sy - sr*sp*cy;
}

void integrateBodyRate(Quaternion_t *q, const Vec3_t *rate, float dT){
    float tx = rate->x * 0.5f * dT;
    float ty = rate->y * 0.5f * dT;
    float tz = rate->z * 0.5f * dT;

    float thetaSq = tx*tx + ty*ty + tz*tz;
    Quaternion_t dq;

    if (thetaSq * thetaSq < 24.0e-6f) {
        float s = 1.0f - thetaSq / 6.0f;
        dq.q0 = 1.0f - thetaSq / 2.0f;
        dq.q1 = tx * s;
        dq.q2 = ty * s;
        dq.q3 = tz * s;
    } else {
        float theta = sqrtf(thetaSq);
        float s     = sinf(theta) / theta;
        dq.q0 = cosf(theta);
        dq.q1 = tx * s;
        dq.q2 = ty * s;
        dq.q3 = tz * s;
    }

    multiply(q, q, &dq);
    normalize(q);
}

