#include "filter.h"
#include <math.h>

/* =========================================================
 * PT1 - Low Pass Filter bậc 1
 *
 * Tương đương mạch RC:
 *   - Tần số < f_cut: cho qua
 *   - Tần số > f_cut: suy giảm -20dB/decade
 *
 * alpha nhỏ (f_cut thấp) → output thay đổi chậm → lọc mạnh
 * alpha lớn (f_cut cao)  → output bám input nhanh → lọc nhẹ
 * ========================================================= */

#define M_PIf 3.14159265358979f

static float computeAlpha(float f_cut, float dT)
{
    float RC = 1.0f / (2.0f * M_PIf * f_cut);
    return dT / (RC + dT);
}

void pt1FilterInit(PT1Filter_t *f, float f_cut, float dT)
{
    f->RC    = 1.0f / (2.0f * M_PIf * f_cut);
    f->alpha = dT / (f->RC + dT);
    f->state = 0.0f;
}

void pt1FilterUpdateCutoff(PT1Filter_t *f, float f_cut, float dT)
{
    f->RC    = 1.0f / (2.0f * M_PIf * f_cut);
    f->alpha = dT / (f->RC + dT);
}

float pt1FilterApply(PT1Filter_t *f, float input)
{
    /* y[n] = y[n-1] + alpha * (x[n] - y[n-1]) */
    f->state = f->state + f->alpha * (input - f->state);
    return f->state;
}

void pt1FilterReset(PT1Filter_t *f, float value)
{
    f->state = value;
}