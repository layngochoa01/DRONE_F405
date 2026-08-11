#ifndef FILTER_H
#define FILTER_H

/* =========================================================
 * filter.h - LPF PT1
 * PT1 = bộ lọc RC bậc 1 (exponential moving average)
 *
 * Công thức:
 *   RC    = 1 / (2π × f_cut)
 *   alpha = dT / (RC + dT)
 *   y[n]  = y[n-1] + alpha × (x[n] - y[n-1])
 * ========================================================= */

typedef struct {
    float state;   /* output hiện tại (y[n-1]) */
    float alpha;   /* hệ số smoothing           */
    float RC;      /* time constant             */
} PT1Filter_t;

/* Khởi tạo filter với tần số cắt f_cut (Hz) và dT (s) */
void  pt1FilterInit(PT1Filter_t *f, float f_cut, float dT);

/* Cập nhật cutoff (khi dT thay đổi) */
void  pt1FilterUpdateCutoff(PT1Filter_t *f, float f_cut, float dT);

/* Apply 1 sample, trả về output */
float pt1FilterApply(PT1Filter_t *f, float input);

/* Reset state về giá trị cụ thể */
void  pt1FilterReset(PT1Filter_t *f, float value);

#endif