#ifndef SPL06_H
#define SPL06_H

#include<stdint.h>
#include<stdbool.h>

typedef struct {
    int32_t c0, c1, c00, c10, c01, c11, c20, c21, c30;
} SPL06_Calib_t;

typedef struct {
    float pressPa;
    float tempC;
} SPL06_Data_t;

bool SPL06_Init(void);
bool SPL06_Update(SPL06_Data_t *out);

#endif