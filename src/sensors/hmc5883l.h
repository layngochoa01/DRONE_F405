#ifndef HMC5883L_H
#define HMC5883L_H

#include <stdbool.h>
#define M_PI 3.14159265358979323846f

#define COMPASS_OFFSET_X   0.0f
#define COMPASS_OFFSET_Y   0.0f
#define COMPASS_OFFSET_Z   0.0f

typedef enum {
    COMPASS_NONE = 0,
    COMPASS_HMC5883L,
    COMPASS_QMC5883L,
} Compass_Type_e;

typedef struct {
    float rawX, rawY, rawZ;
    float x, y, z;
    float headingDeg;
} Compass_Data_t;

bool Compass_Init(void);
bool Compass_Update(Compass_Data_t *out);
Compass_Type_e Compass_GetType(void);
const char *Compass_TypeName(void);
void Compass_SetOffsets(float ox, float oy, float oz);
void Compass_GetOffsets(float *ox, float *oy, float *oz);

void Compass_CalStart(void);
bool Compass_CalFinish(float *ox, float *oy, float *oz);
bool Compass_IsCalibrating(void);

#endif
