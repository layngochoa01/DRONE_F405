#ifndef HMC5883L_H
#define HMC5883L_H

#include <stdbool.h>
#define M_PI 3.14159265358979323846f

/* =========================================================
 * Compass: tự nhận HMC5883L (I2C 0x1E) hoặc QMC5883L (0x0D)
 * — module GPS M8N giá rẻ thường là QMC5883L.
 * ========================================================= */

/* Hard-iron offset mặc định (gauss, trục CHIP) — đo bằng lệnh MAGCAL rồi dán vào đây.
 * 0 = chưa calib. */
#define COMPASS_OFFSET_X   0.0f
#define COMPASS_OFFSET_Y   0.0f
#define COMPASS_OFFSET_Z   0.0f

typedef enum {
    COMPASS_NONE = 0,
    COMPASS_HMC5883L,
    COMPASS_QMC5883L,
} Compass_Type_e;

typedef struct {
    float rawX, rawY, rawZ;  /* gauss, trục CHIP, chưa trừ offset (debug / calib) */
    float x, y, z;           /* gauss, body frame FLU, đã trừ offset */
    float headingDeg;        /* từ x, y body — chỉ đúng khi drone nằm ngang */
} Compass_Data_t;

bool Compass_Init(void);
bool Compass_Update(Compass_Data_t *out);
Compass_Type_e Compass_GetType(void);
const char *Compass_TypeName(void);
void Compass_SetOffsets(float ox, float oy, float oz);   /* trục chip */
void Compass_GetOffsets(float *ox, float *oy, float *oz);

/* Calib hard-iron: CalStart → xoay drone đủ mọi hướng ~30 s → CalFinish.
 * offset = (max + min) / 2 mỗi trục chip, áp dụng ngay (RAM). */
void Compass_CalStart(void);
bool Compass_CalFinish(float *ox, float *oy, float *oz);   /* false nếu chưa đủ dữ liệu */
bool Compass_IsCalibrating(void);

#endif
