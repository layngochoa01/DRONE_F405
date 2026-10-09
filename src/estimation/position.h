#ifndef POSITION_H
#define POSITION_H

#include <stdint.h>
#include <stdbool.h>
#include "../sensors/gps.h"

/* =========================================================
 * Ước lượng vị trí / vận tốc ngang: GPS + gia tốc IMU
 *
 * Hệ trái đất NWU (khớp với rMat của AHRS: Z lên, X = Bắc từ):
 *   X = Bắc, Y = Tây. Gốc tọa độ (home) = fix GPS tốt đầu tiên.
 *
 * Predict 100 Hz bằng gia tốc EF, correct mỗi khi có fix GPS mới
 * (bộ lọc bù 2 trạng thái / trục).
 * ========================================================= */

#define POS_GPS_MIN_SATS     6U
#define POS_GPS_MAX_HDOP     2.5f
#define POS_GPS_TIMEOUT_MS   1500U

typedef struct {
    float posN, posW;   /* m, so với home */
    float velN, velW;   /* m/s */
} PositionEstimate_t;

void Position_Init(void);
void Position_Predict(float accN, float accW, float dT);
void Position_UpdateGps(const GPS_Data_t *gps, uint32_t nowMs);
/* true nếu có home và GPS chưa mất quá POS_GPS_TIMEOUT_MS */
bool Position_IsValid(uint32_t nowMs);
const PositionEstimate_t *Position_Get(void);

#endif
