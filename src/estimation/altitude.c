#include "altitude.h"
#include <math.h>

/* Hệ số complementary filter - P cho vị trí, P cho vận tốc.
 * Bắt đầu với giá trị nhỏ, tinh chỉnh sau khi test thực tế. */
#define K_ALT   1.0f
#define K_VEL   0.3f

/* cm/s^2 -> m/s^2, vì GRAVITY_CMSS trong imu.h dùng đơn vị cm/s^2 */
#define CM_TO_M 0.01f

static AltitudeState_t s_state;
static float s_baselinePressure = 101325.0f;   /* mặc định mực nước biển, sẽ ghi đè bằng SetBaseline */

void Altitude_Init(void)
{
    s_state.altitude = 0.0f;
    s_state.verticalVelocity = 0.0f;
}

void Altitude_SetBaseline(float currentPressurePa)
{
    s_baselinePressure = currentPressurePa;
}

float Altitude_PressureToMeters(float pressurePa)
{
    /* Công thức khí áp chuẩn (barometric formula), theo ISA - International Standard Atmosphere */
    return 44330.0f * (1.0f - powf(pressurePa / s_baselinePressure, 0.1903f));
}

void Altitude_Update(float baroAltitude, float verticalAccel, float dT)
{
    float verticalAccel_m = verticalAccel * CM_TO_M;   /* đổi sang m/s^2 để khớp đơn vị mét của altitude/velocity */

    /* 1. Predict: tích phân accel (nhanh, mượt, nhưng trôi theo thời gian nếu dùng một mình) */
    s_state.verticalVelocity += verticalAccel_m * dT;
    s_state.altitude += s_state.verticalVelocity * dT;

    /* 2. Correct: kéo dần về giá trị baro (chậm, nhiễu cao tần, nhưng không trôi dài hạn) */
    float error = baroAltitude - s_state.altitude;
    s_state.altitude += error * K_ALT * dT;
    s_state.verticalVelocity += error * K_VEL * dT;
}

const AltitudeState_t *Altitude_Get(void)
{
    return &s_state;
}