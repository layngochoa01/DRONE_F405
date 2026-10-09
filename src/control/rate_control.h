#ifndef RATE_CONTROL_H
#define RATE_CONTROL_H

#include "../estimation/imu.h"

/* Vòng tốc độ góc (100 Hz)
 *   vào : rate setpoint và gyro đã lọc, đơn vị deg/s (body FLU)
 *   ra  : mô-men roll/pitch/yaw, đơn vị µs PWM (cộng thẳng vào mixer)
 * D lấy từ đạo hàm gyro (measurement) → không bị giật khi setpoint đổi.
 * Anti-windup back-calculation + giới hạn đầu ra trong PID_Update. */

#define RATE_OUT_LIMIT_RP    400.0f   /* µs */
#define RATE_OUT_LIMIT_YAW   200.0f   /* µs */

void RateControl_Init(void);
/* Xóa tích phân, gán D-state = gyro hiện tại (gọi khi đang ở mặt đất) */
void RateControl_Reset(const Vec3_t *gyroDegS);
void RateControl_Update(Vec3_t *output, const Vec3_t *rateSetpointDegS, const Vec3_t *gyroDegS, float dT);

#endif
