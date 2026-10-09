#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <stdbool.h>
#include "../sensors/icm42605.h"
#include "flight_state.h"
#include "../sensors/hmc5883l.h"

/* =========================================================
 * Điều khiển bay nhiều vòng lồng nhau (cascaded control)
 *
 *  RC/SBUS → STATE ─┬─ Vòng độ cao Z  (25 Hz) ── ga ───────────────┐
 *                   ├─ Vòng vị trí XY (10 Hz) ── roll, pitch ─┐     │
 *                   └─ Vòng yaw       (25 Hz) ── yaw ─────────┤     │
 *                                     Quaternion mục tiêu ◄───┘     │
 *                                     Điều khiển tư thế (100 Hz)    │
 *                                     Vòng rate PID     (100 Hz)    │
 *                                     Motor mixer ◄─────────────────┘
 *                                     PWM → ESC
 *
 * Ước lượng: AHRS (gyro+accel+HMC5883L), độ cao (SPL06 + accel),
 *            vị trí/vận tốc (GPS + accel).
 * ========================================================= */

#define FC_LOOP_HZ        100U
#define FC_DT             (1.0f / (float)FC_LOOP_HZ)

/* Kênh SBUS (chỉ số từ 0) */
#define FC_CH_ROLL        0U
#define FC_CH_PITCH       1U
#define FC_CH_THROTTLE    2U
#define FC_CH_YAW         3U
/* Chọn chế độ bay bằng CH1..CH4: xem flight_state.h */

#define FC_RC_DEADBAND    0.05f   /* vùng chết cần yaw (tỉ lệ) */

/* Hệ số P tư thế: (rad/s) / rad */
#define FC_ATT_KP_ROLL    5.0f
#define FC_ATT_KP_PITCH   5.0f
#define FC_ATT_KP_YAW     3.0f
#define FC_RATE_MAX_RP    250.0f  /* deg/s */
#define FC_RATE_MAX_YAW   120.0f  /* deg/s */

typedef struct {
    float rollSpDeg, pitchSpDeg, yawSpDeg;  /* tư thế mục tiêu */
    float throttle;                         /* 0..1 */
    FlightState_e state;
    bool  magHealthy;
    bool  gpsValid;
    float motorUs[4];
} FlightControl_Status_t;

/* baroOk / magOk: kết quả init cảm biến */
void FlightControl_Init(bool baroOk, bool magOk);

/* Gọi mỗi khi có mẫu IMU mới (100 Hz), data đã remap + calib + lọc */
void FlightControl_Update(const ICM42605_Data *data);

const FlightControl_Status_t *FlightControl_GetStatus(void);

/* Motor test — CHỈ chạy khi DISARMED (THÁO CÁNH!). Quay ở FC_MOTOR_TEST_US trong
 * FC_MOTOR_TEST_MS rồi tự dừng; ARM hoặc FlightControl_MotorTestStop() dừng ngay.
 * motorIndex: 0..3 = chân pwm_out (M1..M4), -1 = cả 4 motor. */
#define FC_MOTOR_TEST_US   1200U   /* BLHeli mặc định chỉ quay từ ~1148 µs */
#define FC_MOTOR_TEST_MS   3000U
bool FlightControl_MotorTest(int8_t motorIndex);

/* BENCH: thử dấu phản hồi trên bàn (THÁO CÁNH). Chỉ bật được khi ARMED_IDLE.
 * Ga cố định FC_BENCH_THROTTLE, vòng tư thế + rate chạy thật (giữ thăng bằng,
 * giữ hướng mũi), KHÔNG dùng vòng độ cao. DISARM / mất RC → tự tắt. */
#define FC_BENCH_THROTTLE  0.35f
bool FlightControl_SetBench(bool on);
bool FlightControl_IsBench(void);

/* Bản đọc compass gần nhất; ageMs = thời gian từ lần đọc thành công (debug) */
const Compass_Data_t *FlightControl_GetCompass(uint32_t *ageMs);
void FlightControl_MotorTestStop(void);

#endif
