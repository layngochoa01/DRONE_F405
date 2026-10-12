#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <stdbool.h>
#include "../sensors/icm42605.h"
#include "flight_state.h"
#include "../sensors/hmc5883l.h"

/* =========================================================
 * Điều khiển bay nhiều vòng lồng nhau (cascaded control)
 *
 *  VÒNG NHANH 2 kHz — trong ngắt DMA đọc IMU (FlightControl_FastUpdate):
 *    IMU → AHRS → [cộng dồn gia tốc hệ trái đất]
 *        → quaternion mục tiêu → điều khiển tư thế → rate PID → mixer → PWM
 *
 *  VÒNG CHẬM — trong main loop (FlightControl_SlowUpdate):
 *    ~100 Hz : trung bình gia tốc → predict Kalman độ cao + ước lượng vị trí
 *     25 Hz  : baro → update Kalman; STATE; vòng độ cao Z → ga; vòng yaw
 *     25 Hz  : compass (lệch pha 20 ms với baro)
 *     10 Hz  : vòng vị trí XY → roll, pitch
 *    khi có  : GPS → update ước lượng vị trí
 *
 *  Vòng chậm ghi setpoint (roll, pitch, yaw, ga, cờ) cho vòng nhanh qua vùng găng.
 * ========================================================= */

#define FC_LOOP_HZ        2000U
#define FC_DT             (1.0f / (float)FC_LOOP_HZ)
#define FC_ACCEL_BLOCK    20U     /* số mẫu IMU gộp cho 1 lần predict (~100 Hz) */

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

/* Vòng nhanh — gọi TRONG NGẮT mỗi mẫu IMU (2 kHz), data đã remap + calib + lọc.
 * Không được gọi hàm chặn (I2C, UART) từ đây. */
void FlightControl_FastUpdate(const ICM42605_Data *data);

/* Vòng chậm — gọi liên tục trong main loop, tự chia nhịp theo SysTick */
void FlightControl_SlowUpdate(void);

/* Đo tải vòng nhanh (µs). resetMax = true: xóa giá trị lớn nhất sau khi đọc */
typedef struct {
    float execUs, execMaxUs;        /* thời gian xử lý 1 lượt */
    float periodMinUs, periodMaxUs; /* khoảng cách giữa 2 mẫu IMU */
} FlightControl_Perf_t;
void FlightControl_GetPerf(FlightControl_Perf_t *out, bool resetMax);

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
