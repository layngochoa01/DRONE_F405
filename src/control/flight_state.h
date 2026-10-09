#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H

#include <stdbool.h>

/* =========================================================
 * Máy trạng thái chế độ bay — chọn bằng CH1..CH4 (SBUS, Mode 2)
 *
 *   CH1 roll (cần phải ngang)   CH2 pitch (cần phải dọc)
 *   CH3 ga   (cần trái dọc)     CH4 yaw   (cần trái ngang)
 *
 *  Mức cần:  THẤP < 1200 µs | GIỮA 1300..1700 µs | CAO > 1800 µs
 *
 *  ARM      : ga THẤP + yaw CAO (phải), giữ 1 s      (chỉ khi DISARMED)
 *  DISARM   : ga THẤP + yaw THẤP (trái), giữ 0.5 s
 *
 *  Cần ga quyết định chuyển động đứng:
 *   1. CLIMB    : đẩy ga lên CAO   → mục tiêu = độ cao hiện tại + 1 m
 *                 đạt mục tiêu → quay về chế độ hold trước đó
 *   4. LAND     : kéo ga xuống THẤP → hạ cánh, chạm đất → ARMED_IDLE
 *      Ga về GIỮA khi đang CLIMB/LAND → dừng, hold tại độ cao hiện tại
 *
 *  Khi ga ở GIỮA, cần phải chọn chế độ (giữ 0.5 s, rồi phải thả về giữa):
 *   2. ALT_HOLD : pitch THẤP (kéo về)   → giữ độ cao, không GPS
 *   3. POS_HOLD : roll  CAO  (sang phải)→ giữ độ cao + vị trí GPS
 *   5. FORWARD  : pitch CAO  (đẩy tới)  → bay tới 1 m/s trong 5 s
 *   6. LEFT     : roll  THẤP (sang trái)→ bay sang trái 1 m/s trong 5 s
 *      Hết 5 s (TIM5) → POS_HOLD tại vị trí mới. Độ cao giữ nguyên suốt quá trình.
 *
 *  POS_HOLD / FORWARD / LEFT cần GPS + compass; không có thì lệnh bị bỏ qua,
 *  mất GPS giữa chừng → tự về ALT_HOLD.
 *  Cần yaw (khi ga không THẤP) xoay mũi trong mọi chế độ đang bay.
 * ========================================================= */

typedef enum {
    FLIGHT_STATE_DISARMED,
    FLIGHT_STATE_ARMED_IDLE,
    FLIGHT_STATE_CLIMB,
    FLIGHT_STATE_ALT_HOLD,
    FLIGHT_STATE_POS_HOLD,
    FLIGHT_STATE_LAND,
    FLIGHT_STATE_MOVE_FORWARD,
    FLIGHT_STATE_MOVE_LEFT,
} FlightState_e;

#define CLIMB_STEP_METERS     1.0f
#define GROUND_ALTITUDE_EPS   0.05f
#define MOVE_DURATION_MS      5000U
#define MOVE_SPEED_MS         1.0f    /* vận tốc khi FORWARD / LEFT */

void FlightState_Init(void);
/* navOk: GPS hợp lệ + compass hoạt động (cho phép POS_HOLD / MOVE) */
void FlightState_Update(float currentAltitude, bool navOk, float dT);
FlightState_e FlightState_Get(void);
float FlightState_GetTargetAltitude(void);
bool FlightState_MotorsEnabled(void);
bool FlightState_IsAirborne(FlightState_e state);
const char *FlightState_Name(FlightState_e state);

/* ARM qua Bluetooth CHỈ để thử trên bàn (THÁO CÁNH): vào ARMED_IDLE, bỏ qua
 * SBUS/failsafe và cần gạt, không thể chuyển sang trạng thái bay.
 * Tự DISARM sau BT_BENCH_TIMEOUT_MS. on=false → DISARM ngay. */
#define BT_BENCH_TIMEOUT_MS   120000.0f
void FlightState_BluetoothBenchArm(bool on);
bool FlightState_IsBluetoothArmed(void);

#endif
