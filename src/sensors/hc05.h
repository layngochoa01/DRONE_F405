#ifndef HC05_H
#define HC05_H

#include <stdint.h>
#include "icm42605.h"
#include "../estimation/altitude.h"
#include "../estimation/attitude.h"
/* =========================================================
 * HC-05 Driver
 * Giao tiếp qua UART5 (PC12=TX, PD2=RX)
 * Baudrate 115200
 *
 * Protocol RX (lệnh điều khiển):
 *   "START\n"  → bắt đầu stream
 *   "STOP\n"   → dừng stream
 *   "PING\n"   → trả lời "PONG\n" để kiểm tra kết nối
 *   "CALIB_GYRO\n"   → calib gyro, lưu Flash
 *   "CALIB_ACCEL\n"  → calib accel, lưu Flash
 *   "CALIB_ERASE\n"  → xóa calib data trong Flash
 * ========================================================= */


typedef enum {
    HC05_STREAM_STOP = 0,
    HC05_STREAM_RUN  = 1,
} HC05_StreamState;

typedef enum {
    HC05_CMD_NONE        = 0,
    HC05_CMD_CALIB_GYRO  = 1,
    HC05_CMD_CALIB_ACCEL = 2,
    HC05_CMD_CALIB_ERASE = 3,
    HC05_CMD_CALIB = 4,
} HC05_CalibCmd;

typedef enum {
    HC05_FLIGHT_CMD_NONE   = 0,
    HC05_FLIGHT_CMD_ARM    = 1,
    HC05_FLIGHT_CMD_DISARM = 2,
    HC05_FLIGHT_CMD_HOLD   = 3,
    HC05_FLIGHT_CMD_CLIMB  = 4,
    HC05_FLIGHT_CMD_LAND   = 5,
} HC05_FlightCmd;

/* thêm vào phần khai báo hàm public, cạnh HC05_GetCalibCmd */
HC05_FlightCmd HC05_GetFlightCmd(void);

void HC05_Init(uint32_t apb1_clk, uint32_t baudrate);
void HC05_SendIMU(const ICM42605_Data *data);
void HC05_SendRawIMU(const ICM42605_RawData *raw);
void HC05_SendAttitude(float roll, float pitch, float yaw);
void HC05_LogQuaternion(const Quaternion_t *q);
void HC05_LogAltitudeFKState(const AltitudeKF_State_t *kf);
HC05_StreamState HC05_Poll(void);

HC05_StreamState HC05_GetStreamState(void);
HC05_CalibCmd    HC05_GetCalibCmd(void);
HC05_FlightCmd HC05_GetFlightCmd(void);
#endif 