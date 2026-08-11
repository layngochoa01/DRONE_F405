#ifndef HC05_H
#define HC05_H

#include <stdint.h>
#include "icm42605.h"

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

/* Trạng thái stream */
typedef enum {
    HC05_STREAM_STOP = 0,
    HC05_STREAM_RUN  = 1,
} HC05_StreamState;

typedef enum {
    HC05_CMD_NONE        = 0,
    HC05_CMD_CALIB_GYRO  = 1,
    HC05_CMD_CALIB_ACCEL = 2,
    HC05_CMD_CALIB_ERASE = 3,
} HC05_CalibCmd;

void HC05_Init(uint32_t apb1_clk, uint32_t baudrate);

void HC05_SendIMU(const ICM42605_Data *data);

void HC05_SendAttitude(float roll, float pitch, float yaw);

HC05_StreamState HC05_Poll(void);

HC05_StreamState HC05_GetStreamState(void);

HC05_CalibCmd    HC05_GetCalibCmd(void);

#endif 