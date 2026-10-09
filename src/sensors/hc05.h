#ifndef HC05_H
#define HC05_H

#include <stdint.h>
#include "icm42605.h"
#include "../estimation/altitude.h"
#include "../estimation/attitude.h"

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
    HC05_CMD_CALIB_SHOW  = 5,
    HC05_CMD_PWM_DEBUG   = 6,
    HC05_CMD_BENCH_ON    = 7,
    HC05_CMD_BENCH_OFF   = 8,
    HC05_CMD_LOG_MAG     = 9,
    HC05_CMD_LOG_GPS     = 10,
    HC05_CMD_LOG_NORMAL  = 11,
    HC05_CMD_MAGCAL      = 12,
    HC05_CMD_MAGCAL_END  = 13,
} HC05_CalibCmd;

typedef enum {
    HC05_FLIGHT_CMD_NONE   = 0,
    HC05_FLIGHT_CMD_ARM    = 1,
    HC05_FLIGHT_CMD_DISARM = 2,
    HC05_FLIGHT_CMD_HOLD   = 3,
    HC05_FLIGHT_CMD_CLIMB  = 4,
    HC05_FLIGHT_CMD_LAND   = 5,
} HC05_FlightCmd;

/* Motor test (chỉ chạy khi DISARMED, THÁO CÁNH): "M1".."M4", "MALL", "MSTOP" */
typedef enum {
    HC05_MOTOR_NONE = 0,
    HC05_MOTOR_1    = 1,
    HC05_MOTOR_2    = 2,
    HC05_MOTOR_3    = 3,
    HC05_MOTOR_4    = 4,
    HC05_MOTOR_ALL  = 5,
    HC05_MOTOR_STOP = 6,
} HC05_MotorTestCmd;

HC05_MotorTestCmd HC05_GetMotorTestCmd(void);

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