#include "hc05.h"
#include "uart5.h"
#include <string.h>
#include <stdio.h>

static HC05_StreamState stream_state = HC05_STREAM_STOP;

static HC05_CalibCmd    calib_cmd    = HC05_CMD_NONE;

#define CMD_BUF_SIZE 16U
static char cmd_buf[CMD_BUF_SIZE];

static uint16_t cmd_len = 0;
static HC05_FlightCmd flight_cmd = HC05_FLIGHT_CMD_NONE;
static HC05_MotorTestCmd motor_cmd = HC05_MOTOR_NONE;

void HC05_Init(uint32_t apb1_clk, uint32_t baudrate)
{
    UART5_Init(apb1_clk, baudrate);
    UART5_RxFlush();
    stream_state = HC05_STREAM_STOP;
    calib_cmd    = HC05_CMD_NONE;
    UART5_WriteString("HC05_READY\r\n");
}

HC05_StreamState HC05_Poll(void)
{
    while (UART5_RxAvailable() > 0U) {
        char c = UART5_RxRead();
        if (c == '\r') continue;
        if (c == '\n') {
            cmd_buf[cmd_len] = '\0';

            if      (strcmp(cmd_buf, "START") == 0) { stream_state = HC05_STREAM_RUN;  UART5_WriteString("OK_START\r\n"); }
            else if (strcmp(cmd_buf, "STOP")  == 0) { stream_state = HC05_STREAM_STOP; UART5_WriteString("OK_STOP\r\n");  }
            else if (strcmp(cmd_buf, "PING")  == 0) { UART5_WriteString("PONG\r\n"); }
            else if (strcmp(cmd_buf, "CALIBGYRO") == 0) { calib_cmd = HC05_CMD_CALIB_GYRO; UART5_WriteString("OK_CALIB_GYRO\r\n"); }
            else if (strcmp(cmd_buf, "CALIBACCEL") == 0) { calib_cmd = HC05_CMD_CALIB_ACCEL; UART5_WriteString("OK_CALIB_ACCEL\r\n"); }
            else if (strcmp(cmd_buf, "CALIBERASE") == 0) { calib_cmd = HC05_CMD_CALIB_ERASE; UART5_WriteString("OK_CALIB_ERASE\r\n"); }
            else if (strcmp(cmd_buf, "BENCH") == 0)    { calib_cmd = HC05_CMD_BENCH_ON; }
            else if (strcmp(cmd_buf, "BENCHOFF") == 0) { calib_cmd = HC05_CMD_BENCH_OFF; }
            else if (strcmp(cmd_buf, "LOGMAG") == 0)    { calib_cmd = HC05_CMD_LOG_MAG; }
            else if (strcmp(cmd_buf, "LOGGPS") == 0)    { calib_cmd = HC05_CMD_LOG_GPS; }
            else if (strcmp(cmd_buf, "LOGNORM") == 0)   { calib_cmd = HC05_CMD_LOG_NORMAL; }
            else if (strcmp(cmd_buf, "LOGPERF") == 0)   { calib_cmd = HC05_CMD_LOG_PERF; }
            else if (strcmp(cmd_buf, "GPSCSV") == 0)    { calib_cmd = HC05_CMD_LOG_GPSCSV; }
            else if (strcmp(cmd_buf, "LOGMAGCSV") == 0) { calib_cmd = HC05_CMD_LOG_MAGCSV; }
            else if (strcmp(cmd_buf, "MAGCAL") == 0)   { calib_cmd = HC05_CMD_MAGCAL; }
            else if (strcmp(cmd_buf, "MAGCALEND") == 0) { calib_cmd = HC05_CMD_MAGCAL_END; }
            else if (strcmp(cmd_buf, "CALIBSHOW") == 0) { calib_cmd = HC05_CMD_CALIB_SHOW; }
            else if (strcmp(cmd_buf, "CALIB") == 0) { calib_cmd = HC05_CMD_CALIB; UART5_WriteString("OK CALIB \r\n");}
            else if (strcmp(cmd_buf, "M1")    == 0) { motor_cmd = HC05_MOTOR_1; }
            else if (strcmp(cmd_buf, "M2")    == 0) { motor_cmd = HC05_MOTOR_2; }
            else if (strcmp(cmd_buf, "M3")    == 0) { motor_cmd = HC05_MOTOR_3; }
            else if (strcmp(cmd_buf, "M4")    == 0) { motor_cmd = HC05_MOTOR_4; }
            else if (strcmp(cmd_buf, "MALL")  == 0) { motor_cmd = HC05_MOTOR_ALL; }
            else if (strcmp(cmd_buf, "MSTOP") == 0) { motor_cmd = HC05_MOTOR_STOP; }
            else if (strcmp(cmd_buf, "ARM")    == 0) { flight_cmd = HC05_FLIGHT_CMD_ARM;    UART5_WriteString("OK_ARM\r\n"); }
            else if (strcmp(cmd_buf, "DISARM") == 0) { flight_cmd = HC05_FLIGHT_CMD_DISARM; UART5_WriteString("OK_DISARM\r\n"); }
            else if (strcmp(cmd_buf, "HOLD")   == 0) { flight_cmd = HC05_FLIGHT_CMD_HOLD;   UART5_WriteString("OK_HOLD\r\n"); }
            else if (strcmp(cmd_buf, "CLIMB")  == 0) { flight_cmd = HC05_FLIGHT_CMD_CLIMB;  UART5_WriteString("OK_CLIMB\r\n"); }
            else if (strcmp(cmd_buf, "LAND")   == 0) { flight_cmd = HC05_FLIGHT_CMD_LAND;   UART5_WriteString("OK_LAND\r\n"); }
            else if (cmd_len > 0U)                  { UART5_WriteString("ERR_CMD\r\n"); }

            cmd_len = 0;
            return stream_state;
        }

        if (cmd_len < CMD_BUF_SIZE - 1U) {
            cmd_buf[cmd_len++] = c;
        } else {
            cmd_len = 0;
        }
    }

    return stream_state;
}

HC05_StreamState HC05_GetStreamState(void)
{
    return stream_state;
}

HC05_CalibCmd HC05_GetCalibCmd(void)
{
    HC05_CalibCmd cmd = calib_cmd;
    calib_cmd = HC05_CMD_NONE;
    return cmd;
}

HC05_FlightCmd HC05_GetFlightCmd(void)
{
    HC05_FlightCmd cmd = flight_cmd;
    flight_cmd = HC05_FLIGHT_CMD_NONE;
    return cmd;
}

HC05_MotorTestCmd HC05_GetMotorTestCmd(void)
{
    HC05_MotorTestCmd cmd = motor_cmd;
    motor_cmd = HC05_MOTOR_NONE;
    return cmd;
}
