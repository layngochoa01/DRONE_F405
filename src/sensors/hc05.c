#include "hc05.h"
#include "uart5.h"
#include <string.h>
#include <stdio.h>

/* =========================================================
 * PRIVATE
 * ========================================================= */
static HC05_StreamState stream_state = HC05_STREAM_STOP;

static HC05_CalibCmd    calib_cmd    = HC05_CMD_NONE;

/* Buffer nhận lệnh */
#define CMD_BUF_SIZE 16U
static char cmd_buf[CMD_BUF_SIZE];

static uint16_t cmd_len = 0;

/* =========================================================
 * PUBLIC API
 * ========================================================= */

void HC05_Init(uint32_t apb1_clk, uint32_t baudrate)
{
    UART5_Init(apb1_clk, baudrate);
    UART5_RxFlush();
    stream_state = HC05_STREAM_STOP;
    calib_cmd    = HC05_CMD_NONE;
    UART5_WriteString("HC05_READY\r\n");
}


void HC05_SendIMU(const ICM42605_Data *data)
{
    /* Format: AX,AY,AZ,GX,GY,GZ,TEMP\r\n */
//    UART5_WriteF("A:[%8.2f, %8.2f, %8.2f] | G:[%8.2f, %8.2f, %8.2f] | T:[%6.2f]\r\n",
//                  data->accel_x, data->accel_y, data->accel_z,
//                  data->gyro_x,  data->gyro_y,  data->gyro_z,
//                  data->temp);

    // UART5_WriteF(" A:[%8.2f, %8.2f, %8.2f] \r\n", data->accel_x, data->accel_y, data->accel_z);
    UART5_WriteF(" G:[%8.2f, %8.2f, %8.2f] \r\n",data->gyro_x,  data->gyro_y,  data->gyro_z);
    // UART5_WriteF(" T:[%6.2f]\r\n",data->temp);
}

void HC05_SendRawIMU(const ICM42605_RawData *raw){

    UART5_WriteF(" A:[%6d, %6d, %6d] \r\n", raw->accel_x, raw->accel_y, raw->accel_z);
    // UART5_WriteF(" G:[%6d, %6d, %6d] \r\n",raw->gyro_x,  raw->gyro_y,  raw->gyro_z);

}

void HC05_SendAttitude(float roll, float pitch, float yaw)
{
    UART5_WriteF("R:%6.2f P:%6.2f Y:%6.2f\r\n", roll, pitch, yaw);
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
            else if (strcmp(cmd_buf, "CALIB") == 0) { calib_cmd = HC05_CMD_CALIB; UART5_WriteString("OK CALIB \r\n");} 
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