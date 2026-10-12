#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <stdbool.h>
#include "../sensors/icm42605.h"
#include "flight_state.h"
#include "../sensors/hmc5883l.h"

#define FC_LOOP_HZ        2000U
#define FC_DT             (1.0f / (float)FC_LOOP_HZ)
#define FC_ACCEL_BLOCK    20U

#define FC_CH_ROLL        0U
#define FC_CH_PITCH       1U
#define FC_CH_THROTTLE    2U
#define FC_CH_YAW         3U

#define FC_RC_DEADBAND    0.05f

#define FC_ATT_KP_ROLL    5.0f
#define FC_ATT_KP_PITCH   5.0f
#define FC_ATT_KP_YAW     3.0f
#define FC_RATE_MAX_RP    250.0f
#define FC_RATE_MAX_YAW   120.0f

typedef struct {
    float rollSpDeg, pitchSpDeg, yawSpDeg;
    float throttle;
    FlightState_e state;
    bool  magHealthy;
    bool  gpsValid;
    float motorUs[4];
} FlightControl_Status_t;

void FlightControl_Init(bool baroOk, bool magOk);

void FlightControl_FastUpdate(const ICM42605_Data *data);

void FlightControl_SlowUpdate(void);

typedef struct {
    float execUs, execMaxUs;
    float periodMinUs, periodMaxUs;
} FlightControl_Perf_t;
void FlightControl_GetPerf(FlightControl_Perf_t *out, bool resetMax);

const FlightControl_Status_t *FlightControl_GetStatus(void);

#define FC_MOTOR_TEST_US   1200U
#define FC_MOTOR_TEST_MS   3000U
bool FlightControl_MotorTest(int8_t motorIndex);

#define FC_BENCH_THROTTLE  0.35f
bool FlightControl_SetBench(bool on);
bool FlightControl_IsBench(void);

const Compass_Data_t *FlightControl_GetCompass(uint32_t *ageMs);
void FlightControl_MotorTestStop(void);

#endif
