#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H

#include <stdbool.h>

typedef enum {
    FLIGHT_STATE_DISARMED,
    FLIGHT_STATE_ARMED_IDLE,
    FLIGHT_STATE_CLIMB,
    FLIGHT_STATE_ALT_HOLD,
    FLIGHT_STATE_POS_HOLD,
    FLIGHT_STATE_LAND,
    FLIGHT_STATE_MOVE_FORWARD,
    FLIGHT_STATE_MOVE_LEFT,
    FLIGHT_STATE_LAND_BLIND,
} FlightState_e;

#define CLIMB_STEP_METERS     1.0f
#define GROUND_ALTITUDE_EPS   0.05f
#define MOVE_DURATION_MS      5000U
#define MOVE_SPEED_MS         1.0f

void FlightState_Init(void);
void FlightState_Update(float currentAltitude, bool navOk, bool altOk, float dT);
FlightState_e FlightState_Get(void);
float FlightState_GetTargetAltitude(void);
bool FlightState_MotorsEnabled(void);
bool FlightState_IsAirborne(FlightState_e state);
const char *FlightState_Name(FlightState_e state);

#define BT_BENCH_TIMEOUT_MS   120000.0f
void FlightState_BluetoothBenchArm(bool on);
bool FlightState_IsBluetoothArmed(void);

#endif
