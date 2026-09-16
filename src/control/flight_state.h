#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H

#include <stdbool.h>
#include "../drivers/hc05.h"  

typedef enum {
    FLIGHT_STATE_DISARMED,
    FLIGHT_STATE_ARMED_IDLE,
    FLIGHT_STATE_ATTITUDE_HOLD,
    FLIGHT_STATE_CLIMB,
    FLIGHT_STATE_LAND,
} FlightState_e;

#define CLIMB_STEP_METERS     1.0f
#define LAND_DESCENT_RATE     0.3f
#define GROUND_ALTITUDE_EPS   0.05f

void FlightState_Init(void);
void FlightState_HandleCommand(HC05_FlightCmd cmd, float currentAltitude);
void FlightState_Update(float currentAltitude, float dT);

FlightState_e FlightState_Get(void);
float FlightState_GetTargetAltitude(void);
bool FlightState_MotorsEnabled(void);

#endif