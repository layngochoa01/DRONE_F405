#ifndef ALTITUDE_CONTROL_H
#define ALTITUDE_CONTROL_H

#include <stdbool.h>
#include "pid.h"
#include "flight_state.h"

#define ALT_KP_POS            1.0f
#define ALT_MAX_CLIMB_RATE    1.0f
#define ALT_MAX_DESCENT_RATE  0.5f
#define ALT_KP_VEL            0.15f
#define ALT_KI_VEL            0.10f
#define ALT_KD_VEL            0.0f
#define ALT_MAX_CORRECTION    0.25f
#define ALT_HOVER_THROTTLE    0.50f
#define ALT_IDLE_THROTTLE     0.00f
#define ALT_THROTTLE_MIN      0.15f
#define ALT_THROTTLE_MAX      0.85f
#define ALT_MIN_COS_TILT      0.50f

#define ALT_SLEW_UP           0.50f
#define ALT_SLEW_DOWN         1.00f

void  AltitudeControl_Init(void);
void  AltitudeControl_Reset(float altitude, float velocity);

float AltitudeControl_Update(FlightState_e state, float targetAlt, float altitude, float velocity, float cosTilt, float dT);

void  AltitudeControl_SeedThrottle(float throttle);

#endif
