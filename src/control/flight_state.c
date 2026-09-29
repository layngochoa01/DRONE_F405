#include "flight_state.h"
#include <stdbool.h>

static FlightState_e s_state = FLIGHT_STATE_DISARMED;
static float s_targetAltitude = 0.0f;

void FlightState_Init(void)
{
    s_state = FLIGHT_STATE_DISARMED;
    s_targetAltitude = 0.0f;
}

void FlightState_HandleCommand(HC05_FlightCmd cmd, float currentAltitude)
{
    if (cmd == HC05_FLIGHT_CMD_DISARM) {
        s_state = FLIGHT_STATE_DISARMED;
        s_targetAltitude = 0.0f;
        return;
    }

    switch (s_state) {
        case FLIGHT_STATE_DISARMED:
            if (cmd == HC05_FLIGHT_CMD_ARM) {
                s_state = FLIGHT_STATE_ARMED_IDLE;
                s_targetAltitude = 0.0f;
            }
            break;

        case FLIGHT_STATE_ARMED_IDLE:
            if (cmd == HC05_FLIGHT_CMD_CLIMB) {
                s_state = FLIGHT_STATE_CLIMB;
                s_targetAltitude = currentAltitude + CLIMB_STEP_METERS;
            } else if (cmd == HC05_FLIGHT_CMD_HOLD) {
                s_state = FLIGHT_STATE_ATTITUDE_HOLD;
                s_targetAltitude = currentAltitude;
            }
            break;

        case FLIGHT_STATE_ATTITUDE_HOLD:
            if (cmd == HC05_FLIGHT_CMD_CLIMB) {
                s_state = FLIGHT_STATE_CLIMB;
                s_targetAltitude = currentAltitude + CLIMB_STEP_METERS;
            } else if (cmd == HC05_FLIGHT_CMD_LAND) {
                s_state = FLIGHT_STATE_LAND;
            }
            break;

        case FLIGHT_STATE_CLIMB:
            if (cmd == HC05_FLIGHT_CMD_CLIMB) {
                s_targetAltitude += CLIMB_STEP_METERS;
            } else if (cmd == HC05_FLIGHT_CMD_HOLD) {
                s_state = FLIGHT_STATE_ATTITUDE_HOLD;
                s_targetAltitude = currentAltitude;
            } else if (cmd == HC05_FLIGHT_CMD_LAND) {
                s_state = FLIGHT_STATE_LAND;
            }
            break;

        case FLIGHT_STATE_LAND:
            if (cmd == HC05_FLIGHT_CMD_HOLD) {
                s_state = FLIGHT_STATE_ATTITUDE_HOLD;
                s_targetAltitude = currentAltitude;
            }
            break;
    }
}

void FlightState_Update(float currentAltitude, float dT)
{
    switch (s_state) {
        case FLIGHT_STATE_CLIMB:
            if (currentAltitude >= s_targetAltitude - 0.1f) {
                s_state = FLIGHT_STATE_ATTITUDE_HOLD;
            }
            break;

        case FLIGHT_STATE_LAND:
            s_targetAltitude -= LAND_DESCENT_RATE * dT;
            if (s_targetAltitude < 0.0f) s_targetAltitude = 0.0f;
            if (currentAltitude <= GROUND_ALTITUDE_EPS) {
                s_state = FLIGHT_STATE_ARMED_IDLE;
                s_targetAltitude = 0.0f;
            }
            break;

        default:
            break;  
    }
}

FlightState_e FlightState_Get(void)
{
    return s_state;
}

float FlightState_GetTargetAltitude(void)
{
    return s_targetAltitude;
}

bool FlightState_MotorsEnabled(void)
{
    return s_state != FLIGHT_STATE_DISARMED;
}