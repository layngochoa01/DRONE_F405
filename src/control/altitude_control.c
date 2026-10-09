#include "altitude_control.h"

static PID_t s_posPid;         
static PID_t s_velPid;          
static float s_lastThrottle = 0.0f;
static float s_velSetpoint  = 0.0f;
static bool  s_wasActive    = false;

static float constrainf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void AltitudeControl_Init(void)
{
    PID_Init(&s_posPid, ALT_KP_POS, 0.0f, 0.0f);
    PID_Init(&s_velPid, ALT_KP_VEL, ALT_KI_VEL, ALT_KD_VEL);
    s_lastThrottle = 0.0f;
    s_velSetpoint  = 0.0f;
    s_wasActive    = false;
}

void AltitudeControl_Reset(float altitude, float velocity)
{
    PID_Reset(&s_posPid);
    PID_Reset(&s_velPid);
    s_posPid.prevMeasurement = altitude;
    s_velPid.prevMeasurement = velocity;
    s_velSetpoint = 0.0f;
}

float AltitudeControl_Update(FlightState_e state, float targetAlt,
                             float altitude, float velocity,
                             float cosTilt, float dT)
{
    /* Mọi trạng thái đang bay (kể cả FORWARD / LEFT) đều giữ độ cao */
    bool active = FlightState_IsAirborne(state);

    if (!active || dT <= 0.0f) {
        AltitudeControl_Reset(altitude, velocity);
        s_wasActive = false;
        s_lastThrottle = (state == FLIGHT_STATE_ARMED_IDLE) ? ALT_IDLE_THROTTLE : 0.0f;
        return s_lastThrottle;
    }
    if (!s_wasActive) {
        AltitudeControl_Reset(altitude, velocity);
        s_wasActive = true;
    }

    /* Outer loop: altitude -> climb rate */
    s_velSetpoint = PID_Update(&s_posPid, targetAlt - altitude, altitude, dT,
                               -ALT_MAX_DESCENT_RATE, ALT_MAX_CLIMB_RATE);

    /* Inner loop: climb rate -> throttle correction */
    float correction = PID_Update(&s_velPid, s_velSetpoint - velocity, velocity, dT,
                                  -ALT_MAX_CORRECTION, ALT_MAX_CORRECTION);

    /* Tilt compensation: lift drops by cos(tilt) when the drone leans */
    float c = constrainf(cosTilt, ALT_MIN_COS_TILT, 1.0f);
    float throttle = (ALT_HOVER_THROTTLE + correction) / c;
    throttle = constrainf(throttle, ALT_THROTTLE_MIN, ALT_THROTTLE_MAX);

    /* Slew rate limit */
    float delta = constrainf(throttle - s_lastThrottle,
                             -ALT_SLEW_DOWN * dT, ALT_SLEW_UP * dT);
    s_lastThrottle += delta;

    return s_lastThrottle;
}

float AltitudeControl_GetVelSetpoint(void)
{
    return s_velSetpoint;
}