#include "flight_state.h"
#include "../drivers/sbus.h"
#include "../drivers/timer.h"

#define CH_ROLL      0U
#define CH_PITCH     1U
#define CH_THROTTLE  2U
#define CH_YAW       3U

#define STICK_LOW_MAX     1200U
#define STICK_MID_MIN     1300U
#define STICK_MID_MAX     1700U
#define STICK_HIGH_MIN    1800U

#define ARM_HOLD_TIME_MS      1000.0f
#define DISARM_HOLD_TIME_MS   500.0f
#define GESTURE_HOLD_TIME_MS  500.0f

typedef enum {
    STICK_NONE,
    STICK_LOW,
    STICK_MID,
    STICK_HIGH,
} StickPos_e;

typedef enum {
    GESTURE_NONE,
    GESTURE_CENTER,
    GESTURE_ALT_HOLD,
    GESTURE_POS_HOLD,
    GESTURE_FORWARD,
    GESTURE_LEFT,
} Gesture_e;

static FlightState_e s_state      = FLIGHT_STATE_DISARMED;
static FlightState_e s_holdState  = FLIGHT_STATE_ALT_HOLD;
static float      s_targetAltitude = 0.0f;
static StickPos_e s_lastThr        = STICK_NONE;
static float      s_armTimer       = 0.0f;
static float      s_disarmTimer    = 0.0f;
static Gesture_e  s_pendingGesture = GESTURE_NONE;
static float      s_gestureTimer   = 0.0f;
static bool       s_gestureLatched = false;
static bool       s_btArmed        = false;
static float      s_btArmTimer     = 0.0f;
static bool       s_altOk          = false;

static StickPos_e stickPos(uint16_t us)
{
    if (us <= STICK_LOW_MAX)  return STICK_LOW;
    if (us >= STICK_HIGH_MIN) return STICK_HIGH;
    if (us >= STICK_MID_MIN && us <= STICK_MID_MAX) return STICK_MID;
    return STICK_NONE;
}

static Gesture_e readGesture(uint16_t roll, uint16_t pitch)
{
    StickPos_e r = stickPos(roll);
    StickPos_e p = stickPos(pitch);

    if (r == STICK_MID  && p == STICK_MID)  return GESTURE_CENTER;
    if (r == STICK_MID  && p == STICK_LOW)  return GESTURE_ALT_HOLD;
    if (r == STICK_HIGH && p == STICK_MID)  return GESTURE_POS_HOLD;
    if (r == STICK_MID  && p == STICK_HIGH) return GESTURE_FORWARD;
    if (r == STICK_LOW  && p == STICK_MID)  return GESTURE_LEFT;
    return GESTURE_NONE;
}

static bool isMoving(void)
{
    return s_state == FLIGHT_STATE_MOVE_FORWARD || s_state == FLIGHT_STATE_MOVE_LEFT;
}

static bool isHolding(void)
{
    return s_state == FLIGHT_STATE_ALT_HOLD || s_state == FLIGHT_STATE_POS_HOLD || isMoving();
}

static void resetGesture(void)
{
    s_pendingGesture = GESTURE_NONE;
    s_gestureTimer   = 0.0f;
}

static void goDisarmed(void)
{
    Timer5_Stop();
    s_btArmed        = false;
    s_btArmTimer     = 0.0f;
    s_state          = FLIGHT_STATE_DISARMED;
    s_holdState      = FLIGHT_STATE_ALT_HOLD;
    s_targetAltitude = 0.0f;
    s_lastThr        = STICK_NONE;
    s_armTimer       = 0.0f;
    s_disarmTimer    = 0.0f;
    s_gestureLatched = false;
    resetGesture();
}

static void enterHold(bool navOk)
{
    Timer5_Stop();
    if (s_holdState == FLIGHT_STATE_POS_HOLD && !navOk) s_holdState = FLIGHT_STATE_ALT_HOLD;
    s_state = s_holdState;
}

static void applyThrottle(StickPos_e thr, float currentAltitude, bool navOk)
{
    switch (thr) {
        case STICK_HIGH:
            if (!s_altOk) break;
            if (s_state == FLIGHT_STATE_CLIMB) {
                s_targetAltitude += CLIMB_STEP_METERS;
            } else {
                if (s_state == FLIGHT_STATE_ALT_HOLD || s_state == FLIGHT_STATE_POS_HOLD) s_holdState = s_state;
                if (isMoving()) s_holdState = FLIGHT_STATE_POS_HOLD;
                Timer5_Stop();
                s_targetAltitude = currentAltitude + CLIMB_STEP_METERS;
                s_state = FLIGHT_STATE_CLIMB;
            }
            break;

        case STICK_LOW:
            if (FlightState_IsAirborne(s_state)) {
                Timer5_Stop();
                s_state = FLIGHT_STATE_LAND;
                s_targetAltitude = 0.0f;
            }
            break;

        case STICK_MID:
            if (s_state == FLIGHT_STATE_CLIMB || s_state == FLIGHT_STATE_LAND) {
                s_targetAltitude = currentAltitude;
                enterHold(navOk);
            }
            break;

        default:
            break;
    }
}

static void applyGesture(Gesture_e g, bool navOk)
{
    switch (g) {
        case GESTURE_ALT_HOLD:
            Timer5_Stop();
            s_state = s_holdState = FLIGHT_STATE_ALT_HOLD;
            break;

        case GESTURE_POS_HOLD:
            if (!navOk) break;
            Timer5_Stop();
            s_state = s_holdState = FLIGHT_STATE_POS_HOLD;
            break;

        case GESTURE_FORWARD:
        case GESTURE_LEFT:
            if (!navOk || isMoving()) break;
            s_state = (g == GESTURE_FORWARD) ? FLIGHT_STATE_MOVE_FORWARD : FLIGHT_STATE_MOVE_LEFT;
            Timer5_StartOneShot(MOVE_DURATION_MS);
            break;

        default:
            break;
    }
}

void FlightState_Init(void)
{
    goDisarmed();
}

void FlightState_Update(float currentAltitude, bool navOk, bool altOk, float dT)
{
    float dtMs = dT * 1000.0f;
    s_altOk = altOk;

    if (s_btArmed) {
        s_btArmTimer += dtMs;
        if (s_btArmTimer >= BT_BENCH_TIMEOUT_MS) goDisarmed();
        return;
    }

    if (!SBUS_IsConnected() || SBUS_IsFailsafe()) {
        goDisarmed();
        return;
    }

    uint16_t roll  = SBUS_GetChannel(CH_ROLL);
    uint16_t pitch = SBUS_GetChannel(CH_PITCH);
    uint16_t thrUs = SBUS_GetChannel(CH_THROTTLE);
    uint16_t yawUs = SBUS_GetChannel(CH_YAW);

    StickPos_e thr = stickPos(thrUs);
    StickPos_e yaw = stickPos(yawUs);

    if (s_state != FLIGHT_STATE_DISARMED && thr == STICK_LOW && yaw == STICK_LOW) {
        s_disarmTimer += dtMs;
        if (s_disarmTimer >= DISARM_HOLD_TIME_MS) {
            goDisarmed();
            return;
        }
    } else {
        s_disarmTimer = 0.0f;
    }

    if (s_state == FLIGHT_STATE_DISARMED) {
        if (thr == STICK_LOW && yaw == STICK_HIGH) {
            s_armTimer += dtMs;
            if (s_armTimer >= ARM_HOLD_TIME_MS) {
                s_state          = FLIGHT_STATE_ARMED_IDLE;
                s_holdState      = FLIGHT_STATE_ALT_HOLD;
                s_targetAltitude = 0.0f;
                s_armTimer       = 0.0f;
                s_lastThr        = thr;
            }
        } else {
            s_armTimer = 0.0f;
        }
        return;
    }

    if (s_state == FLIGHT_STATE_LAND_BLIND) {
        s_lastThr = thr;
        if (altOk) {
            s_targetAltitude = currentAltitude;
            s_state = s_holdState = FLIGHT_STATE_ALT_HOLD;
        }
        return;
    }
    if (!altOk && FlightState_IsAirborne(s_state)) {
        Timer5_Stop();
        s_lastThr = thr;
        s_state = FLIGHT_STATE_LAND_BLIND;
        return;
    }

    if (thr != STICK_NONE && thr != s_lastThr) {
        applyThrottle(thr, currentAltitude, navOk);
    }
    s_lastThr = thr;

    Gesture_e g = readGesture(roll, pitch);
    if (g == GESTURE_CENTER) {
        s_gestureLatched = false;
        resetGesture();
    } else if (thr == STICK_MID && isHolding() && !s_gestureLatched && g != GESTURE_NONE) {
        if (g != s_pendingGesture) {
            s_pendingGesture = g;
            s_gestureTimer   = 0.0f;
        }
        s_gestureTimer += dtMs;
        if (s_gestureTimer >= GESTURE_HOLD_TIME_MS) {
            applyGesture(g, navOk);
            s_gestureLatched = true;
            resetGesture();
        }
    } else {
        resetGesture();
    }

    switch (s_state) {
        case FLIGHT_STATE_CLIMB:
            if (currentAltitude >= s_targetAltitude - 0.1f) enterHold(navOk);
            break;

        case FLIGHT_STATE_LAND:
            if (currentAltitude <= GROUND_ALTITUDE_EPS) {
                s_state = FLIGHT_STATE_ARMED_IDLE;
                s_targetAltitude = 0.0f;
            }
            break;

        case FLIGHT_STATE_MOVE_FORWARD:
        case FLIGHT_STATE_MOVE_LEFT:
            if (!navOk) {
                Timer5_Stop();
                s_state = s_holdState = FLIGHT_STATE_ALT_HOLD;
            } else if (Timer5_IsExpired()) {
                Timer5_Stop();
                s_state = s_holdState = FLIGHT_STATE_POS_HOLD;
            }
            break;

        case FLIGHT_STATE_POS_HOLD:
            if (!navOk) s_state = s_holdState = FLIGHT_STATE_ALT_HOLD;
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

bool FlightState_IsAirborne(FlightState_e state)
{
    return state == FLIGHT_STATE_CLIMB        ||
           state == FLIGHT_STATE_ALT_HOLD     ||
           state == FLIGHT_STATE_POS_HOLD     ||
           state == FLIGHT_STATE_LAND         ||
           state == FLIGHT_STATE_MOVE_FORWARD ||
           state == FLIGHT_STATE_MOVE_LEFT    ||
           state == FLIGHT_STATE_LAND_BLIND;
}

const char *FlightState_Name(FlightState_e state)
{
    switch (state) {
        case FLIGHT_STATE_DISARMED:     return "DISARMED";
        case FLIGHT_STATE_ARMED_IDLE:   return "IDLE";
        case FLIGHT_STATE_CLIMB:        return "CLIMB";
        case FLIGHT_STATE_ALT_HOLD:     return "ALTHOLD";
        case FLIGHT_STATE_POS_HOLD:     return "POSHOLD";
        case FLIGHT_STATE_LAND:         return "LAND";
        case FLIGHT_STATE_MOVE_FORWARD: return "FORWARD";
        case FLIGHT_STATE_MOVE_LEFT:    return "LEFT";
        case FLIGHT_STATE_LAND_BLIND:   return "LANDBLIND";
        default:                        return "?";
    }
}

void FlightState_BluetoothBenchArm(bool on)
{
    if (!on) {
        goDisarmed();
        return;
    }
    if (s_state != FLIGHT_STATE_DISARMED) return;
    goDisarmed();
    s_state      = FLIGHT_STATE_ARMED_IDLE;
    s_btArmed    = true;
    s_btArmTimer = 0.0f;
}

bool FlightState_IsBluetoothArmed(void)
{
    return s_btArmed;
}
