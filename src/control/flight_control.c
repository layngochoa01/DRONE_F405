#include "flight_control.h"
#include <math.h>
#include "system.h"
#include "sbus.h"
#include "pwm_out.h"
#include "spl06.h"
#include "hmc5883l.h"
#include "gps.h"
#include "attitude.h"
#include "altitude.h"
#include "position.h"
#include "flight_state.h"
#include "altitude_control.h"
#include "position_control.h"
#include "yaw_control.h"
#include "attitude_control.h"
#include "rate_control.h"
#include "motor_mixer.h"

#define DIV_25HZ        (FC_LOOP_HZ / 25U)
#define DIV_10HZ        (FC_LOOP_HZ / 10U)
#define DT_25HZ         (1.0f / 25.0f)
#define DT_10HZ         (1.0f / 10.0f)

#define MAG_TIMEOUT_MS  200U

#define RAD2DEG         (180.0f / M_PIf)
#define DEG2RAD         (M_PIf / 180.0f)

static uint32_t s_cycle = 0;
static bool     s_baroOk = false;
static bool     s_magOk  = false;

static Vec3_t   s_mag;
static Compass_Data_t s_compass;   /* bản đọc gần nhất (debug) */
static bool     s_hasMag = false;
static uint32_t s_lastMagMs = 0;

static float    s_throttle = 0.0f;
static float    s_rollSp = 0.0f, s_pitchSp = 0.0f, s_yawSp = 0.0f;   /* rad */
static FlightState_e s_prevState = FLIGHT_STATE_DISARMED;

static FlightControl_Status_t s_status;

static bool     s_bench = false;
static bool     s_testActive = false;
static int8_t   s_testMotor  = -1;
static uint32_t s_testStartMs = 0;

static float constrainf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 1000..2000 µs → [-1, 1] có vùng chết; mất RC → 0 */
static float rcStick(uint8_t ch)
{
    if (!SBUS_IsConnected() || SBUS_IsFailsafe()) return 0.0f;

    float v = ((float)SBUS_GetChannel(ch) - 1500.0f) / 500.0f;
    if (fabsf(v) < FC_RC_DEADBAND) return 0.0f;
    v = (v > 0.0f) ? (v - FC_RC_DEADBAND) : (v + FC_RC_DEADBAND);
    return constrainf(v / (1.0f - FC_RC_DEADBAND), -1.0f, 1.0f);
}

static void readCompass(uint32_t now)
{
    Compass_Data_t c;
    if (Compass_Update(&c)) {
        s_compass = c;
        s_mag = (Vec3_t){ c.x, c.y, c.z };
        s_lastMagMs = now;
        s_hasMag = true;
    }
}

static void readBaro(void)
{
    SPL06_Data_t baro;
    if (SPL06_Update(&baro)) {
        float press = Baro_MovingAverage(baro.pressPa);
        AltitudeKF_UpdateBaro(Altitude_PressureToMeters(press));
    }
}

void FlightControl_Init(bool baroOk, bool magOk)
{
    s_baroOk = baroOk;
    s_magOk  = magOk;
    s_cycle  = 0;
    s_hasMag = false;

    Attitude_Init(FC_DT);
    Position_Init();
    FlightState_Init();
    AltitudeControl_Init();
    YawControl_Reset(0.0f);
    RateControl_Init();
    MotorMixer_Init();

    s_throttle = 0.0f;
    s_rollSp = s_pitchSp = s_yawSp = 0.0f;
    s_prevState = FLIGHT_STATE_DISARMED;
}

void FlightControl_Update(const ICM42605_Data *data)
{
    uint32_t now   = get_tick();
    uint32_t phase = s_cycle++;
    bool slot25    = (phase % DIV_25HZ) == 0U;
    bool slot10    = (phase % DIV_10HZ) == 0U;

    /* ===================== ƯỚC LƯỢNG ===================== */

    /* Compass đọc lệch pha với baro để rải tải I2C */
    if (s_magOk && (phase % DIV_25HZ) == 2U) readCompass(now);
    bool magHealthy = s_magOk && s_hasMag && ((now - s_lastMagMs) < MAG_TIMEOUT_MS);

    /* AHRS (quaternion): gyro + accel + HMC5883L */
    Attitude_Update(data, magHealthy ? &s_mag : NULL, FC_DT);

    float rMat[3][3];
    Attitude_GetRotationMatrix(rMat);
    float yaw = Attitude_GetYawRad();

    /* Gia tốc hệ trái đất (NWU) cho ước lượng độ cao và vị trí */
    float ax = data->accel_x, ay = data->accel_y, az = data->accel_z;
    AltitudeKF_Predict(Altitude_ComputeVerticalAccel(rMat, ax, ay, az), FC_DT);
    float accN = rMat[0][0]*ax + rMat[0][1]*ay + rMat[0][2]*az;
    float accW = rMat[1][0]*ax + rMat[1][1]*ay + rMat[1][2]*az;
    Position_Predict(accN, accW, FC_DT);

    if (GPS_IsNewData()) Position_UpdateGps(GPS_GetData(), now);
    bool gpsValid = Position_IsValid(now);

    if (slot25 && s_baroOk) readBaro();   /* SPL06 cập nhật 8 Hz, poll 25 Hz */

    const AltitudeKF_State_t *alt = AltitudeKF_Get();

    /* ===================== STATE ===================== */

    bool navOk = gpsValid && magHealthy;
    if (slot25) FlightState_Update(alt->altitude, navOk, DT_25HZ);
    FlightState_e state = FlightState_Get();
    if (state != FLIGHT_STATE_ARMED_IDLE) s_bench = false;   /* BENCH chỉ sống ở ARMED_IDLE */
    bool airborne = FlightState_IsAirborne(state) || s_bench;
    bool entered  = (state != s_prevState);   /* vừa đổi trạng thái */
    s_prevState   = state;

    /* Cần yaw xoay mũi khi đang bay; khi LAND cần yaw dùng cho cử chỉ DISARM */
    float stickYaw = (state == FLIGHT_STATE_LAND) ? 0.0f : rcStick(FC_CH_YAW);

    Vec3_t gyroDegS = { data->gyro_x, data->gyro_y, data->gyro_z };

    if (!airborne) {
        /* Dưới đất: bám theo trạng thái hiện tại, không tích lũy tích phân */
        YawControl_Reset(yaw);
        RateControl_Reset(&gyroDegS);
        s_yawSp = yaw;
        s_rollSp = s_pitchSp = 0.0f;
    }

    /* ===================== VÒNG NGOÀI ===================== */

    /* Vòng độ cao Z (25 Hz) → ga, đã bù nghiêng. Chạy trong MỌI trạng thái bay,
     * kể cả FORWARD / LEFT → độ cao không đổi khi di chuyển */
    if (slot25) {
        s_throttle = AltitudeControl_Update(state, FlightState_GetTargetAltitude(),
                                            alt->altitude, alt->velocity,
                                            rMat[2][2], DT_25HZ);
        if (s_bench) s_throttle = FC_BENCH_THROTTLE;
    }

    /* Vòng yaw (25 Hz) → hướng mũi mong muốn */
    if (slot25 && airborne) {
        s_yawSp = YawControl_Update(stickYaw, yaw, DT_25HZ);
    }

    /* Vòng vị trí XY (10 Hz) → roll/pitch theo trạng thái */
    const PositionEstimate_t *pos = Position_Get();
    switch (state) {
        case FLIGHT_STATE_POS_HOLD:
            if (entered) PositionControl_Reset(pos);   /* chốt điểm giữ tại vị trí hiện tại */
            if (slot10 || entered) PositionControl_Update(pos, yaw, DT_10HZ, &s_rollSp, &s_pitchSp);
            break;

        case FLIGHT_STATE_MOVE_FORWARD:
        case FLIGHT_STATE_MOVE_LEFT: {
            if (entered) PositionControl_Reset(pos);
            float vFwd  = (state == FLIGHT_STATE_MOVE_FORWARD) ? MOVE_SPEED_MS : 0.0f;
            float vLeft = (state == FLIGHT_STATE_MOVE_LEFT)    ? MOVE_SPEED_MS : 0.0f;
            if (slot10 || entered) PositionControl_UpdateVelocity(pos, vFwd, vLeft, yaw, DT_10HZ, &s_rollSp, &s_pitchSp);
            break;
        }

        default:   /* CLIMB, ALT_HOLD, LAND, dưới đất: giữ thăng bằng */
            s_rollSp = s_pitchSp = 0.0f;
            break;
    }

    /* ===================== VÒNG TƯ THẾ (100 Hz) ===================== */

    Quaternion_t qSetpoint;
    Quaternion_FromEuler(&qSetpoint, s_rollSp, s_pitchSp, s_yawSp);

    Vec3_t rateSpRad;
    AttitudeControl_Update(&rateSpRad, &qSetpoint, Quaternion_Get(),
                           FC_ATT_KP_ROLL, FC_ATT_KP_PITCH, FC_ATT_KP_YAW);

    Vec3_t rateSpDegS = {
        constrainf(rateSpRad.x * RAD2DEG, -FC_RATE_MAX_RP,  FC_RATE_MAX_RP),
        constrainf(rateSpRad.y * RAD2DEG, -FC_RATE_MAX_RP,  FC_RATE_MAX_RP),
        constrainf(rateSpRad.z * RAD2DEG, -FC_RATE_MAX_YAW, FC_RATE_MAX_YAW),
    };

    /* ===================== VÒNG RATE (100 Hz) ===================== */

    Vec3_t torque = { 0.0f, 0.0f, 0.0f };
    if (airborne) {
        RateControl_Update(&torque, &rateSpDegS, &gyroDegS, FC_DT);
    }

    /* ===================== MIXER → PWM ===================== */

    MotorMixer_SetArmed(FlightState_MotorsEnabled());

    MotorOutput_t out;
    MotorMixer_Update(&out, s_throttle, &torque);

    /* Motor test: chỉ khi DISARMED, tự dừng sau FC_MOTOR_TEST_MS */
    if (s_testActive) {
        if (state != FLIGHT_STATE_DISARMED || (now - s_testStartMs) >= FC_MOTOR_TEST_MS) {
            s_testActive = false;
        } else {
            for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
                bool on = (s_testMotor < 0) || (s_testMotor == (int8_t)i);
                out.pulseUs[i] = on ? (float)FC_MOTOR_TEST_US : (float)MOTOR_MIN_US;
            }
        }
    }
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_SetUs(i, (uint16_t)out.pulseUs[i]);
    }

    /* ===================== STATUS ===================== */

    s_status.rollSpDeg     = s_rollSp  * RAD2DEG;
    s_status.pitchSpDeg    = s_pitchSp * RAD2DEG;
    s_status.yawSpDeg      = s_yawSp   * RAD2DEG;
    s_status.throttle      = s_throttle;
    s_status.state         = state;
    s_status.magHealthy    = magHealthy;
    s_status.gpsValid      = gpsValid;
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) s_status.motorUs[i] = out.pulseUs[i];
}

const FlightControl_Status_t *FlightControl_GetStatus(void)
{
    return &s_status;
}

bool FlightControl_MotorTest(int8_t motorIndex)
{
    if (FlightState_Get() != FLIGHT_STATE_DISARMED) return false;
    if (motorIndex >= (int8_t)MOTOR_COUNT) return false;
    s_testMotor   = motorIndex;
    s_testStartMs = get_tick();
    s_testActive  = true;
    return true;
}

void FlightControl_MotorTestStop(void)
{
    s_testActive = false;
}

bool FlightControl_SetBench(bool on)
{
    if (on && FlightState_Get() != FLIGHT_STATE_ARMED_IDLE) return false;
    s_bench = on;
    return true;
}

bool FlightControl_IsBench(void)
{
    return s_bench;
}

const Compass_Data_t *FlightControl_GetCompass(uint32_t *ageMs)
{
    if (ageMs) *ageMs = s_hasMag ? (get_tick() - s_lastMagMs) : 0xFFFFFFFFU;
    return &s_compass;
}
