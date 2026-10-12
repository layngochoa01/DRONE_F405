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

#define PERIOD_25HZ_MS   40U
#define PERIOD_10HZ_MS   100U
#define DT_25HZ          (PERIOD_25HZ_MS / 1000.0f)
#define DT_10HZ          (PERIOD_10HZ_MS / 1000.0f)

#define MAG_TIMEOUT_MS   200U

/* Dưới đất: kéo áp suất gốc về áp suất đo (mỗi mẫu baro ~8 Hz, τ ≈ 2.5 s) */
#define BARO_NUDGE_ALPHA 0.05f

/* LAND_BLIND: hạ ga tuyến tính từ mức đang bay về BLIND_THROTTLE trong BLIND_RAMP_MS */
#define BLIND_THROTTLE   0.30f
#define BLIND_RAMP_MS    10000.0f

#define RAD2DEG          (180.0f / M_PIf)
#define DEG2RAD          (M_PIf / 180.0f)

/* =========================================================
 * Dữ liệu dùng chung giữa vòng nhanh (ngắt) và vòng chậm (main)
 * ========================================================= */

/* Main → ngắt: setpoint và cờ. Main ghi trong vùng găng, ngắt chỉ đọc. */
typedef struct {
    float  rollSp, pitchSp, yawSp;   /* rad */
    float  throttle;                 /* 0..1 */
    bool   runRate;                  /* đang bay / BENCH → chạy rate PID */
    bool   motorsEnabled;
    bool   testActive;               /* motor test */
    int8_t testMotor;
    bool   magValid;
    Vec3_t mag;
} FastCmd_t;

/* Ngắt → main: gia tốc hệ trái đất cộng dồn. Main lấy ra trong vùng găng. */
typedef struct {
    float    sumZ, sumN, sumW;   /* m/s², Z đã trừ g */
    float    sumDt;              /* s, thời gian thực */
    uint32_t n;
} AccelAccum_t;

static FastCmd_t       s_cmd;
static AccelAccum_t    s_acc;
static volatile float  s_fastYaw     = 0.0f;   /* rad */
static volatile float  s_fastCosTilt = 1.0f;

/* Đo tải vòng nhanh (đơn vị chu kỳ CPU) */
static uint32_t s_lastSampleCyc = 0;
static uint32_t s_execCyc = 0, s_execMaxCyc = 0;
static uint32_t s_periodMinCyc = 0xFFFFFFFFU, s_periodMaxCyc = 0;

/* =========================================================
 * Trạng thái của vòng chậm
 * ========================================================= */
static bool     s_baroOk = false;
static bool     s_magOk  = false;

static Vec3_t   s_mag;
static Compass_Data_t s_compass;   /* bản đọc gần nhất (debug) */
static bool     s_hasMag = false;
static uint32_t s_lastMagMs = 0;

static uint32_t s_next25 = 0, s_nextMag = 0, s_next10 = 0;

static float    s_throttle = 0.0f;
static float    s_rollSp = 0.0f, s_pitchSp = 0.0f, s_yawSp = 0.0f;   /* rad */
static FlightState_e s_prevState = FLIGHT_STATE_DISARMED;
static bool     s_gpsValid = false;

static float    s_blindStartThr = 0.0f;
static uint32_t s_blindStartMs  = 0;

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

/* Đến lượt chạy tác vụ chu kỳ period chưa; tụt hậu quá 1 chu kỳ thì bắt nhịp lại */
static bool due(uint32_t now, uint32_t *next, uint32_t period)
{
    if ((int32_t)(now - *next) < 0) return false;
    *next += period;
    if ((int32_t)(now - *next) >= 0) *next = now + period;
    return true;
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

static bool onGround(FlightState_e state)
{
    return (state == FLIGHT_STATE_DISARMED || state == FLIGHT_STATE_ARMED_IDLE) && !s_bench;
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

static void readBaro(uint32_t now, bool ground)
{
    SPL06_Data_t baro;
    if (!SPL06_Update(&baro)) return;

    float press = Baro_MovingAverage(baro.pressPa);
    if (ground) Altitude_NudgeBaseline(press, BARO_NUDGE_ALPHA);
    AltitudeKF_UpdateBaro(Altitude_PressureToMeters(press), now);
}

void FlightControl_Init(bool baroOk, bool magOk)
{
    s_baroOk = baroOk;
    s_magOk  = magOk;
    s_hasMag = false;

    Attitude_Init(FC_DT);
    Position_Init();
    FlightState_Init();
    AltitudeControl_Init();
    YawControl_Reset(0.0f);
    RateControl_Init(FC_DT);
    MotorMixer_Init();

    s_throttle = 0.0f;
    s_rollSp = s_pitchSp = s_yawSp = 0.0f;
    s_prevState = FLIGHT_STATE_DISARMED;

    s_cmd = (FastCmd_t){ 0 };
    s_cmd.testMotor = -1;
    s_acc = (AccelAccum_t){ 0 };

    uint32_t now = get_tick();
    s_next25  = now;
    s_nextMag = now + PERIOD_25HZ_MS / 2U;   /* lệch pha với baro để rải tải I2C */
    s_next10  = now;
}

/* =========================================================
 * VÒNG NHANH — 2 kHz, trong ngắt DMA
 * ========================================================= */
void FlightControl_FastUpdate(const ICM42605_Data *data)
{
    uint32_t t0 = cycle_now();

    /* Chu kỳ thực giữa 2 mẫu */
    float dt = FC_DT;
    if (s_lastSampleCyc != 0U) {
        uint32_t period = t0 - s_lastSampleCyc;
        if (period < s_periodMinCyc) s_periodMinCyc = period;
        if (period > s_periodMaxCyc) s_periodMaxCyc = period;
        dt = constrainf((float)period / (float)CPU_HZ, 0.25f * FC_DT, 4.0f * FC_DT);
    }
    s_lastSampleCyc = t0;

    FastCmd_t cmd = s_cmd;   /* main không chạy chen được vào ngắt → chép thẳng */

    /* ---- AHRS ---- */
    Attitude_Update(data, cmd.magValid ? &cmd.mag : NULL, dt);

    float rMat[3][3];
    Attitude_GetRotationMatrix(rMat);
    s_fastYaw     = atan2f(rMat[1][0], rMat[0][0]);
    s_fastCosTilt = rMat[2][2];

    /* ---- Cộng dồn gia tốc hệ trái đất (NWU) cho predict 100 Hz ---- */
    float ax = data->accel_x, ay = data->accel_y, az = data->accel_z;
    s_acc.sumZ  += Altitude_ComputeVerticalAccel(rMat, ax, ay, az);
    s_acc.sumN  += rMat[0][0]*ax + rMat[0][1]*ay + rMat[0][2]*az;
    s_acc.sumW  += rMat[1][0]*ax + rMat[1][1]*ay + rMat[1][2]*az;
    s_acc.sumDt += dt;
    s_acc.n++;

    /* ---- Tư thế + rate PID ---- */
    Vec3_t gyroDegS = { data->gyro_x, data->gyro_y, data->gyro_z };
    Vec3_t torque   = { 0.0f, 0.0f, 0.0f };

    if (cmd.runRate) {
        Quaternion_t qSetpoint;
        Quaternion_FromEuler(&qSetpoint, cmd.rollSp, cmd.pitchSp, cmd.yawSp);

        Vec3_t rateSpRad;
        AttitudeControl_Update(&rateSpRad, &qSetpoint, Quaternion_Get(),
                               FC_ATT_KP_ROLL, FC_ATT_KP_PITCH, FC_ATT_KP_YAW);

        Vec3_t rateSpDegS = {
            constrainf(rateSpRad.x * RAD2DEG, -FC_RATE_MAX_RP,  FC_RATE_MAX_RP),
            constrainf(rateSpRad.y * RAD2DEG, -FC_RATE_MAX_RP,  FC_RATE_MAX_RP),
            constrainf(rateSpRad.z * RAD2DEG, -FC_RATE_MAX_YAW, FC_RATE_MAX_YAW),
        };
        RateControl_Update(&torque, &rateSpDegS, &gyroDegS, FC_DT);
    } else {
        RateControl_Reset(&gyroDegS);   /* dưới đất: không tích lũy tích phân */
    }

    /* ---- Mixer → PWM ---- */
    MotorMixer_SetArmed(cmd.motorsEnabled);

    MotorOutput_t out;
    MotorMixer_Update(&out, cmd.throttle, &torque);

    if (cmd.testActive) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            bool on = (cmd.testMotor < 0) || (cmd.testMotor == (int8_t)i);
            out.pulseUs[i] = on ? (float)FC_MOTOR_TEST_US : (float)MOTOR_MIN_US;
        }
    }
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_SetUs(i, (uint16_t)out.pulseUs[i]);
        s_status.motorUs[i] = out.pulseUs[i];
    }

    uint32_t exec = cycle_now() - t0;
    s_execCyc = exec;
    if (exec > s_execMaxCyc) s_execMaxCyc = exec;
}

/* =========================================================
 * VÒNG CHẬM — main loop
 * ========================================================= */
void FlightControl_SlowUpdate(void)
{
    uint32_t now = get_tick();
    FlightState_e state = FlightState_Get();
    bool ground = onGround(state);

    /* ---- ~100 Hz: trung bình gia tốc → predict (dT = thời gian tích lũy thực) ---- */
    if (s_acc.n >= FC_ACCEL_BLOCK) {
        CRITICAL_BEGIN();
        AccelAccum_t a = s_acc;
        s_acc = (AccelAccum_t){ 0 };
        CRITICAL_END();

        float inv = 1.0f / (float)a.n;
        float azAvg = a.sumZ * inv;

        if (ground) AltitudeKF_GroundAnchor(azAvg);   /* neo h = 0, v = 0, học bias */
        else        AltitudeKF_Predict(azAvg, a.sumDt);

        Position_Predict(a.sumN * inv, a.sumW * inv, a.sumDt);
    }

    /* ---- GPS: update ước lượng vị trí khi có mẫu mới ---- */
    if (GPS_IsNewData()) Position_UpdateGps(GPS_GetData(), now);
    s_gpsValid = Position_IsValid(now);

    /* ---- Compass 25 Hz ---- */
    if (s_magOk && due(now, &s_nextMag, PERIOD_25HZ_MS)) readCompass(now);
    bool magHealthy = s_magOk && s_hasMag && ((now - s_lastMagMs) < MAG_TIMEOUT_MS);

    float yaw = s_fastYaw;

    /* ---- 25 Hz: baro, STATE, vòng độ cao, vòng yaw ---- */
    if (due(now, &s_next25, PERIOD_25HZ_MS)) {
        if (s_baroOk) readBaro(now, ground);

        const AltitudeKF_State_t *alt = AltitudeKF_Get();
        bool altOk = s_baroOk && AltitudeKF_IsHealthy(now);
        bool navOk = s_gpsValid && magHealthy;

        FlightState_Update(alt->altitude, navOk, altOk, DT_25HZ);
        state  = FlightState_Get();
        ground = onGround(state);

        if (state != FLIGHT_STATE_ARMED_IDLE) s_bench = false;   /* BENCH chỉ sống ở IDLE */
        bool airborne = FlightState_IsAirborne(state) || s_bench;
        bool entered  = (state != s_prevState);
        s_prevState   = state;

        if (!airborne) {
            YawControl_Reset(yaw);
            s_yawSp = yaw;
            s_rollSp = s_pitchSp = 0.0f;
        }

        /* Vòng độ cao → ga (bù nghiêng) */
        if (state == FLIGHT_STATE_LAND_BLIND) {
            if (entered) {
                s_blindStartThr = s_throttle;
                s_blindStartMs  = now;
            }
            float k   = constrainf((float)(now - s_blindStartMs) / BLIND_RAMP_MS, 0.0f, 1.0f);
            float end = fminf(s_blindStartThr, BLIND_THROTTLE);
            s_throttle = s_blindStartThr + (end - s_blindStartThr) * k;
            AltitudeControl_SeedThrottle(s_throttle);
        } else {
            s_throttle = AltitudeControl_Update(state, FlightState_GetTargetAltitude(),
                                                alt->altitude, alt->velocity,
                                                s_fastCosTilt, DT_25HZ);
        }
        if (s_bench) s_throttle = FC_BENCH_THROTTLE;

        /* Vòng yaw; khi LAND / LAND_BLIND cần yaw dành cho cử chỉ DISARM */
        if (airborne) {
            bool yawStickFree = (state != FLIGHT_STATE_LAND) && (state != FLIGHT_STATE_LAND_BLIND);
            s_yawSp = YawControl_Update(yawStickFree ? rcStick(FC_CH_YAW) : 0.0f, yaw, DT_25HZ);
        }

        /* Vào POS_HOLD / FORWARD / LEFT: chốt điểm giữ, chạy XY ngay lượt này */
        if (entered && (state == FLIGHT_STATE_POS_HOLD ||
                        state == FLIGHT_STATE_MOVE_FORWARD ||
                        state == FLIGHT_STATE_MOVE_LEFT)) {
            PositionControl_Reset(Position_Get());
            s_next10 = now;
        }
    }

    /* ---- 10 Hz: vòng vị trí XY → roll, pitch ---- */
    switch (state) {
        case FLIGHT_STATE_POS_HOLD:
            if (due(now, &s_next10, PERIOD_10HZ_MS))
                PositionControl_Update(Position_Get(), yaw, DT_10HZ, &s_rollSp, &s_pitchSp);
            break;

        case FLIGHT_STATE_MOVE_FORWARD:
        case FLIGHT_STATE_MOVE_LEFT:
            if (due(now, &s_next10, PERIOD_10HZ_MS)) {
                float vFwd  = (state == FLIGHT_STATE_MOVE_FORWARD) ? MOVE_SPEED_MS : 0.0f;
                float vLeft = (state == FLIGHT_STATE_MOVE_LEFT)    ? MOVE_SPEED_MS : 0.0f;
                PositionControl_UpdateVelocity(Position_Get(), vFwd, vLeft, yaw, DT_10HZ,
                                               &s_rollSp, &s_pitchSp);
            }
            break;

        default:   /* CLIMB, ALT_HOLD, LAND, LAND_BLIND, dưới đất: giữ thăng bằng */
            s_rollSp = s_pitchSp = 0.0f;
            break;
    }

    /* ---- Motor test: chỉ khi DISARMED, tự dừng sau FC_MOTOR_TEST_MS ---- */
    if (s_testActive && (state != FLIGHT_STATE_DISARMED || (now - s_testStartMs) >= FC_MOTOR_TEST_MS)) {
        s_testActive = false;
    }

    /* ---- Gửi setpoint cho vòng nhanh ---- */
    FastCmd_t cmd = {
        .rollSp        = s_rollSp,
        .pitchSp       = s_pitchSp,
        .yawSp         = s_yawSp,
        .throttle      = s_throttle,
        .runRate       = FlightState_IsAirborne(state) || s_bench,
        .motorsEnabled = FlightState_MotorsEnabled(),
        .testActive    = s_testActive,
        .testMotor     = s_testMotor,
        .magValid      = magHealthy,
        .mag           = s_mag,
    };
    CRITICAL_BEGIN();
    s_cmd = cmd;
    CRITICAL_END();

    /* ---- Trạng thái cho telemetry ---- */
    s_status.rollSpDeg  = s_rollSp  * RAD2DEG;
    s_status.pitchSpDeg = s_pitchSp * RAD2DEG;
    s_status.yawSpDeg   = s_yawSp   * RAD2DEG;
    s_status.throttle   = s_throttle;
    s_status.state      = state;
    s_status.magHealthy = magHealthy;
    s_status.gpsValid   = s_gpsValid;
}

const FlightControl_Status_t *FlightControl_GetStatus(void)
{
    return &s_status;
}

void FlightControl_GetPerf(FlightControl_Perf_t *out, bool resetMax)
{
    CRITICAL_BEGIN();
    uint32_t exec = s_execCyc, execMax = s_execMaxCyc;
    uint32_t pMin = s_periodMinCyc, pMax = s_periodMaxCyc;
    if (resetMax) {
        s_execMaxCyc   = 0;
        s_periodMinCyc = 0xFFFFFFFFU;
        s_periodMaxCyc = 0;
    }
    CRITICAL_END();

    const float cycPerUs = (float)CPU_HZ / 1e6f;
    out->execUs      = (float)exec    / cycPerUs;
    out->execMaxUs   = (float)execMax / cycPerUs;
    out->periodMinUs = (pMin == 0xFFFFFFFFU) ? 0.0f : (float)pMin / cycPerUs;
    out->periodMaxUs = (float)pMax / cycPerUs;
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
