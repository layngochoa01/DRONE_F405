#include "flight_state.h"
#include <stdbool.h>

static FlightState_e s_state = FLIGHT_STATE_DISARMED;
static float s_targetAltitude = 0.0f;

void FlightState_Init(void)
{
    s_state = FLIGHT_STATE_DISARMED;
    s_targetAltitude = 0.0f;
}

void FlightState_HandleCommand(FlightCommand_e cmd, float currentAltitude)
{
    /* CMD_DISARM luôn được xử lý trước tiên, bất kể đang ở trạng thái nào - an toàn khẩn cấp */
    if (cmd == CMD_DISARM) {
        s_state = FLIGHT_STATE_DISARMED;
        s_targetAltitude = 0.0f;
        return;
    }

    switch (s_state) {
        case FLIGHT_STATE_DISARMED:
            if (cmd == CMD_ARM) {
                s_state = FLIGHT_STATE_ARMED_IDLE;
                s_targetAltitude = 0.0f;
            }
            /* các lệnh khác bị bỏ qua khi đang DISARMED - không cho bay nếu chưa arm */
            break;

        case FLIGHT_STATE_ARMED_IDLE:
            if (cmd == CMD_CLIMB_STEP) {
                s_state = FLIGHT_STATE_CLIMB;
                s_targetAltitude = currentAltitude + CLIMB_STEP_METERS;
            } else if (cmd == CMD_HOLD) {
                s_state = FLIGHT_STATE_ATTITUDE_HOLD;
                s_targetAltitude = currentAltitude;
            }
            break;

        case FLIGHT_STATE_ATTITUDE_HOLD:
            if (cmd == CMD_CLIMB_STEP) {
                s_state = FLIGHT_STATE_CLIMB;
                s_targetAltitude = currentAltitude + CLIMB_STEP_METERS;
            } else if (cmd == CMD_LAND) {
                s_state = FLIGHT_STATE_LAND;
            }
            break;

        case FLIGHT_STATE_CLIMB:
            if (cmd == CMD_CLIMB_STEP) {
                s_targetAltitude += CLIMB_STEP_METERS;   /* cộng dồn thêm 1m mỗi lần gửi */
            } else if (cmd == CMD_HOLD) {
                s_state = FLIGHT_STATE_ATTITUDE_HOLD;
                s_targetAltitude = currentAltitude;
            } else if (cmd == CMD_LAND) {
                s_state = FLIGHT_STATE_LAND;
            }
            break;

        case FLIGHT_STATE_LAND:
            /* trong khi đang hạ, chỉ nhận lệnh HOLD để hủy hạ cánh giữa chừng nếu cần */
            if (cmd == CMD_HOLD) {
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
            /* Đã đạt độ cao mục tiêu -> tự chuyển sang giữ yên (ATTITUDE_HOLD) */
            if (currentAltitude >= s_targetAltitude - 0.1f) {
                s_state = FLIGHT_STATE_ATTITUDE_HOLD;
            }
            break;

        case FLIGHT_STATE_LAND:
            s_targetAltitude -= LAND_DESCENT_RATE * dT;
            if (s_targetAltitude < 0.0f) s_targetAltitude = 0.0f;

            /* Chạm đất thật (dựa vào altitude đo được, không chỉ setpoint) -> tự chuyển ARMED_IDLE */
            if (currentAltitude <= GROUND_ALTITUDE_EPS) {
                s_state = FLIGHT_STATE_ARMED_IDLE;
                s_targetAltitude = 0.0f;
            }
            break;

        default:
            break;   /* DISARMED, ARMED_IDLE, ATTITUDE_HOLD: setpoint không tự đổi, chờ lệnh mới */
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