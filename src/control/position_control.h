#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include "../estimation/position.h"

/* =========================================================
 * Vòng vị trí XY (10 Hz)
 *   sai số vị trí --P--> vận tốc mong muốn --PID--> gia tốc mong muốn
 *   → xoay về hệ heading của drone → roll/pitch mục tiêu (rad)
 *
 * Hệ body FLU: pitch dương = chúc mũi (lực đẩy nghiêng về trước),
 *              roll dương  = nghiêng phải.
 * ========================================================= */

#define POS_KP              0.8f    /* (m/s) / m */
#define POS_MAX_VEL         2.0f    /* m/s */
#define POS_VEL_KP          1.2f    /* (m/s^2) / (m/s) */
#define POS_VEL_KI          0.15f
#define POS_VEL_KD          0.0f
#define POS_MAX_ACCEL       3.0f    /* m/s^2 */
#define POS_MAX_TILT_DEG    20.0f

/* Đặt điểm giữ = vị trí hiện tại, reset tích phân */
void PositionControl_Reset(const PositionEstimate_t *est);
void PositionControl_Update(const PositionEstimate_t *est, float yawRad, float dT,
                            float *rollSpRad, float *pitchSpRad);

/* Bám vận tốc theo hướng mũi (m/s): velFwd dương = tiến, velLeft dương = sang trái.
 * Dùng cho FORWARD / LEFT; gọi PositionControl_Reset trước khi bắt đầu. */
void PositionControl_UpdateVelocity(const PositionEstimate_t *est, float velFwd, float velLeft,
                                    float yawRad, float dT, float *rollSpRad, float *pitchSpRad);

#endif
