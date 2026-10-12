#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include "../estimation/position.h"

#define POS_KP              0.8f
#define POS_MAX_VEL         2.0f
#define POS_VEL_KP          1.2f
#define POS_VEL_KI          0.15f
#define POS_VEL_KD          0.0f
#define POS_MAX_ACCEL       3.0f
#define POS_MAX_TILT_DEG    20.0f

void PositionControl_Reset(const PositionEstimate_t *est);
void PositionControl_Update(const PositionEstimate_t *est, float yawRad, float dT,
                            float *rollSpRad, float *pitchSpRad);

void PositionControl_UpdateVelocity(const PositionEstimate_t *est, float velFwd, float velLeft,
                                    float yawRad, float dT, float *rollSpRad, float *pitchSpRad);

#endif
