#ifndef YAW_CONTROL_H
#define YAW_CONTROL_H

#define YAW_MAX_RATE_DEGS   90.0f
#define YAW_MAX_LEAD_DEG    45.0f

void  YawControl_Reset(float currentYawRad);
float YawControl_Update(float stickYaw, float currentYawRad, float dT);

#endif
