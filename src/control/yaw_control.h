#ifndef YAW_CONTROL_H
#define YAW_CONTROL_H

/* =========================================================
 * Vòng yaw (25 Hz): giữ hướng mũi mong muốn.
 * Cần yaw → tốc độ đổi hướng; nhả cần → giữ hướng hiện tại.
 * Đầu ra: yaw mục tiêu (rad) để ghép vào quaternion mục tiêu.
 * Quy ước: yaw dương = quay trái (CCW nhìn từ trên).
 * ========================================================= */

#define YAW_MAX_RATE_DEGS   90.0f   /* tốc độ đổi hướng ở cần full */
#define YAW_MAX_LEAD_DEG    45.0f   /* target không được vượt quá yaw hiện tại bao nhiêu */

void  YawControl_Reset(float currentYawRad);
/* stickYaw: [-1, 1], dương = gạt phải (quay phải) */
float YawControl_Update(float stickYaw, float currentYawRad, float dT);
float YawControl_GetTarget(void);

#endif
