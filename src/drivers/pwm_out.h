#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <stdint.h>

#define MOTOR_COUNT     4U
#define MOTOR_MIN_US    1000U
#define MOTOR_MAX_US    2000U
#define MOTOR_PWM_HZ    400U

void Motor_Init(uint32_t tim1_clk_hz, uint32_t tim2_clk_hz);

void Motor_SetUs(uint8_t motor, uint16_t us);
void Motor_StopAll(void);
#endif
