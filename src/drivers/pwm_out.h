#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <stdint.h>

#define MOTOR_COUNT     4U
#define MOTOR_MIN_US    1000U    
#define MOTOR_MAX_US    2000U
#define MOTOR_PWM_HZ    400U      

/* Ngõ ra motor = pad S3..S6 trên mạch (PWM3..PWM6):
 *   M1 → S3 PB15 TIM1_CH3N    M2 → S4 PA8  TIM1_CH1
 *   M3 → S5 PB11 TIM2_CH4     M4 → S6 PB10 TIM2_CH3
 * tim1_clk_hz: clock TIM1 (APB2 x2 = 168 MHz), tim2_clk_hz: clock TIM2 (APB1 x2 = 84 MHz) */
void Motor_Init(uint32_t tim1_clk_hz, uint32_t tim2_clk_hz);

void Motor_SetUs(uint8_t motor, uint16_t us);     
void Motor_SetNorm(uint8_t motor, float throttle); 
void Motor_StopAll(void);                          
#endif