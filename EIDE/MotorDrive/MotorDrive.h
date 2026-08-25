#ifndef MOTOR_DRIVE_H
#define MOTOR_DRIVE_H

#include <stdint.h>

/* TIM1 的 ARR，CCR 范围 0~3599 */
#define MOTOR_CCR_MAX    3599U
#define MOTOR_CCR_BRAKE  3599U

/*
   电机驱动总入口（由 PID 运算后调用）

   形参 ccr:
     > 0   → 正转，PWM=ccr，另一通道=0
     = 0   → 刹车，两个通道都给 3599
     < 0   → 反转，PWM=|ccr|，另一通道=0

   硬件接线:
     电机 B — PA8(IN1)=TIM1_CH1,  PA9(IN2)=TIM1_CH2
     电机 A — PA10(IN1)=TIM1_CH3, PA11(IN2)=TIM1_CH4
*/
void MotorDrive_SetCcr(int16_t ccr);
void MotorDrive_SetMotorA(int16_t ccr);  /* A 正向：CH3=0，CH4=PWM */
void MotorDrive_SetMotorB(int16_t ccr);  /* 仅控制电机 B：CH1(IN1), CH2(IN2) */

#endif /* MOTOR_DRIVE_H */
