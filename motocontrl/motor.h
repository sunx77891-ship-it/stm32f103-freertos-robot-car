/*
 * motor.h - 电机驱动模块 (A4950)
 *
 * 硬件:
 *   电机A: TIM1_CH1(PA8) -> A_IN1,  TIM1_CH2(PA9)  -> A_IN2
 *   电机B: TIM1_CH3(PA10) -> B_IN1, TIM1_CH4(PA11) -> B_IN2
 *   TIM1: PSC=0, ARR=3599, PWM频率=20kHz
 *
 * 接口说明:
 *   PID 运算后直接调用 Motor_SetBoth(val_a, val_b)
 *   正数 -> 正转, 负数 -> 反转, 0 -> 停止
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

/* TIM1 周期 (与 CubeMX 配置一致) */
#define MOTOR_ARR           3599

/* 电机编号 */
typedef enum {
    MOTOR_A = 0,
    MOTOR_B = 1
} MotorId_t;

/* 电机方向 */
typedef enum {
    MOTOR_STOP    = 0,  /* 惯性停止, 全部输出低 */
    MOTOR_FORWARD,      /* 正转 */
    MOTOR_BACKWARD,     /* 反转 */
    MOTOR_BRAKE         /* 刹车, 全部输出高 */
} MotorDir_t;

/*---------- 用户调用接口 ----------*/

/* 初始化: 启动 TIM1 四路 PWM 输出, 电机停止 */
void Motor_Init(void);

/* 设置单个电机方向和占空比
   duty: 0~3599, 对应 0%~100% */
void Motor_Set(MotorId_t motor, MotorDir_t dir, uint16_t duty);

/* PID 直接调用: 同时设置两个电机
   val_a, val_b: 正=正转, 负=反转, 0=停止, 数值范围 -3599~+3599 */
void Motor_SetBoth(int16_t val_a, int16_t val_b);

/*---------- 保护功能 ----------*/

/* 紧急停止: 立即切断所有电机输出 */
void Motor_Stop(void);

/* 设置最大占空比限制 (保护电机/驱动)
   默认 MOTOR_ARR, 调用 Motor_SetMaxDuty 后生效 */
void Motor_SetMaxDuty(uint16_t max_duty);

#endif /* __MOTOR_H */
