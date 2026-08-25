/*
 * motor.c - 电机驱动模块实现 (A4950)
 *
 * A4950 真值表:
 *   IN1   IN2   OUT1 OUT2  功能
 *   0     0     Hi-Z Hi-Z  惯性停止
 *   1     0     H    L     正转
 *   0     1     L    H     反转
 *   1     1     L    L     刹车
 *
 * TIM1 PWM1 模式: CNT<CCR 输出高, CNT>=CCR 输出低
 *   正转: IN1=PWM(=CCR), IN2=0   -> CCR 越大转矩越大
 *   反转: IN1=0, IN2=PWM(=CCR)
 *   刹车: IN1=1(CCR=ARR), IN2=1(CCR=ARR)
 */

#include "motor.h"
#include "tim.h"       /* htim1 */

/* 当前最大占空比限制 (保护) */
static uint16_t g_max_duty = MOTOR_ARR;

/* TIM1 通道与电机的对应关系 */
static uint32_t const _ch_in1[] = { TIM_CHANNEL_1, TIM_CHANNEL_3 };  /* A, B */
static uint32_t const _ch_in2[] = { TIM_CHANNEL_2, TIM_CHANNEL_4 };  /* A, B */

/*----------------------------------------------------------*/
void Motor_Init(void)
{
    /* 启动 4 路 PWM 输出 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* 初始确保电机停止 */
    Motor_Stop();
}

/*----------------------------------------------------------*/
void Motor_Set(MotorId_t motor, MotorDir_t dir, uint16_t duty)
{
    uint32_t ch1 = _ch_in1[motor];
    uint32_t ch2 = _ch_in2[motor];

    /* 保护: 占空比限幅 */
    if (duty > g_max_duty)
        duty = g_max_duty;

    switch (dir)
    {
    case MOTOR_FORWARD:
        __HAL_TIM_SET_COMPARE(&htim1, ch1, duty);
        __HAL_TIM_SET_COMPARE(&htim1, ch2, 0);
        break;

    case MOTOR_BACKWARD:
        __HAL_TIM_SET_COMPARE(&htim1, ch1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, ch2, duty);
        break;

    case MOTOR_STOP:
        __HAL_TIM_SET_COMPARE(&htim1, ch1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, ch2, 0);
        break;

    case MOTOR_BRAKE:
        __HAL_TIM_SET_COMPARE(&htim1, ch1, MOTOR_ARR);
        __HAL_TIM_SET_COMPARE(&htim1, ch2, MOTOR_ARR);
        break;
    }
}

/*----------------------------------------------------------*/
void Motor_SetBoth(int16_t val_a, int16_t val_b)
{
    int16_t v[] = { val_a, val_b };

    for (int i = 0; i < 2; i++)
    {
        MotorDir_t dir;
        uint16_t duty;

        if (v[i] > 0) {
            dir  = MOTOR_FORWARD;
            duty = (uint16_t)v[i];
        } else if (v[i] < 0) {
            dir  = MOTOR_BACKWARD;
            duty = (uint16_t)(-v[i]);
        } else {
            dir  = MOTOR_STOP;
            duty = 0;
        }

        if (duty > g_max_duty)
            duty = g_max_duty;

        uint32_t ch1 = _ch_in1[i];
        uint32_t ch2 = _ch_in2[i];

        switch (dir)
        {
        case MOTOR_FORWARD:
            __HAL_TIM_SET_COMPARE(&htim1, ch1, duty);
            __HAL_TIM_SET_COMPARE(&htim1, ch2, 0);
            break;
        case MOTOR_BACKWARD:
            __HAL_TIM_SET_COMPARE(&htim1, ch1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, ch2, duty);
            break;
        case MOTOR_STOP:
            __HAL_TIM_SET_COMPARE(&htim1, ch1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, ch2, 0);
            break;
        default:
            break;
        }
    }
}

/*----------------------------------------------------------*/
void Motor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}

/*----------------------------------------------------------*/
void Motor_SetMaxDuty(uint16_t max_duty)
{
    g_max_duty = (max_duty > MOTOR_ARR) ? MOTOR_ARR : max_duty;
}
