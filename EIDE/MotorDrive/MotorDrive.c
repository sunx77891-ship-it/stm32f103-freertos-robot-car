#include "MotorDrive.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"

/**************************************************************************
函数名：MotorDrive_SetCcr
函数功能：设置两个电机的驱动 CCR，支持正转/刹车/反转
入口参数：ccr - PID 运算后的 CCR 值
          > 0 → 正转，PWM=ccr
          = 0 → 刹车
          < 0 → 反转，PWM=|ccr|
返回值：无
**************************************************************************/
void MotorDrive_SetCcr(int16_t ccr)
{
    uint16_t pwm;

    taskENTER_CRITICAL();

    if (ccr > 0)
    {
        pwm = (uint16_t)ccr;
        if (pwm > MOTOR_CCR_MAX) pwm = MOTOR_CCR_MAX;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
    }
    else if (ccr == 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MOTOR_CCR_BRAKE);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, MOTOR_CCR_BRAKE);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MOTOR_CCR_BRAKE);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, MOTOR_CCR_BRAKE);
    }
    else
    {
        pwm = (uint16_t)(-(int32_t)ccr);
        if (pwm > MOTOR_CCR_MAX) pwm = MOTOR_CCR_MAX;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }

    taskEXIT_CRITICAL();
}

/**************************************************************************
函数名：MotorDrive_SetMotorA
函数功能：单独控制电机 A，并按编码器正方向修正驱动极性
入口参数：ccr > 0 时 CH3=0、CH4=PWM
              = 0 时 CH3=CH4=刹车电平
              < 0 时 CH3=PWM、CH4=0
返回值：无
**************************************************************************/
void MotorDrive_SetMotorA(int16_t ccr)
{
    uint16_t pwm;

    taskENTER_CRITICAL();

    if (ccr > 0)
    {
        pwm = (uint16_t)ccr;
        if (pwm > MOTOR_CCR_MAX) pwm = MOTOR_CCR_MAX;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
    }
    else if (ccr == 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MOTOR_CCR_BRAKE);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, MOTOR_CCR_BRAKE);
    }
    else
    {
        pwm = (uint16_t)(-(int32_t)ccr);
        if (pwm > MOTOR_CCR_MAX) pwm = MOTOR_CCR_MAX;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }

    taskEXIT_CRITICAL();
}

/**************************************************************************
函数名：MotorDrive_SetMotorB
函数功能：单独控制电机 B（CH1=IN1, CH2=IN2）
入口参数：ccr > 0 正转  =0 刹车  <0 反转
**************************************************************************/
void MotorDrive_SetMotorB(int16_t ccr)
{
    uint16_t pwm;

    taskENTER_CRITICAL();

    if (ccr > 0)
    {
        pwm = (uint16_t)ccr;
        if (pwm > MOTOR_CCR_MAX) pwm = MOTOR_CCR_MAX;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
    }
    else if (ccr == 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MOTOR_CCR_BRAKE);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, MOTOR_CCR_BRAKE);
    }
    else
    {
        pwm = (uint16_t)(-(int32_t)ccr);
        if (pwm > MOTOR_CCR_MAX) pwm = MOTOR_CCR_MAX;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }

    taskEXIT_CRITICAL();
}
