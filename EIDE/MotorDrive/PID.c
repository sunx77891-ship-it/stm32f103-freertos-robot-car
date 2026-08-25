#include "PID.h"

/**************************************************************************
函数名：PID_Init
函数功能：初始化 PID 参数
入口参数：pid - PID 对象指针
          Kp, Ki, Kd - 比例、积分、微分增益
          out_max - 输出限幅绝对值
          int_max - 积分限幅绝对值（防饱和）
**************************************************************************/
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float out_max, float int_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->out_max = out_max;
    pid->int_max = int_max;
    PID_Reset(pid);
}

/**************************************************************************
函数名：PID_Update
函数功能：单步 PID 运算（位置式）
         输出 = Kp*e + Ki*∫e + Kd*Δe
入口参数：pid - PID 对象指针
          target - 目标值
          feedback - 反馈值
返回值：PID 输出值（已限幅）
**************************************************************************/
float PID_Update(PID_t *pid, float target, float feedback)
{
    float err = target - feedback;

    /* 积分 */
    pid->integral += err;
    if (pid->integral >  pid->int_max) pid->integral =  pid->int_max;
    if (pid->integral < -pid->int_max) pid->integral = -pid->int_max;

    /* 微分 */
    float derivative = err - pid->prev_err;
    pid->prev_err = err;

    /* 合成输出 */
    float out = pid->Kp * err + pid->Ki * pid->integral + pid->Kd * derivative;

    /* 输出限幅 */
    if (out >  pid->out_max) out =  pid->out_max;
    if (out < -pid->out_max) out = -pid->out_max;

    return out;
}

/**************************************************************************
函数名：PID_Reset
函数功能：清除积分和微分历史
**************************************************************************/
void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_err = 0.0f;
}
