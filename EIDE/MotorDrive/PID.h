#ifndef PID_H
#define PID_H

#include <stdint.h>

/* PID 控制器对象 */
typedef struct {
    float Kp, Ki, Kd;       /* 增益 */
    float integral;          /* 积分累计 */
    float prev_err;          /* 上次偏差（用于微分） */
    float out_max;           /* 输出限幅 */
    float int_max;           /* 积分限幅（抗饱和） */
} PID_t;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float out_max, float int_max);
float PID_Update(PID_t *pid, float target, float feedback);
void PID_Reset(PID_t *pid);

#endif
