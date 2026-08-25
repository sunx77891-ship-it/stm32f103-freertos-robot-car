#ifndef CTRL_H
#define CTRL_H

#include <stdint.h>

/* ===== 电机 ID ===== */
#define MOT_A    0U
#define MOT_B    1U

/* ===== 运动模式 ===== */
typedef enum {
    STOP     = 0,
    FORWARD  = 1,    /* 双轮速度模式（支持前进/后退）+ IMU 航向保持 */
    TURN     = 2,    /* IMU 转向 + 平均位置保持 */
    POSITION = 3,    /* 定位到两轮平均目标位置 + 航向保持 */
} MotionMode_t;

/* ===== 运行状态（外部可读） ===== */
typedef struct {
    int32_t pos_a;          /* 电机 A 编码器位置 */
    int32_t pos_b;          /* 电机 B 编码器位置 */
    int16_t speed_a;        /* 电机 A 速度 (脉冲/10ms) */
    int16_t speed_b;        /* 电机 B 速度 */
    int16_t target_speed_a; /* 速度环目标值 */
    int16_t target_speed_b;
    int16_t ccr_a;          /* 电机 A 有符号 CCR 输出，范围 -3599~3599 */
    int16_t ccr_b;          /* 电机 B 有符号 CCR 输出，范围 -3599~3599 */
    int32_t tar_pos_a;      /* 电机 A 目标位置 */
    int32_t tar_pos_b;      /* 电机 B 目标位置 */
    MotionMode_t mode;      /* 当前运动模式 */
    float    yaw;           /* IMU 航向角 (°) [-180,180] */
    float    wz;            /* IMU 角速度 (°/s) */
    float    tar_yaw;       /* 目标航向角 (°) */
    float    tar_wz;        /* 目标角速度 (°/s) */
} Ctrl_Status_t;

extern volatile Ctrl_Status_t g_ctrl;

void Ctrl_Init(void);
void Ctrl_Update10ms(void);  /* 编码器+IMU→WZ率环→速度环→电机 */
void Ctrl_Update50ms(void);  /* 位置环→速度目标, 角度环→WZ目标 */

void Ctrl_SetMode(MotionMode_t mode);
void Ctrl_SetTargetSpeed(int16_t speed_a, int16_t speed_b);
void Ctrl_SetTargetPos(int32_t pos_a, int32_t pos_b);
void Ctrl_AddTargetPos(int32_t delta);
void Ctrl_SetTargetYaw(float yaw);        /* 设置绝对目标角度 + 进入 TURN，需要有效 Yaw */
void Ctrl_AddTargetYaw(float delta);      /* 相对目标增量 + 进入 TURN，需要有效 Yaw */
void Ctrl_CalibrateYaw(void);             /* 锁定当前航向为目标，不改 IMU 零点 */
void Ctrl_ZeroPos(void);

#endif /* CTRL_H */
