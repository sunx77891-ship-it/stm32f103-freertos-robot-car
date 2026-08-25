#include "Ctrl.h"
#include "MotorDrive.h"
#include "Gyro.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdlib.h>

/* ===== 运行状态 ===== */
volatile Ctrl_Status_t g_ctrl = {0};

/* ===== 增量式 PI 结构（仅运行时状态，参数用宏） ===== */
typedef struct {
    float   prev_err;     /* 上一周期误差 */
    int16_t out;          /* 累计输出 (=P+I 总和) */
    int16_t out_max;      /* 输出限幅，兼积分抗饱和 */
} IncPI_t;

/* ===== 速度位置式 PI 运行状态 ===== */
typedef struct {
    float   integral;     /* 积分输出，单位 CCR */
    int16_t out;          /* 上一次实际 CCR 输出 */
} SpeedPI_t;

/* ==========================================================================
   速度环（内环 10ms）— 速度前馈 + 位置式 PI + 条件积分抗饱和
   ========================================================================== */
#define SPD_KP            5.0f
#define SPD_KI            8.0f
#define SPD_KFF_A        60.0f   /* 电机 A：每 1 脉冲/10ms 对应的前馈 CCR */
#define SPD_KFF_B        60.0f   /* 电机 B：每 1 脉冲/10ms 对应的前馈 CCR */
#define SPD_TS            0.01f
#define SPD_INT_LIM    1000.0f
#define SPD_OUT_MAX     3599
#define SPD_RAMP_STEP      1     /* FORWARD 模式每 10ms 最大目标变化量 */

static SpeedPI_t spd_pi_a;
static SpeedPI_t spd_pi_b;

/* ==========================================================================
   总线位置环 PID（外环 50ms）— 只管走多远，左右同速，与航向完全解耦
   ==========================================================================
   输入：总位移 = (pos_a + pos_b) / 2，目标 = g_tar_pos_a
   输出：g_tar_spd_a = g_tar_spd_b = 共同前向速度
   */
#define POS_KP        8.0f
#define POS_KI        0.2f
#define POS_KD        0.5f
#define POS_INT_LIM   30.0f
#define POS_SPD_MAX   90

static int32_t prev_pos_err   = 0;
static float   pos_int        = 0.0f;

/* ==========================================================================
   航向角环 PD（外环 50ms）— yaw_err → target_wz（目标角速度）
   物理含义：角度差(°) → 期望转向角速度(°/s)
   ==========================================================================
   YAW_KP：角度误差→目标角速度的放大倍数（P 项）。
       当前 KP=5，90°误差会输出 450°/s，随后被限制为 200°/s
   YAW_KI：角度误差积分，消除静态偏航。
       默认 0，确认有稳态误差后从 0.2 起加，每加 0.1 试一次
       过大易与 WZ_KI 双积分串联致超调
   YAW_KD：角速度阻尼系数（D 项实现为 -YAW_KD * g_ctrl.wz，非误差差分）。
       IMU 已对 wz 内部低通滤波，比 (yaw_err - prev_yaw_err)/dt 抗噪
       实测 yaw 增量与 WZ 同号，保留原补偿幅值 0.2 并使用正阻尼
   YAW_OUT_MAX：目标角速度上限（°/s）。
       200°/s ≈ 0.45s 转完 90°，留余量给内环线性区
   ========================================================================== */
#define YAW_KP        5.00f      /* MATLAB 整定 */
#define YAW_KI        0.00f      /* 级联已含积分 */
#define YAW_KD        0.2f       /* 正阻尼系数，实测 yaw 与 WZ 正方向一致 */
#define YAW_INT_LIM   100.0f
#define YAW_OUT_MAX   200.0f

static float yaw_int     = 0.0f;

/* ==========================================================================
   角速度环 PI（内环 10ms，增量式）— wz_err → diff_spd（速度差速，脉冲/10ms）
   物理含义：目标与实际角速度之差(°/s) → 两轮速度差(脉冲/10ms)
   ==========================================================================
   WZ_KP：角速度误差→差速的放大倍数。
       例：目标 180°/s - 实际 0°/s = 180°误差 × 2 = 360 → 限到 100
       越大则同样误差下差速越大，转向越猛
       过大：电机瞬间拉满，转向生硬 → 可降到 5
   WZ_KI：角速度积分，保证实际角速度追得上目标。
       当前 19 × 180°误差 × 0.01s = 34.2/cycle，由输出限幅抗饱和
   WZ_OUT_MAX：TURN 模式最大差速上限(脉冲/10ms)。
       FORWARD 模式会进一步限制为基础速度以内且不超过 10，防止单轮反转。
   ========================================================================== */
#define WZ_KP         2.00f      /* MATLAB 整定 */
#define WZ_KI         19.00f     /* MATLAB 整定 */
#define WZ_OUT_MAX    100.0f
#define FORWARD_DIFF_MAX 10

static IncPI_t wz_pi;      /* WZ 角速度增量式 PI */

/* ==========================================================================
   航向校准
   ========================================================================== */
static uint8_t yaw_calibrated = 0U;   /* 首次读到有效 IMU 数据后设为 1 */
static uint8_t yaw_valid      = 0U;   /* 已接收过有效 Yaw 数据 */
static float   g_target_wz  = 0.0f;

/* ==========================================================================
   目标值缓存
   ========================================================================== */
static MotionMode_t g_target_mode = STOP;
static int16_t  g_tar_spd_a = 0;
static int16_t  g_tar_spd_b = 0;
static int16_t  g_ramp_spd_a = 0;
static int16_t  g_ramp_spd_b = 0;
static int32_t  g_tar_pos_a = 0;
static int32_t  g_tar_pos_b = 0;
static float    g_tar_yaw   = 0.0f;   /* 目标航向角（IMU 原始坐标） */
static int32_t  g_key_target = 0;
static int32_t  pos_raw_a  = 0;
static int32_t  pos_raw_b  = 0;
static uint16_t enc_last_a = 0U;
static uint16_t enc_last_b = 0U;

/**************************************************************************
函数名：Ctrl_RampSpeed
函数功能：按固定步长将当前速度目标平滑逼近命令目标
入口参数：current - 当前斜坡目标，单位脉冲/10ms
          target - 最终命令目标，单位脉冲/10ms
返回值：本周期更新后的斜坡目标
**************************************************************************/
static int16_t Ctrl_RampSpeed(int16_t current, int16_t target)
{
    if (current < target)
    {
        current += SPD_RAMP_STEP;
        if (current > target) current = target;
    }
    else if (current > target)
    {
        current -= SPD_RAMP_STEP;
        if (current < target) current = target;
    }

    return current;
}

/**************************************************************************
函数名：Ctrl_SpeedPIUpdate
函数功能：运行单轮速度前馈和位置式 PI，并执行条件积分抗饱和
入口参数：state - 单轮速度 PI 运行状态
          target - 目标速度，单位脉冲/10ms
          measured - 实际速度，单位脉冲/10ms
          kff - 速度前馈系数，单位 CCR/(脉冲/10ms)
返回值：有符号电机 CCR，范围 -3599~3599
**************************************************************************/
static int16_t Ctrl_SpeedPIUpdate(SpeedPI_t *state, int16_t target, int16_t measured, float kff)
{
    float error;
    float integral_next;
    float output_raw;
    float output_min;
    float output_max;

    if (target == 0)
    {
        state->integral = 0.0f;
        state->out = 0;
        return 0;
    }

    error = (float)(target - measured);
    integral_next = state->integral + SPD_KI * error * SPD_TS;
    if (integral_next >  SPD_INT_LIM) integral_next =  SPD_INT_LIM;
    if (integral_next < -SPD_INT_LIM) integral_next = -SPD_INT_LIM;

    output_min = (target > 0) ? 0.0f : -(float)SPD_OUT_MAX;
    output_max = (target < 0) ? 0.0f :  (float)SPD_OUT_MAX;
    output_raw = kff * (float)target + SPD_KP * error + integral_next;

    /* 输出饱和且误差仍把输出推向饱和区时，冻结积分 */
    if ((output_raw > output_max && error > 0.0f) ||
        (output_raw < output_min && error < 0.0f))
    {
        output_raw = kff * (float)target + SPD_KP * error + state->integral;
    }
    else
    {
        state->integral = integral_next;
    }

    if (output_raw > output_max) output_raw = output_max;
    if (output_raw < output_min) output_raw = output_min;
    state->out = (int16_t)output_raw;
    return state->out;
}

/**************************************************************************
函数名：Ctrl_Init
函数功能：初始化控制状态，启动编码器和 PWM，并立即进入刹车状态
入口参数：无
返回值：无
**************************************************************************/
void Ctrl_Init(void)
{
    memset((void *)&g_ctrl, 0, sizeof(g_ctrl));
    /* 速度位置式 PI 与角速度增量式 PI 初始化 */
    spd_pi_a = (SpeedPI_t){ 0.0f, 0 };
    spd_pi_b = (SpeedPI_t){ 0.0f, 0 };
    wz_pi    = (IncPI_t){ 0.0f, 0, (int16_t)WZ_OUT_MAX };
    pos_raw_a = 0;   pos_raw_b = 0;
    pos_int  = 0.0f;
    yaw_int  = 0.0f;
    prev_pos_err = 0;
    g_target_mode = STOP;  g_key_target = 0;
    g_target_wz = 0.0f;  g_tar_yaw = 0.0f;
    g_ramp_spd_a = 0;  g_ramp_spd_b = 0;
    yaw_calibrated = 0U;  yaw_valid = 0U;
    enc_last_a = 0U;  enc_last_b = 0U;

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    MotorDrive_SetCcr(0);
}

/**************************************************************************
函数名：Ctrl_Update10ms
函数功能：每 10ms 读取编码器和 IMU，运行角速度环、轮速环并更新电机 PWM
入口参数：无
返回值：无
**************************************************************************/
void Ctrl_Update10ms(void)
{
    uint16_t enc_now_a, enc_now_b;
    int16_t enc_a, enc_b;
    int16_t base_a, base_b;
    int16_t tar_a, tar_b;
    int16_t out_a, out_b;

    /* ---- 读编码器 ---- */
    enc_now_a = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    enc_now_b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    enc_a = (int16_t)(enc_now_a - enc_last_a);
    enc_b = -(int16_t)(enc_now_b - enc_last_b);
    enc_last_a = enc_now_a;
    enc_last_b = enc_now_b;

    pos_raw_a += enc_a;  pos_raw_b += enc_b;
    g_ctrl.speed_a = enc_a;  g_ctrl.speed_b = enc_b;
    g_ctrl.pos_a = pos_raw_a;  g_ctrl.pos_b = pos_raw_b;

    /* ---- 读取 IMU 数据 ---- */
    {
        Gyro_Data_t gyro;
        if (Gyro_Read(&gyro, 0))
        {
            if (gyro.fresh & 0x01U)
                g_ctrl.wz = gyro.wz;

            if (gyro.fresh & 0x02U)
            {
                g_ctrl.yaw = gyro.yaw;
                yaw_valid = 1U;

                /* 首个有效 Yaw 帧锁定当前航向，0° 也是有效值 */
                if (!yaw_calibrated)
                {
                    g_tar_yaw = g_ctrl.yaw;
                    g_ctrl.tar_yaw = g_tar_yaw;
                    yaw_calibrated = 1U;
                }
            }
        }
    }

    /* ---- WZ 角速度增量式 PI（内环） ---- */
    {
        float wz_err = g_target_wz - g_ctrl.wz;

        /* 增量公式：out += Kp*(err-prev) + Ki*err*Ts，限幅即抗饱和 */
        float wz_delta = WZ_KP * (wz_err - wz_pi.prev_err) + WZ_KI * wz_err * SPD_TS;
        wz_pi.prev_err = wz_err;

        int32_t wz_tmp = (int32_t)((float)wz_pi.out + wz_delta);
        if (wz_tmp >  wz_pi.out_max) wz_tmp =  wz_pi.out_max;
        if (wz_tmp < -wz_pi.out_max) wz_tmp = -wz_pi.out_max;
        wz_pi.out = (int16_t)wz_tmp;

        int16_t diff_spd = wz_pi.out;

        if (g_target_mode == FORWARD)
        {
            int16_t base_abs;
            int16_t diff_lim;

            g_ramp_spd_a = Ctrl_RampSpeed(g_ramp_spd_a, g_tar_spd_a);
            g_ramp_spd_b = Ctrl_RampSpeed(g_ramp_spd_b, g_tar_spd_b);
            base_a = g_ramp_spd_a;
            base_b = g_ramp_spd_b;

            /* 直行差速不得让任一车轮目标越过 0，更不能反向 */
            base_abs = (base_a >= 0) ? base_a : (int16_t)-base_a;
            if (((base_b >= 0) ? base_b : (int16_t)-base_b) < base_abs)
                base_abs = (base_b >= 0) ? base_b : (int16_t)-base_b;
            diff_lim = (base_abs < FORWARD_DIFF_MAX) ? base_abs : FORWARD_DIFF_MAX;
            if (diff_spd >  diff_lim) diff_spd =  diff_lim;
            if (diff_spd < -diff_lim) diff_spd = -diff_lim;
        }
        else
        {
            base_a = g_tar_spd_a;
            base_b = g_tar_spd_b;
            g_ramp_spd_a = base_a;
            g_ramp_spd_b = base_b;
        }

        tar_a = base_a + (int16_t)diff_spd;
        tar_b = base_b - (int16_t)diff_spd;
    }

    g_ctrl.target_speed_a = tar_a;
    g_ctrl.target_speed_b = tar_b;
    g_ctrl.tar_wz = g_target_wz;
    g_ctrl.tar_yaw = g_tar_yaw;

    /* ---- 输出到电机 ---- */
    if (g_target_mode == STOP)
    {
        /* 控制器全清零，下一次启动从目标斜坡起点开始 */
        spd_pi_a = (SpeedPI_t){ 0.0f, 0 };
        spd_pi_b = (SpeedPI_t){ 0.0f, 0 };
        wz_pi    = (IncPI_t){ 0.0f,     0, (int16_t)WZ_OUT_MAX };
        g_ramp_spd_a = 0;
        g_ramp_spd_b = 0;
        g_ctrl.ccr_a = 0;
        g_ctrl.ccr_b = 0;
        MotorDrive_SetCcr(0);
    }
    else
    {
        out_a = Ctrl_SpeedPIUpdate(&spd_pi_a, tar_a, enc_a, SPD_KFF_A);
        out_b = Ctrl_SpeedPIUpdate(&spd_pi_b, tar_b, enc_b, SPD_KFF_B);

        g_ctrl.ccr_a = out_a;
        g_ctrl.ccr_b = out_b;
        MotorDrive_SetMotorA(out_a);
        MotorDrive_SetMotorB(out_b);
    }

}

/**************************************************************************
函数名：Ctrl_Update50ms
函数功能：每 50ms 运行位置外环和航向外环，生成基础速度与目标角速度
入口参数：无
返回值：无
**************************************************************************/
void Ctrl_Update50ms(void)
{
    /* ======== 航向角环 PD（非 STOP 模式均保持航向，FORWARD 加死区防抖动） ======== */
    if (g_target_mode != STOP)
    {
        float yaw_err = g_tar_yaw - g_ctrl.yaw;
        /* 归一化 ±180（角度减法后必须处理） */
        if (yaw_err > 180.0f)  yaw_err -= 360.0f;
        if (yaw_err < -180.0f) yaw_err += 360.0f;

        /* FORWARD 模式直行：±3°以内彻底清除差速，避免 WZ 环残留振荡 */
        if (g_target_mode == FORWARD && yaw_err > -3.0f && yaw_err < 3.0f)
        {
            yaw_int = 0.0f;
            g_target_wz = 0.0f;
            wz_pi = (IncPI_t){ 0.0f, 0, (int16_t)WZ_OUT_MAX };
        }
        else
        {
            float wz_ref;

            yaw_int += yaw_err * YAW_KI * 0.05f;
            if (yaw_int >  YAW_INT_LIM) yaw_int =  YAW_INT_LIM;
            if (yaw_int < -YAW_INT_LIM) yaw_int = -YAW_INT_LIM;

            /* D 项用 IMU 角速度反馈做阻尼，避免误差差分放大 IMU 噪声 */
            wz_ref = YAW_KP * yaw_err + yaw_int - YAW_KD * g_ctrl.wz;
            if (wz_ref >  YAW_OUT_MAX) wz_ref =  YAW_OUT_MAX;
            if (wz_ref < -YAW_OUT_MAX) wz_ref = -YAW_OUT_MAX;

            g_target_wz = wz_ref;
        }
    }

    /* ======== 总线位置环 PID（POSITION 全功能，TURN 锁位置不漂移） ====== */
    if (g_target_mode == POSITION || g_target_mode == TURN)
    {
        int32_t total_pos = (g_ctrl.pos_a + g_ctrl.pos_b) / 2;
        int32_t target_pos = (g_tar_pos_a + g_tar_pos_b) / 2;
        int32_t err = target_pos - total_pos;

        /* TURN 模式位置死区：≤±15 脉冲不纠正 */
        if (g_target_mode == TURN && err >= -15 && err <= 15) {
            err = 0;
            pos_int = 0.0f;
        }

        float deriv = -0.5f * (float)(g_ctrl.speed_a + g_ctrl.speed_b);

        if (err >= -200 && err <= 200) pos_int += (float)err * 0.05f;
        else pos_int = 0.0f;
        if (pos_int >  POS_INT_LIM) pos_int =  POS_INT_LIM;
        if (pos_int < -POS_INT_LIM) pos_int = -POS_INT_LIM;

        float pe = POS_KP * (float)err + POS_KI * pos_int + POS_KD * deriv;

        /* TURN 锁位置限速 20 防止冲过头，POSITION 用全速 90 */
        float spd_lim = (g_target_mode == TURN) ? 20.0f : (float)POS_SPD_MAX;
        if (pe >  spd_lim) pe =  spd_lim;
        if (pe < -spd_lim) pe = -spd_lim;

        g_tar_spd_a = (int16_t)pe;
        g_tar_spd_b = (int16_t)pe;

        if (g_target_mode == POSITION)
        {
            int cross = (prev_pos_err > 0 && err <= 0) || (prev_pos_err < 0 && err >= 0);
            if (cross || abs(err) < 20)
                Ctrl_SetMode(STOP);
            else
                prev_pos_err = err;
        }
        else
        {
            prev_pos_err = err;
        }
    }
}

/* ===== 外部接口 ===== */

/**************************************************************************
函数名：Ctrl_SetModeUnlocked
函数功能：切换运动模式并清除上一模式的控制器历史，由已加锁的接口调用
入口参数：mode - 新的运动模式
返回值：无
**************************************************************************/
static void Ctrl_SetModeUnlocked(MotionMode_t mode)
{
    g_target_mode = mode;
    g_ctrl.mode = mode;

    spd_pi_a = (SpeedPI_t){ 0.0f, 0 };
    spd_pi_b = (SpeedPI_t){ 0.0f, 0 };
    wz_pi = (IncPI_t){ 0.0f, 0, (int16_t)WZ_OUT_MAX };
    pos_int = 0.0f;
    prev_pos_err = 0;
    yaw_int = 0.0f;
    g_target_wz = 0.0f;
    g_tar_spd_a = 0;
    g_tar_spd_b = 0;
    g_ramp_spd_a = 0;
    g_ramp_spd_b = 0;

    if (mode == STOP)
    {
        g_ctrl.ccr_a = 0;
        g_ctrl.ccr_b = 0;
        MotorDrive_SetCcr(0);
    }
}

/**************************************************************************
函数名：Ctrl_SetMode
函数功能：原子切换小车运动模式，并清除旧模式的控制器状态
入口参数：mode - STOP、FORWARD、TURN 或 POSITION
返回值：无
**************************************************************************/
void Ctrl_SetMode(MotionMode_t mode)
{
    taskENTER_CRITICAL();
    Ctrl_SetModeUnlocked(mode);
    taskEXIT_CRITICAL();
}

/**************************************************************************
函数名：Ctrl_SetTargetSpeed
函数功能：设置两轮基础目标速度，停止状态下同步进入直行模式
入口参数：speed_a - 电机 A 目标速度，单位为脉冲/10ms
          speed_b - 电机 B 目标速度，单位为脉冲/10ms
返回值：无
**************************************************************************/
void Ctrl_SetTargetSpeed(int16_t speed_a, int16_t speed_b)
{
    taskENTER_CRITICAL();
    if (g_target_mode == STOP)
        Ctrl_SetModeUnlocked(FORWARD);
    g_tar_spd_a = speed_a;
    g_tar_spd_b = speed_b;
    taskEXIT_CRITICAL();
}

/**************************************************************************
函数名：Ctrl_SetTargetPos
函数功能：设置两轮目标位置，位置环使用两者平均值
入口参数：pos_a - 电机 A 目标编码器脉冲
          pos_b - 电机 B 目标编码器脉冲
返回值：无
**************************************************************************/
void Ctrl_SetTargetPos(int32_t pos_a, int32_t pos_b)
{
    taskENTER_CRITICAL();
    g_tar_pos_a = pos_a;  g_tar_pos_b = pos_b;
    g_ctrl.tar_pos_a = pos_a;  g_ctrl.tar_pos_b = pos_b;
    taskEXIT_CRITICAL();
}

/**************************************************************************
函数名：Ctrl_AddTargetPos
函数功能：在累计位置目标上增加相同的两轮位移
入口参数：delta - 位置增量，单位为编码器脉冲
返回值：无
**************************************************************************/
void Ctrl_AddTargetPos(int32_t delta)
{
    taskENTER_CRITICAL();
    g_key_target += delta;
    g_tar_pos_a = g_key_target;
    g_tar_pos_b = g_key_target;
    g_ctrl.tar_pos_a = g_key_target;
    g_ctrl.tar_pos_b = g_key_target;
    taskEXIT_CRITICAL();
}

/**************************************************************************
函数名：Ctrl_SetTargetYaw
函数功能：设置绝对目标航向并进入转向，IMU 航向无效时保持停止
入口参数：yaw - 目标航向角，单位度
返回值：无
**************************************************************************/
void Ctrl_SetTargetYaw(float yaw)
{
    int32_t hold_pos;

    taskENTER_CRITICAL();
    if (!yaw_valid)
    {
        Ctrl_SetModeUnlocked(STOP);
        taskEXIT_CRITICAL();
        return;
    }

    hold_pos = (g_ctrl.pos_a + g_ctrl.pos_b) / 2;
    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;
    g_tar_yaw = yaw;
    g_tar_pos_a = hold_pos;
    g_tar_pos_b = hold_pos;
    g_ctrl.tar_pos_a = hold_pos;
    g_ctrl.tar_pos_b = hold_pos;
    Ctrl_SetModeUnlocked(TURN);
    taskEXIT_CRITICAL();
}

/**************************************************************************
函数名：Ctrl_AddTargetYaw
函数功能：叠加目标航向并进入转向，IMU 航向无效时保持停止
入口参数：delta - 航向增量，单位度
返回值：无
**************************************************************************/
void Ctrl_AddTargetYaw(float delta)
{
    int32_t hold_pos;

    taskENTER_CRITICAL();
    if (!yaw_valid)
    {
        Ctrl_SetModeUnlocked(STOP);
        taskEXIT_CRITICAL();
        return;
    }

    hold_pos = (g_ctrl.pos_a + g_ctrl.pos_b) / 2;
    g_tar_yaw += delta;
    while (g_tar_yaw > 180.0f) g_tar_yaw -= 360.0f;
    while (g_tar_yaw < -180.0f) g_tar_yaw += 360.0f;
    g_tar_pos_a = hold_pos;
    g_tar_pos_b = hold_pos;
    g_ctrl.tar_pos_a = hold_pos;
    g_ctrl.tar_pos_b = hold_pos;
    Ctrl_SetModeUnlocked(TURN);
    taskEXIT_CRITICAL();
}

/**************************************************************************
函数名：Ctrl_CalibrateYaw
函数功能：将当前有效航向锁定为目标，不修改 IMU 本身的零点
入口参数：无
返回值：无
**************************************************************************/
void Ctrl_CalibrateYaw(void)
{
    taskENTER_CRITICAL();
    g_tar_yaw = g_ctrl.yaw;
    g_target_wz = 0.0f;
    wz_pi    = (IncPI_t){ 0.0f,     0, (int16_t)WZ_OUT_MAX };
    yaw_int = 0.0f;
    g_ctrl.tar_yaw = g_tar_yaw;
    yaw_calibrated = yaw_valid;
    taskEXIT_CRITICAL();
}

/**************************************************************************
函数名：Ctrl_ZeroPos
函数功能：原子清零编码器硬件计数、软件位置和位置环历史
入口参数：无
返回值：无
**************************************************************************/
void Ctrl_ZeroPos(void)
{
    taskENTER_CRITICAL();
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    enc_last_a = 0U;  enc_last_b = 0U;
    pos_raw_a = 0;  pos_raw_b = 0;
    g_key_target = 0;
    g_tar_pos_a = 0;  g_tar_pos_b = 0;
    g_ctrl.pos_a = 0;  g_ctrl.pos_b = 0;
    g_ctrl.tar_pos_a = 0;  g_ctrl.tar_pos_b = 0;
    pos_int = 0.0f;
    prev_pos_err = 0;
    taskEXIT_CRITICAL();
}
