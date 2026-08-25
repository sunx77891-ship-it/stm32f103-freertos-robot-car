#ifndef YAWI_D_H
#define YAWI_D_H

#include <stdint.h>

/* ===== TIM1 参数 ===== */
#define YAWI_D_TIM1_ARR   3599U

/* ===== M 序列参数 ===== */
#define YAWI_D_SEQ_LEN    1023U      /* 10 阶 M 序列长度（固定） */
#define YAWI_D_LFSR_INIT  0x03FFU    /* 移位寄存器初值（非零） */

/* ===== 实验参数（可调） ===== */
#define YAWI_D_DIFF_CCR   2800U      /* 差速 CCR 幅值（正负交替，建议 800~2800） */
#define YAWI_D_SAMPLE_MS  10U        /* 采样周期(ms)，和速度环周期一致 */
#define YAWI_D_SPB        100U        /* 每位 M 序列保持的采样次数 */
                                     /* 每次保持时间 = SAMPLE_MS × SPB = 500ms */

/* ===== 停止条件 ===== */
/* 采集到 TARGET_SAMPLES 个样本后自动停止（SPB×1023=51150 约 8.5min） */
#define YAWI_D_TARGET_SAMPLES  10230U  /* 目标采样数，到数即停 */

/* ===== M 序列实验参数字典（给 YawID_Init 传入） ===== */
typedef struct {
    uint16_t diff_low;          /* 差速低电平 CCR（当前未使用，保留） */
    uint16_t diff_high;         /* 差速高电平 CCR = 幅值 */
    uint16_t sample_time_ms;    /* 采样周期 (ms) */
    uint16_t samples_per_bit;   /* 每位保持的采样次数 */
} YawID_Config_t;

extern volatile uint8_t g_yawid_running;

uint8_t YawID_Init(const YawID_Config_t *config);
uint8_t YawID_Update(void);
void YawID_Stop(void);

#endif /* YAWI_D_H */
