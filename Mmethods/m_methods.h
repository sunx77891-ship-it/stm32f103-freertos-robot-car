#ifndef M_METHODS_H
#define M_METHODS_H

#include <stdint.h>

/* TIM1 的 ARR 固定为 3599，M 序列只改变 CCR，不改变 PWM 频率。 */
#define M_METHODS_TIM1_ARR 3599U

/* 10 阶 M 序列共有 1023 位，运行完后会自动从头重复。 */
#define M_METHODS_SEQUENCE_LENGTH 1023U

/* M 序列实验参数，由 freertos.c 填写后传给驱动。 */
typedef struct
{
    uint16_t ccr_low;          /* M 序列低电平 CCR，范围 0~3599。 */
    uint16_t ccr_high;         /* M 序列高电平 CCR，范围 0~3599。 */
    uint16_t sample_time_ms;   /* 编码器采样周期，单位 ms。 */
    uint16_t samples_per_bit;  /* 每位 M 序列保持的采样次数。 */
} M_Method_Config_t;

/**************************************************************************
函数名：M_Method_Init
函数功能：保存实验参数，启动 TIM1、TIM2、TIM3，并发送 CSV 表头
入口参数：config - M 序列实验参数结构体指针
返回值：1 - 初始化成功
        0 - 参数错误或外设启动失败
**************************************************************************/
/* 编码器累计计数值（供 OLED 监控任务读取），每 10ms 累加一次，只增不减 */
extern volatile int32_t g_encoder_accum_a;
extern volatile int32_t g_encoder_accum_b;

uint8_t M_Method_Init(const M_Method_Config_t *config);

/**************************************************************************
函数名：M_Method_Update
函数功能：完成一次编码器采样、CSV 发送和 M 序列 CCR 更新
入口参数：无
返回值：1 - 本次采样和发送成功
        0 - 驱动未初始化或串口发送失败
**************************************************************************/
uint8_t M_Method_Update(void);

/**************************************************************************
函数名：M_Method_Stop
函数功能：停止 M 序列实验，并把 TIM1 四个通道的 CCR 清零
入口参数：无
返回值：无
**************************************************************************/
void M_Method_Stop(void);

#endif
