/*
 * encoder.h - 编码器测速模块 (TIM2 / TIM3)
 *
 * 硬件:
 *   编码器A: TIM2_CH1(PA0), TIM2_CH2(PA1)   -> TI12 四倍频
 *   编码器B: TIM3_CH1(PA6), TIM3_CH2(PA7)   -> TI12 四倍频
 *
 * 使用步骤:
 *   1. main 中已 MX_TIM2_Init / MX_TIM3_Init 初始化定时器
 *   2. 调用 Encoder_Init() 启动编码器
 *   3. 每个采样周期调用 Encoder_GetBoth() 读取脉冲增量
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

/* 编码器编号 */
typedef enum {
    ENCODER_A = 0,   /* TIM2 */
    ENCODER_B = 1    /* TIM3 */
} EncoderId_t;

/**************************************************************************
函数名：Encoder_Init
函数功能：启动 TIM2 和 TIM3 编码器模式计数
入口参数：无
返回值：无
说明：必须先调用 MX_TIM2_Init / MX_TIM3_Init 初始化定时器
      启动后编码器计数器从 0 开始累加
**************************************************************************/
void Encoder_Init(void);

/**************************************************************************
函数名：Encoder_GetCount
函数功能：读取单个编码器计数增量并自动清零计数器
入口参数：enc - 编码器编号 (ENCODER_A 或 ENCODER_B)
返回值：自上次读取以来的脉冲增量 (有符号)
说明：返回值单位是"计数/采样周期"，不是 RPM
      TIM2/TIM3 为 TI12 四倍频模式
**************************************************************************/
int16_t Encoder_GetCount(EncoderId_t enc);

/**************************************************************************
函数名：Encoder_GetBoth
函数功能：同时读取两个编码器计数增量并清零计数器
入口参数：count_a - 保存 ENCODER_A 计数增量的指针
          count_b - 保存 ENCODER_B 计数增量的指针
返回值：无
说明：两个编码器先后读取，间隔极短，适合在一个采样点同时采集
**************************************************************************/
void Encoder_GetBoth(int16_t *count_a, int16_t *count_b);

/**************************************************************************
函数名：Encoder_CountToRPM
函数功能：将编码器脉冲增量换算为 RPM
入口参数：count - Encoder_GetCount 或 Encoder_GetBoth 得到的脉冲数
          ppr - 编码器每转总脉冲数 (线数 x 4，四倍频后)
          sample_ms - 采样周期，单位 ms
返回值：对应的 RPM 值 (浮点数)
说明：RPM = count * 60000 / (ppr * sample_ms)
      例: 编码器 13 线, TI12 四倍频后 ppr = 52
          采样 10ms, count = 26, 则 RPM = 26*60000/(52*10) = 3000
**************************************************************************/
float Encoder_CountToRPM(int16_t count, uint16_t ppr, uint16_t sample_ms);

#endif /* __ENCODER_H */
