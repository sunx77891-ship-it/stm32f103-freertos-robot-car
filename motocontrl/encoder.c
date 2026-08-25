/*
 * encoder.c - 编码器测速模块实现
 *
 * TIM2/TIM3 编码器模式 TI12:
 *   在 TI1 和 TI2 的上升沿和下降沿都计数，实现四倍频。
 *   计数器方向反映电机转向 (递增=正转, 递减=反转)。
 *
 * 编码器配置 (来自 CubeMX):
 *   TIM2: PA0=TIM2_CH1, PA1=TIM2_CH2, IC1Filter=5, IC2Filter=5
 *   TIM3: PA6=TIM3_CH1, PA7=TIM3_CH2, IC1Filter=5, IC2Filter=5
 *   均为 TI12 模式, Period=65535
 */

#include "encoder.h"
#include "tim.h"       /* htim2, htim3 */

/**************************************************************************
函数名：Encoder_Init
函数功能：启动 TIM2 和 TIM3 编码器模式计数
入口参数：无
返回值：无
说明：必须在 MX_TIM2_Init / MX_TIM3_Init 之后调用
      启动后编码器计数器从 0 开始累加
**************************************************************************/
void Encoder_Init(void)
{
    /* 编码器 A: TIM2 */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    /* 编码器 B: TIM3 */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

/**************************************************************************
函数名：Encoder_GetCount
函数功能：读取单个编码器计数增量并自动清零计数器
入口参数：enc - 编码器编号 (ENCODER_A 或 ENCODER_B)
返回值：自上次读取以来的脉冲增量 (有符号)
**************************************************************************/
int16_t Encoder_GetCount(EncoderId_t enc)
{
    int16_t count;

    if (enc == ENCODER_A)
    {
        count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        __HAL_TIM_SET_COUNTER(&htim2, 0);
    }
    else
    {
        count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
    }

    return count;
}

/**************************************************************************
函数名：Encoder_GetBoth
函数功能：同时读取两个编码器计数增量并清零计数器
入口参数：count_a - 保存 ENCODER_A 计数增量的指针
          count_b - 保存 ENCODER_B 计数增量的指针
返回值：无
**************************************************************************/
void Encoder_GetBoth(int16_t *count_a, int16_t *count_b)
{
    if (count_a != 0)
    {
        *count_a = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        __HAL_TIM_SET_COUNTER(&htim2, 0);
    }

    if (count_b != 0)
    {
        *count_b = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
    }
}

/**************************************************************************
函数名：Encoder_CountToRPM
函数功能：将编码器脉冲增量换算为 RPM
入口参数：count - Encoder_GetCount/GetBoth 得到的脉冲数
          ppr - 编码器每转总脉冲数 (线数 x 4)
          sample_ms - 采样周期 (ms)
返回值：对应的 RPM 值
说明：RPM = count * 60000 / (ppr * sample_ms)
      计算结果可能为负数，表示反转
**************************************************************************/
float Encoder_CountToRPM(int16_t count, uint16_t ppr, uint16_t sample_ms)
{
    float rpm;

    if (ppr == 0U || sample_ms == 0U)
        return 0.0f;

    rpm = (float)count * 60000.0f / ((float)ppr * (float)sample_ms);

    return rpm;
}
