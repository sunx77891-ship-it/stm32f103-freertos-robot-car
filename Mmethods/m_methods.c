#include "m_methods.h"

#include "tim.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

/* 编码器累计计数值（供 OLED 监控任务读取） */
volatile int32_t g_encoder_accum_a = 0;
volatile int32_t g_encoder_accum_b = 0;

/* FreeRTOS 传入的实验参数在初始化时复制到驱动内部。 */
static M_Method_Config_t M_Config;

/* 10 阶移位寄存器，非零初值对应长度为 1023 的 M 序列。 */
static uint16_t M_Lfsr = 0x03FFU;

static uint16_t M_Current_Ccr;
static uint16_t M_Samples_On_Bit;
static uint32_t M_Sample_Index;
static uint8_t M_Init_Ok;

/* 双缓冲 — DMA 发送时不能覆写正在使用的缓冲区 */
static char M_Uart_Buf[2][60];
static uint8_t M_Uart_Buf_Idx = 0;

/**************************************************************************
函数名：M_Motor_Set_Ccr
函数功能：把同一个 CCR 同步送给两个电机
入口参数：ccr - 需要写入电机 A 和电机 B 的 CCR，范围 0~3599
返回值：无
说明：用户硬件接线:
          电机 A — PA10(IN1)=TIM1_CH3, PA11(IN2)=TIM1_CH4
          电机 B — PA8(IN1)=TIM1_CH1,  PA9(IN2)=TIM1_CH2
       每个电机的 IN1 接 PWM、IN2 接 0，两个电机都向前转
**************************************************************************/
static void M_Motor_Set_Ccr(uint16_t ccr)
{
    if (ccr == 0U)
    {
        /* 刹车：IN1=IN2=高 → A4950 主动制动 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 3599);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 3599);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 3599);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 3599);
    }
    else
    {
        /* 正转：IN1=PWM, IN2=0 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);   /* PA8 = B_IN1 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr);      /* PA9 = B_IN2 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr);   /* PA10 = A_IN1 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);      /* PA11 = A_IN2 */
    }
}

/**************************************************************************
函数名：M_Sequence_Next_Ccr
函数功能：产生下一位 M 序列，并把 0/1 映射为低档或高档 CCR
入口参数：无
返回值：下一位 M 序列对应的 CCR
**************************************************************************/
static uint16_t M_Sequence_Next_Ccr(void)
{
    uint8_t m_bit;
    uint8_t feedback;

    m_bit = (uint8_t)(M_Lfsr & 0x01U);
    feedback = (uint8_t)(((M_Lfsr >> 0U) ^ (M_Lfsr >> 3U)) & 0x01U);
    M_Lfsr = (uint16_t)((M_Lfsr >> 1U) | ((uint16_t)feedback << 9U));

    if (m_bit == 0U)
        return M_Config.ccr_low;
    else
        return M_Config.ccr_high;
}

/**************************************************************************
函数名：M_Encoder_Read
函数功能：读取两个编码器在一个采样周期内的计数增量，并清零计数器
入口参数：speed_a_count - 保存 TIM2 编码器计数增量的指针
          speed_b_count - 保存 TIM3 编码器计数增量的指针
返回值：无
说明：两个输出的单位都是“计数/采样周期”
**************************************************************************/
static void M_Encoder_Read(int16_t *speed_a_count, int16_t *speed_b_count)
{
    *speed_a_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    g_encoder_accum_a += *speed_a_count;

    *speed_b_count = -(int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    g_encoder_accum_b += *speed_b_count;
}

/**************************************************************************
函数名：M_Uart_Send_Row
函数功能：通过串口 1 发送一行 M 序列实验 CSV 数据
入口参数：sample - 当前采样序号
          ccr - 当前采样周期对应的 TIM1 CCR
          speed_a_count - 电机 A 的编码器计数增量
          speed_b_count - 电机 B 的编码器计数增量
返回值：1 - 串口发送成功
        0 - 字符串生成失败或串口发送失败
**************************************************************************/
static uint8_t M_Uart_Send_Row(uint32_t sample,
                               uint16_t ccr,
                               int16_t speed_a_count,
                               int16_t speed_b_count)
{
    int length;
    uint32_t time_ms;

    time_ms = (sample + 1U) * M_Config.sample_time_ms;

    /* 切换缓冲：DMA 还在读上一块，用另一块 */
    M_Uart_Buf_Idx ^= 1U;
    length = sprintf(M_Uart_Buf[M_Uart_Buf_Idx],
                     "%lu,%lu,%u,%d,%d\r\n",
                     (unsigned long)sample,
                     (unsigned long)time_ms,
                     (unsigned int)ccr,
                     (int)speed_a_count,
                     (int)speed_b_count);

    if (length <= 0)
        return 0U;

    /* DMA 发送，CPU 不阻塞；若上次 DMA 还没完则丢弃本次（不该发生） */
    if (HAL_UART_Transmit_DMA(&huart1,
                              (uint8_t *)M_Uart_Buf[M_Uart_Buf_Idx],
                              (uint16_t)length) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

/**************************************************************************
函数名：M_Method_Init
函数功能：保存实验参数，启动 TIM1、TIM2、TIM3，并发送 CSV 表头
入口参数：config - M 序列实验参数结构体指针
返回值：1 - 初始化成功
        0 - 参数错误或外设启动失败
**************************************************************************/
uint8_t M_Method_Init(const M_Method_Config_t *config)
{
    static const char csv_title[] =
        "sample,time_ms,ccr,speed_a_count,speed_b_count\r\n";

    if (config == 0 ||
        config->ccr_low > M_METHODS_TIM1_ARR ||
        config->ccr_high > M_METHODS_TIM1_ARR ||
        config->sample_time_ms == 0U ||
        config->samples_per_bit == 0U)
    {
        return 0U;
    }

    M_Config = *config;
    M_Lfsr = 0x03FFU;
    M_Samples_On_Bit = 0U;
    M_Sample_Index = 0U;
    M_Init_Ok = 0U;

    M_Motor_Set_Ccr(0U);

    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
        return 0U;
    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
        return 0U;

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
        return 0U;
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
        return 0U;
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
        return 0U;
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
        return 0U;

    /* 发送 CSV 表头（DMA，不阻塞；失败不影响实验） */
    HAL_UART_Transmit_DMA(&huart1,
                          (uint8_t *)csv_title,
                          (uint16_t)strlen(csv_title));

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    M_Current_Ccr = M_Sequence_Next_Ccr();
    M_Motor_Set_Ccr(M_Current_Ccr);

    M_Init_Ok = 1U;
    return 1U;
}

/**************************************************************************
函数名：M_Method_Update
函数功能：完成一次编码器采样、CSV 发送和 M 序列 CCR 更新
入口参数：无
返回值：1 - 本次采样和发送成功
        0 - 驱动未初始化或串口发送失败
**************************************************************************/
uint8_t M_Method_Update(void)
{
    int16_t speed_a_count;
    int16_t speed_b_count;

    if (M_Init_Ok == 0U)
        return 0U;

    M_Encoder_Read(&speed_a_count, &speed_b_count);

    /* 尝试发送 CSV 数据行，发送失败不影响电机实验 */
    M_Uart_Send_Row(M_Sample_Index,
                    M_Current_Ccr,
                    speed_a_count,
                    speed_b_count);

    M_Sample_Index++;
    M_Samples_On_Bit++;

    if (M_Samples_On_Bit >= M_Config.samples_per_bit)
    {
        M_Samples_On_Bit = 0U;
        M_Current_Ccr = M_Sequence_Next_Ccr();
        M_Motor_Set_Ccr(M_Current_Ccr);
    }

    return 1U;
}

/**************************************************************************
函数名：M_Method_Stop
函数功能：停止 M 序列实验，并把 TIM1 四个通道的 CCR 清零
入口参数：无
返回值：无
**************************************************************************/
void M_Method_Stop(void)
{
    M_Init_Ok = 0U;

    /* 等最后一行 DMA 发完后再清零电机（约 5ms） */
    while (huart1.hdmatx->State == HAL_DMA_STATE_BUSY)
    {
        /* 等待 DMA 传输完成 */
    }

    M_Current_Ccr = 0U;
    M_Motor_Set_Ccr(0U);
}
