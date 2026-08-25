#include "YawID.h"
#include "tim.h"
#include "usart.h"
#include "Gyro.h"
#include "Periph.h"
#include <stdio.h>
#include <string.h>

volatile uint8_t g_yawid_running = 0U;

static YawID_Config_t Y_Config;
static uint16_t Y_Lfsr = YAWI_D_LFSR_INIT;
static int16_t Y_Curr_Ccr;
static uint16_t Y_Samples_On_Bit;
static uint32_t Y_Sample_Index;
static uint32_t Y_Total_Samples;
static uint8_t  Y_Init_Ok;

static char Y_Uart_Buf[2][64];
static uint8_t Y_Uart_Buf_Idx;

/**************************************************************************
设置两轮差速（A+ccr 正转，B-ccr 反转 = 差速旋转）
  diff > 0 → MotorA 正转, MotorB 反转 → 顺时针旋转
  diff = 0 → 刹车
  diff < 0 → MotorA 反转, MotorB 正转 → 逆时针旋转
**************************************************************************/
static void YawID_SetDiff(int16_t diff_ccr)
{
    if (diff_ccr == 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 3599);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 3599);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 3599);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 3599);
    }
    else if (diff_ccr > 0)
    {
        /* A 正转: CH3=PWM, CH4=0  |  B 反转: CH1=PWM, CH2=0 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)diff_ccr);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t)diff_ccr);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
    else
    {
        /* A 反转: CH3=0, CH4=PWM  |  B 正转: CH1=0, CH2=PWM */
        uint16_t pwm = (uint16_t)(-diff_ccr);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
    }
}

static int16_t YawID_Next_Ccr(void)
{
    uint8_t m_bit = (uint8_t)(Y_Lfsr & 0x01U);
    uint8_t feedback = (uint8_t)(((Y_Lfsr >> 0U) ^ (Y_Lfsr >> 3U)) & 0x01U);
    Y_Lfsr = (uint16_t)((Y_Lfsr >> 1U) | ((uint16_t)feedback << 9U));

    /* 单极性：0→刹车  1→差速旋转 */
    return (m_bit == 0U) ? 0 : (int16_t)Y_Config.diff_high;
}

static void YawID_ReadIMU(float *yaw, float *wz)
{
    Gyro_Data_t gyro;
    if (Gyro_Read(&gyro, 0))
    {
        *yaw = gyro.yaw;
        *wz  = gyro.wz;
    }
}

static uint8_t YawID_Send_Row(uint32_t sample, int16_t ccr,
                               float yaw, float wz)
{
    int len;
    uint32_t time_ms = (sample + 1U) * Y_Config.sample_time_ms;

    Y_Uart_Buf_Idx ^= 1U;
    len = sprintf(Y_Uart_Buf[Y_Uart_Buf_Idx],
                  "%lu,%lu,%d,%d,%d\r\n",
                  (unsigned long)sample,
                  (unsigned long)time_ms,
                  (int)ccr,
                  (int)(yaw ),
                  (int)(wz ));

    if (len <= 0) return 0U;

    if (HAL_UART_Transmit_DMA(&huart1,
                              (uint8_t *)Y_Uart_Buf[Y_Uart_Buf_Idx],
                              (uint16_t)len) != HAL_OK)
        return 0U;

    return 1U;
}

uint8_t YawID_Init(const YawID_Config_t *config)
{
    static const char csv_title[] =
        "sample,time_ms,diff_ccr,yaw_x10,wz_x10\r\n";

    if (config == NULL ||
        config->diff_low  > YAWI_D_TIM1_ARR ||
        config->diff_high > YAWI_D_TIM1_ARR ||
        config->sample_time_ms  == 0U ||
        config->samples_per_bit == 0U)
        return 0U;

    Y_Config = *config;
    Y_Lfsr = YAWI_D_LFSR_INIT;
    Y_Samples_On_Bit = 0U;
    Y_Sample_Index = 0U;
    Y_Total_Samples = YAWI_D_TARGET_SAMPLES;
    Y_Init_Ok = 0U;

    /* 刹车（外设已在 Ctrl_Init 中启动，不再重复启） */
    YawID_SetDiff(0);

    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)csv_title, (uint16_t)strlen(csv_title));

    Y_Curr_Ccr = YawID_Next_Ccr();
    YawID_SetDiff(Y_Curr_Ccr);

    Y_Init_Ok = 1U;
    g_yawid_running = 1U;
    return 1U;
}

uint8_t YawID_Update(void)
{
    float yaw = 0.0f, wz = 0.0f;

    if (Y_Init_Ok == 0U) return 0U;

    YawID_ReadIMU(&yaw, &wz);
    YawID_Send_Row(Y_Sample_Index, Y_Curr_Ccr, yaw, wz);

    Y_Sample_Index++;
    Y_Samples_On_Bit++;

    if (Y_Samples_On_Bit >= Y_Config.samples_per_bit)
    {
        Y_Samples_On_Bit = 0U;
        Y_Curr_Ccr = YawID_Next_Ccr();
        YawID_SetDiff((int16_t)Y_Curr_Ccr);
    }

    /* 全部采样完成 */
    if (Y_Sample_Index >= Y_Total_Samples)
    {
        YawID_Stop();
    }

    return 1U;
}

void YawID_Stop(void)
{
    Y_Init_Ok = 0U;
    g_yawid_running = 0U;

    while (huart1.hdmatx->State == HAL_DMA_STATE_BUSY) { }

    YawID_SetDiff(0);

    /* 蜂鸣器鸣叫 3 声 */
    Buzzer_Beep(150, 200, 3);
}
