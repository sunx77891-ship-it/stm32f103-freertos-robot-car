#include "Key.h"
#include "gpio.h"
#include "Periph.h"

/* ===== 引脚映射表 ===== */
static const struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} Key_Map[KEY_NUM] = {
    {GPIOC, GPIO_PIN_13},   /* K1 */
    {GPIOC, GPIO_PIN_14},   /* K2 */
    {GPIOC, GPIO_PIN_0},    /* K3 */
    {GPIOC, GPIO_PIN_1},    /* K4 */
    {GPIOC, GPIO_PIN_2},    /* K5 */
    {GPIOC, GPIO_PIN_3},    /* K6 */
};

/* ===== 长按阈值 ===== */
/* Key_Process 每 10ms 调用一次，200 计数 ≈ 2 秒 */
#define LONG_PRESS_CNT  200U
#define ENABLED_KEY_NUM 4U     /* 启用 K1~K4，K5~K6 不扫描 */
#define KEY_FEEDBACK_MS 30U     /* 按键声光反馈持续时间，单位：ms */
#define KEY_FEEDBACK_LEDS (LED4 | LED5)

/* ===== 按键状态结构 ===== */
typedef struct {
    uint8_t  key_std;       /* 状态: 0=松开  1=确认按下  2=保持 */
    uint8_t  key_flag;      /* 当前电平 (0=按下  1=松开) */
    uint8_t  key_press_bit; /* 消抖确认按下事件 */
    uint8_t  key_bit;       /* 短按标志 */
    uint8_t  key_long_bit;  /* 长按标志 */
    uint16_t key_time;      /* 按下持续计数 (10ms 单位) */
} Key_State_t;

static Key_State_t key_states[KEY_NUM];

/**************************************************************************
函数名：Key_Init
函数功能：初始化所有按键状态
入口参数：无
返回值：无
**************************************************************************/
void Key_Init(void)
{
    uint8_t i;

    for (i = 0; i < KEY_NUM; i++)
    {
        key_states[i].key_std       = 0U;
        key_states[i].key_flag      = 0U;
        key_states[i].key_press_bit = 0U;
        key_states[i].key_bit       = 0U;
        key_states[i].key_long_bit  = 0U;
        key_states[i].key_time      = 0U;
    }
}

/**************************************************************************
函数名：Key_PlayFeedback
函数功能：确认按键按下时，让 LED4、LED5 同时点亮并与蜂鸣器同步保持 30ms
入口参数：无
返回值：无
**************************************************************************/
static void Key_PlayFeedback(void)
{
    LED_On(KEY_FEEDBACK_LEDS);
    Buzzer_Beep(KEY_FEEDBACK_MS, 0U, 1U);
    LED_Off(KEY_FEEDBACK_LEDS);
}

/**************************************************************************
函数名：Key_Process
函数功能：K1~K4 按键状态机（每 10ms 调用一次），K5~K6 保持禁用
          读取引脚电平 → 消抖确认并输出声光反馈 → 保持计时 → 松开判定短按/长按
入口参数：无
返回值：无
**************************************************************************/
void Key_Process(void)
{
    uint8_t i;

    for (i = 0; i < ENABLED_KEY_NUM; i++)
    {
        /* 读取当前电平：0=按下 (RESET)  1=松开 (SET) */
        key_states[i].key_flag = HAL_GPIO_ReadPin(Key_Map[i].port, Key_Map[i].pin);

        switch (key_states[i].key_std)
        {
        case 0:  /* 松开状态 */
            if (key_states[i].key_flag == 0U)
            {
                key_states[i].key_std  = 1U;  /* 检测到按下 → 进入确认 */
                key_states[i].key_time = 0U;  /* 重置计时 */
            }
            else
            {
                key_states[i].key_std = 0U;   /* 保持松开 */
            }
            break;

        case 1:  /* 确认按下（消抖） */
            if (key_states[i].key_flag == 0U)
            {
                key_states[i].key_std = 2U;   /* 确认持续按下 */
                key_states[i].key_press_bit = 1U;
                Key_PlayFeedback();           /* 按键确认 → LED4、LED5 与蜂鸣器同步反馈 */
            }
            else
            {
                key_states[i].key_std = 0U;   /* 抖动，回到松开 */
            }
            break;

        case 2:  /* 保持按下 */
            if (key_states[i].key_flag == 1U && key_states[i].key_time >= LONG_PRESS_CNT)
            {
                /* 松开且达到长按阈值 → 长按事件 */
                key_states[i].key_std      = 0U;
                key_states[i].key_long_bit = 1U;
            }
            else if (key_states[i].key_flag == 0U)
            {
                /* 持续按下 → 计时 */
                key_states[i].key_time++;
            }
            else if (key_states[i].key_flag == 1U && key_states[i].key_time < LONG_PRESS_CNT)
            {
                /* 松开且未达阈值 → 短按事件 */
                key_states[i].key_std = 0U;
                key_states[i].key_bit = 1U;
            }
            break;
        }
    }
}

/**************************************************************************
函数名：Key_GetPress
函数功能：读取并清除指定按键的消抖确认按下事件
入口参数：key - 按键 ID
返回值：1=已确认按下，0=无事件
**************************************************************************/
uint8_t Key_GetPress(Key_ID_t key)
{
    uint8_t ret;

    if (key >= KEY_NUM) return 0U;

    ret = key_states[key].key_press_bit;
    key_states[key].key_press_bit = 0U;
    return ret;
}

/**************************************************************************
函数名：Key_GetShortPress
函数功能：读取并清除短按标志
入口参数：key - 按键 ID
返回值  ：1=有短按事件  0=无
**************************************************************************/
uint8_t Key_GetShortPress(Key_ID_t key)
{
    uint8_t ret;

    if (key >= KEY_NUM) return 0U;

    ret = key_states[key].key_bit;
    key_states[key].key_bit = 0U;
    return ret;
}

/**************************************************************************
函数名：Key_GetLongPress
函数功能：读取并清除长按标志
入口参数：key - 按键 ID
返回值  ：1=有长按事件  0=无
**************************************************************************/
uint8_t Key_GetLongPress(Key_ID_t key)
{
    uint8_t ret;

    if (key >= KEY_NUM) return 0U;

    ret = key_states[key].key_long_bit;
    key_states[key].key_long_bit = 0U;
    return ret;
}
