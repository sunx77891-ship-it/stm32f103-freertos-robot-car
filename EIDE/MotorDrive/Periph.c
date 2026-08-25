#include "Periph.h"
#include "main.h"
#include "cmsis_os.h"

/* ====== 蜂鸣器 ====== */
void Buzzer_On(void)  { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); }
void Buzzer_Off(void) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); }

void Buzzer_Beep(uint16_t on_ms, uint16_t off_ms, uint8_t times)
{
    for (uint8_t i = 0; i < times; i++)
    {
        Buzzer_On();
        osDelay(on_ms);
        Buzzer_Off();
        if (i < times - 1)
            osDelay(off_ms);
    }
}

/* ====== LED ====== */
void LED_On(uint8_t mask)
{
    if (mask & LED_B12) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    if (mask & LED_B13) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    if (mask & LED_B14) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    if (mask & LED_B15) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    if (mask & LED_PC6) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  GPIO_PIN_SET);
}

void LED_Off(uint8_t mask)
{
    if (mask & LED_B12) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    if (mask & LED_B13) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    if (mask & LED_B14) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    if (mask & LED_B15) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    if (mask & LED_PC6) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  GPIO_PIN_RESET);
}

void LED_Toggle(uint8_t mask)
{
    if (mask & LED_B12) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
    if (mask & LED_B13) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
    if (mask & LED_B14) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
    if (mask & LED_B15) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
    if (mask & LED_PC6) HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
}

/**************************************************************************
函数名：LED_AllOff
函数功能：熄灭 LED1~LED5，LED 控制引脚均输出低电平
入口参数：无
返回值：无
**************************************************************************/
void LED_AllOff(void)
{
    LED_Off(LED_ALL);
}
