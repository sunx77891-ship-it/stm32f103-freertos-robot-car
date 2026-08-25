#ifndef PERIPH_H
#define PERIPH_H

#include <stdint.h>

/* ====== 蜂鸣器 PC15 (低电平有效) ====== */
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_Beep(uint16_t on_ms, uint16_t off_ms, uint8_t times);

/* ====== LED ====== */
/* PB12~PB15, PC6 — 高电平点亮 */
#define LED_B12  0x01
#define LED_B13  0x02
#define LED_B14  0x04
#define LED_B15  0x08
#define LED_PC6  0x10

/* LED1~LED5 在车体上从左到右排列 */
#define LED1     LED_PC6
#define LED2     LED_B15
#define LED3     LED_B14
#define LED4     LED_B13
#define LED5     LED_B12
#define LED_ALL  (LED1 | LED2 | LED3 | LED4 | LED5)

void LED_On(uint8_t mask);
void LED_Off(uint8_t mask);
void LED_Toggle(uint8_t mask);
void LED_AllOff(void);

#endif
