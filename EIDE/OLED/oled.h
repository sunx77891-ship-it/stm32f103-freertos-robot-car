#ifndef _OLED_H_
#define _OLED_H_

#include "oled_i2c.h"
#include <string.h>

/* SSD1306 分辨率 — 0.96 寸 128x64 */
#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_PAGES   (OLED_HEIGHT / 8)

/* SSD1306 I2C 地址 */
#define OLED_ADDR     0x3C
#define OLED_WRITE    (OLED_ADDR << 1)

/* 8x16 字库，定义在 oled.c */
extern const uint8_t Font8x16[];

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh(void);
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void OLED_ShowChar16(uint8_t x, uint8_t y, char ch);
void OLED_ShowString16(uint8_t x, uint8_t y, const char *str);

#endif
