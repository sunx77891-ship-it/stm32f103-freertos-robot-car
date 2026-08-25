#ifndef _OLED_I2C_H_
#define _OLED_I2C_H_

#include "main.h"

/* OLED I2C 引脚定义 — SCL: PB5, SDA: PB4, 开漏输出 */
#define OLED_SCL_PORT    GPIOB
#define OLED_SCL_PIN     GPIO_PIN_4
#define OLED_SDA_PORT    GPIOB
#define OLED_SDA_PIN     GPIO_PIN_5

/* 引脚控制宏（开漏输出 + 内部上拉，模拟 I2C 时序） */
#define OLED_SDA_HIGH()  HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_SET)
#define OLED_SDA_LOW()   HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_RESET)
#define OLED_SCL_HIGH()  HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_SET)
#define OLED_SCL_LOW()   HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_RESET)
#define OLED_SDA_READ()  HAL_GPIO_ReadPin(OLED_SDA_PORT, OLED_SDA_PIN)

void OLED_I2C_GPIO_Init(void);
void OLED_I2C_Start(void);
void OLED_I2C_Stop(void);
void OLED_I2C_WriteByte(uint8_t data);
uint8_t OLED_I2C_WaitAck(void);

#endif /* _OLED_I2C_H_ */
