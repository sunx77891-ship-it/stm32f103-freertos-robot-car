#include "oled_i2c.h"

/* 软件 I2C 延时 — 72MHz 下约 2us
   满足 400kHz 快速模式 I2C 时序: tLOW>=1.3us, tHIGH>=0.6us
   全屏刷新约 60ms, 低于任务周期 100ms
   volatile 防止编译器优化掉循环 */
static void OLED_I2C_Delay(void)
{
    volatile uint16_t i = 48;
    while (i--);
}

/* 初始化 PB5(SCL) 和 PB4(SDA) 为开漏输出，启用内部上拉
   注: 0.96 寸 OLED 模块板上通常已自带 4.7k 上拉电阻 */
void OLED_I2C_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin   = OLED_SCL_PIN | OLED_SDA_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;   /* 开漏输出，支持 I2C 线与 */
    GPIO_InitStruct.Pull  = GPIO_PULLUP;            /* 启用内部上拉 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;    /* 软件 I2C 用低速即可，降低边沿反射 */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 空闲状态：SDA 和 SCL 都拉高 */
    OLED_SDA_HIGH();
    OLED_SCL_HIGH();
}

/* I2C 起始信号：SCL 高时 SDA 下降沿 */
void OLED_I2C_Start(void)
{
    OLED_SDA_HIGH();
    OLED_SCL_HIGH();
    OLED_I2C_Delay();
    OLED_SDA_LOW();        /* SCL 高时 SDA 下降沿 = START */
    OLED_I2C_Delay();
    OLED_SCL_LOW();        /* 拉低 SCL 准备传输 */
}

/* I2C 停止信号：SCL 高时 SDA 上升沿 */
void OLED_I2C_Stop(void)
{
    OLED_SDA_LOW();
    OLED_SCL_HIGH();
    OLED_I2C_Delay();
    OLED_SDA_HIGH();       /* SCL 高时 SDA 上升沿 = STOP */
    OLED_I2C_Delay();
}

/* 发送一个字节，高位在前
   每位时序: SDA建立 -> SCL高(tHIGH) -> SCL低(tLOW) */
void OLED_I2C_WriteByte(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++) {
        if (data & 0x80)
            OLED_SDA_HIGH();
        else
            OLED_SDA_LOW();
        data <<= 1;
        OLED_I2C_Delay();     /* SDA 建立时间 tSU */
        OLED_SCL_HIGH();       /* 拉高 SCL，从机采样 */
        OLED_I2C_Delay();     /* SCL 高电平时间 tHIGH */
        OLED_SCL_LOW();        /* 拉低 SCL */
        OLED_I2C_Delay();     /* SCL 低电平时间 tLOW */
    }
}

/* 等待从机 ACK — 返回 0 表示收到 ACK，1 表示 NACK */
uint8_t OLED_I2C_WaitAck(void)
{
    uint8_t ack;

    OLED_SDA_HIGH();       /* 释放 SDA，等从机拉低 */
    OLED_I2C_Delay();     /* SDA 建立时间 tSU */
    OLED_SCL_HIGH();       /* 拉高 SCL 产生 ACK 时钟 */
    OLED_I2C_Delay();     /* SCL 高电平时间 tHIGH */
    ack = OLED_SDA_READ(); /* 读取 SDA 电平: 0=ACK, 1=NACK */
    OLED_SCL_LOW();        /* 拉低 SCL */
    OLED_I2C_Delay();     /* SCL 低电平时间 tLOW */
    return ack;
}
