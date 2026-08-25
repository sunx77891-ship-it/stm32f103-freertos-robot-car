#ifndef GYRO_H
#define GYRO_H

#include <stdint.h>

/* 陀螺仪物理量 */
typedef struct {
    float wz;      /* 角速度 (°/s)，TYPE=0xAA 更新 */
    float yaw;     /* 航向角 (°)，  TYPE=0xBB 更新 */
    uint8_t fresh; /* bit0=Wz 新数据, bit1=Yaw 新数据，读后清 */
} Gyro_Data_t;

/* 外部可访问的最新数据 */
extern volatile Gyro_Data_t g_gyro;

/**************************************************************************
函数名：Gyro_Start
函数功能：启动 USART3 DMA 接收，开始解析陀螺仪数据帧
入口参数：无
返回值：无
说明：在 FreeRTOS 任务启动后调用
**************************************************************************/
void Gyro_Start(void);
uint8_t Gyro_Read(Gyro_Data_t *out, uint32_t timeout_ms);
void SerialCmd_Start(void);
uint8_t SerialCmd_GetCmd(void);

#endif /* GYRO_H */
