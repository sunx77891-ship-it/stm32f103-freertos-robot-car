#include "Gyro.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

/* ===== 协议常量 ===== */
#define FRAME_HEAD   0x5AU
#define TYPE_WZ      0xAAU
#define TYPE_YAW     0xBBU

/* ===== 串口命令 ===== */
#define CMD_BUF      4             /* "K5\0" 最长 3 字符 */
static uint8_t g_cmd_buf[CMD_BUF];
static uint8_t g_cmd_idx = 0;
static volatile uint8_t g_cmd_pending = 0U;  /* bit0=K1 … bit4=K5 */
static uint8_t g_cmd_byte;                    /* USART1 单字节 DMA */
static uint8_t g_gyro_byte;                   /* USART3 单字节 DMA */

/* ===== 全局数据 ===== */
volatile Gyro_Data_t g_gyro = {0};

/* ===== 帧接收状态机 ===== */
typedef enum {
    ST_IDLE,     /* 等待 0x5A */
    ST_TYPE,     /* 已收 TYPE */
    ST_LOW,      /* 已收 L */
    ST_HIGH,     /* 已收 H */
    ST_CRC       /* 已收 CRC，校验 */
} GyroRxState;

static volatile GyroRxState g_rx_state = ST_IDLE;
static uint8_t g_frame[5];             /* 当前帧缓冲区（仅 ISR 访问） */
static uint8_t g_crc;                  /* 累计校验和 */
static SemaphoreHandle_t g_gyro_sem = NULL;  /* 帧就绪信号量 */

/* ===== 前向声明 ===== */
static void gyro_fsm(uint8_t byte);
static void gyro_parse(const uint8_t *frame);

/**************************************************************************
状态机 — 逐字节解析 0x5A TYPE L H CRC
**************************************************************************/
static void gyro_fsm(uint8_t byte)
{
    switch (g_rx_state)
    {
    case ST_IDLE:
        if (byte == FRAME_HEAD)
        {
            g_frame[0] = byte;
            g_crc = byte;
            g_rx_state = ST_TYPE;
        }
        break;

    case ST_TYPE:
        g_frame[1] = byte;
        g_crc += byte;
        g_rx_state = ST_LOW;
        break;

    case ST_LOW:
        g_frame[2] = byte;
        g_crc += byte;
        g_rx_state = ST_HIGH;
        break;

    case ST_HIGH:
        g_frame[3] = byte;
        g_crc += byte;
        g_rx_state = ST_CRC;
        break;

    case ST_CRC:
        g_frame[4] = byte;
        /* 校验和 = 前 4 字节累加，取低 8 位 */
        if (((uint8_t)g_crc) == byte)
        {
            gyro_parse(g_frame);
        }
        g_rx_state = ST_IDLE;
        break;
    }
}

/**************************************************************************
数据解析 — 从 5 字节帧提取物理量
  TYPE=0xAA → Wz = raw / 32768 * 2000  (°/s)
  TYPE=0xBB → Yaw = raw / 32768 * 180  (°)
  高低字节拼接：raw = (short)(H << 8 | L)
**************************************************************************/
static void gyro_parse(const uint8_t *frame)
{
    int16_t raw;
    BaseType_t need_yield = pdFALSE;

    raw = (int16_t)(((uint16_t)frame[3] << 8U) | frame[2]);

    if (frame[1] == TYPE_WZ)
    {
        g_gyro.wz = (float)raw / 32768.0f * 2000.0f;
        g_gyro.fresh |= 0x01U;
    }
    else if (frame[1] == TYPE_YAW)
    {
        g_gyro.yaw = (float)raw / 32768.0f * 180.0f;
        g_gyro.fresh |= 0x02U;
    }

    /* 通知任务 */
    if (g_gyro_sem != NULL)
    {
        xSemaphoreGiveFromISR(g_gyro_sem, &need_yield);
        portYIELD_FROM_ISR(need_yield);
    }
}

/**************************************************************************
函数名：HAL_UART_RxCpltCallback
函数功能：处理 USART3 陀螺仪字节和 USART1 K1~K5 命令的 DMA 接收完成中断
入口参数：huart - 触发接收完成中断的 UART 句柄
返回值：无
**************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        gyro_fsm(g_gyro_byte);
        HAL_UART_Receive_DMA(&huart3, &g_gyro_byte, 1);
    }
    else if (huart->Instance == USART1)
    {
        uint8_t b = g_cmd_byte;
        if (b == '\n' || b == '\r')
        {
            /* 命令格式 "K1"~"K5" */
            if (g_cmd_idx >= 2 && g_cmd_buf[0] == 'K')
            {
                uint8_t n = g_cmd_buf[1] - '0';
                if (n >= 1 && n <= 5)
                    g_cmd_pending |= (1U << (n - 1));
            }
            g_cmd_idx = 0;
        }
        else if (g_cmd_idx < CMD_BUF - 1)
        {
            g_cmd_buf[g_cmd_idx++] = b;
        }
        HAL_UART_Receive_DMA(&huart1, &g_cmd_byte, 1);
    }
}

/**************************************************************************
函数名：SerialCmd_GetCmd
函数功能：读取并清除串口命令标志（非阻塞）
返回值：1-5=K1-K5, 0=无命令
**************************************************************************/
uint8_t SerialCmd_GetCmd(void)
{
    uint8_t i;
    for (i = 0; i < 5; i++)
    {
        if (g_cmd_pending & (1U << i))
        {
            g_cmd_pending &= ~(1U << i);
            return i + 1;
        }
    }
    return 0U;
}

/**************************************************************************
函数名：SerialCmd_Start
函数功能：启动 USART1 单字节 DMA 接收（命令行）
**************************************************************************/
void SerialCmd_Start(void)
{
    g_cmd_idx = 0;
    g_cmd_pending = 0U;
    memset(g_cmd_buf, 0, CMD_BUF);

    /* 先停 DMA TX，防与 DMA RX 争抢 USART1 */
    HAL_UART_DMAStop(&huart1);

    if (HAL_UART_Receive_DMA(&huart1, &g_cmd_byte, 1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**************************************************************************
函数名：Gyro_Start
函数功能：初始化信号量，启动 USART3 单字节 DMA 接收
入口参数：无
返回值：无
**************************************************************************/
void Gyro_Start(void)
{
    if (g_gyro_sem == NULL)
    {
        g_gyro_sem = xSemaphoreCreateBinary();
    }

    g_rx_state = ST_IDLE;
    g_gyro.wz = 0.0f;
    g_gyro.yaw = 0.0f;
    g_gyro.fresh = 0U;

    /* 启动 USART3 DMA 单字节接收 */
    if (HAL_UART_Receive_DMA(&huart3, &g_gyro_byte, 1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**************************************************************************
函数名：Gyro_Read
函数功能：等待新数据并原子读取（清除 fresh 标志）
入口参数：out - 输出缓冲区
          timeout_ms - 超时(ms)，0=不等待
返回值：1=读到新数据  0=超时或无新数据
**************************************************************************/
uint8_t Gyro_Read(Gyro_Data_t *out, uint32_t timeout_ms)
{
    Gyro_Data_t tmp;

    if (out == NULL)
        return 0U;

    /* 等待信号量（ISR 每帧给一次） */
    if (timeout_ms > 0U)
    {
        if (xSemaphoreTake(g_gyro_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
            return 0U;
    }
    else if (g_gyro.fresh == 0U)
    {
        return 0U;
    }

    /* 临界区内原子拷贝 + 清标志 */
    taskENTER_CRITICAL();
    tmp = g_gyro;
    g_gyro.fresh = 0U;
    taskEXIT_CRITICAL();

    *out = tmp;
    return 1U;
}
