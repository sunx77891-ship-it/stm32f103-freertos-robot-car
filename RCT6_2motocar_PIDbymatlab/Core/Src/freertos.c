/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "oled_i2c.h"
#include "Ctrl.h"
#include "Gyro.h"
#include "Key.h"
#include "Periph.h"
#include "usart.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    RECT_PATH_IDLE = 0,
    RECT_PATH_SIDE,
    RECT_PATH_BRAKE,
    RECT_PATH_TURN,
    RECT_PATH_AFTER_TURN
} RectPathState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CTRL_FAST_MS    10U   /* 速度环周期 */
#define CTRL_SLOW_MS    50U   /* 位置/角度环周期 */
#define KEY_SPEED_TARGET 30    /* 按键速度目标，单位：脉冲/10ms */
#define OLED_POWER_STABLE_MS 150U
#define STARTUP_BEEP_ON_MS    80U
#define STARTUP_BEEP_OFF_MS   80U
#define STARTUP_BEEP_COUNT     3U
#define RECT_SIDE_PULSES       20000L  /* 每条边的编码器平均脉冲数 */
#define RECT_SPEED_FAST           30   /* 巡航速度，脉冲/10ms */
#define RECT_SPEED_MEDIUM         20   /* 第一减速段速度 */
#define RECT_SPEED_SLOW           10   /* 角点前低速 */
#define RECT_DECEL_MEDIUM       1500L  /* 剩余 1500 脉冲进入第一减速段 */
#define RECT_DECEL_SLOW          500L  /* 剩余 500 脉冲进入低速段 */
#define RECT_STOP_MARGIN          40L  /* 允许的角点距离误差 */
#define RECT_CLOCKWISE_ANGLE    90.0f  /* 本工程正角度对应顺时针旋转 */
#define RECT_BRAKE_TICKS          20U  /* 角点主动刹车 200ms */
#define RECT_AFTER_TURN_TICKS     10U  /* 转向后静置 100ms */
#define RECT_TURN_SETTLE_TICKS    20U  /* 连续稳定 200ms 才认为转向完成 */
#define RECT_YAW_TOLERANCE       3.0f  /* 转向完成角度误差，单位：度 */
#define RECT_WZ_TOLERANCE        5.0f  /* 转向完成角速度阈值，单位：度/秒 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static char        uart_tx_buf[64];
static volatile uint8_t uart_tx_busy = 0U;
static volatile uint8_t oled_ready = 0U;
static RectPathState_t rect_path_state = RECT_PATH_IDLE;
static uint8_t rect_side_index = 0U;
static uint8_t rect_wait_ticks = 0U;
static uint8_t rect_turn_settle_ticks = 0U;
static int16_t rect_speed_command = 0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId oledTaskHandle;
osThreadId keyTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void Startup_RunRightLight(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void OLED_DisplayTask(void const * argument);
void Key_ScanTask(void const * argument);

void MX_FREERTOS_Init(void);

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(oledTask, OLED_DisplayTask, osPriorityLow, 0, 256);
  oledTaskHandle = osThreadCreate(osThread(oledTask), NULL);

  osThreadDef(keyTask, Key_ScanTask, osPriorityLow, 0, 128);
  keyTaskHandle = osThreadCreate(osThread(keyTask), NULL);
}

/**************************************************************************
函数名：StartDefaultTask
函数功能：完成控制器、OLED、通信和按键初始化，执行上电声光提示，并运行 10ms/50ms 控制周期
入口参数：argument - FreeRTOS 任务参数，当前未使用
返回值：无
**************************************************************************/
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  TickType_t last_fast;
  uint32_t   slow_cnt = 0U;

  (void)argument;

  Buzzer_Off();
  Ctrl_Init();
  Gyro_Start();
  SerialCmd_Start();
  Key_Init();

  /* 冷启动先释放 I2C 总线，等待 OLED 电源稳定后再发送初始化序列 */
  OLED_I2C_GPIO_Init();
  osDelay(OLED_POWER_STABLE_MS);
  OLED_Init();

  OLED_Clear();
  OLED_ShowString16(8, 24, "System Ready");
  OLED_Refresh();
  oled_ready = 1U;

  /* 三声短鸣期间，流水灯从 LED1 开始向右同步移动 */
  Startup_RunRightLight();

  last_fast = xTaskGetTickCount();

  for (;;)
  {
    vTaskDelayUntil(&last_fast, pdMS_TO_TICKS(CTRL_FAST_MS));

    Ctrl_Update10ms();

    slow_cnt++;
    if (slow_cnt >= (CTRL_SLOW_MS / CTRL_FAST_MS))
    {
      slow_cnt = 0U;
      Ctrl_Update50ms();
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**************************************************************************
函数名：Startup_RunRightLight
函数功能：上电三声蜂鸣期间驱动流水灯向右移动，蜂鸣时点亮、静音时熄灭
入口参数：无
返回值：无
**************************************************************************/
static void Startup_RunRightLight(void)
{
    static const uint8_t led_order[] = {LED1, LED2, LED3, LED4, LED5};
    uint8_t i;

    LED_AllOff();

    for (i = 0U; i < STARTUP_BEEP_COUNT; i++)
    {
        LED_On(led_order[i % (sizeof(led_order) / sizeof(led_order[0]))]);
        Buzzer_On();
        osDelay(STARTUP_BEEP_ON_MS);

        Buzzer_Off();
        LED_AllOff();

        if (i + 1U < STARTUP_BEEP_COUNT)
            osDelay(STARTUP_BEEP_OFF_MS);
    }
}

/**************************************************************************
函数名：OLED_DisplayTask
函数功能：周期刷新 OLED，并通过 USART1 输出控制器诊断数据
入口参数：argument - FreeRTOS 任务参数，当前未使用
返回值：无
**************************************************************************/
void OLED_DisplayTask(void const * argument)
{
    char line[17];
    TickType_t last_wake;
    Ctrl_Status_t snap;
    const char *mode_str[] = {"STOP", "FWD", "TURN", "POS"};

    (void)argument;

    /* 等待默认任务完成 OLED 冷启动初始化，避免两个任务同时访问软件 I2C */
    while (!oled_ready)
        osDelay(10);

    last_wake = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(200));

        taskENTER_CRITICAL();
        snap = g_ctrl;
        taskEXIT_CRITICAL();

        OLED_Clear();

        sprintf(line, "VA:%+4d VB:%+4d", snap.speed_a, snap.speed_b);
        OLED_ShowString16(0, 0, line);

        sprintf(line, "Y:%+4d WZ:%+4d", (int)(snap.yaw * 10.0f), (int)(snap.wz * 10.0f));
        OLED_ShowString16(0, 16, line);

        sprintf(line, "P:%+6ld Tp:%+5ld", (long)snap.pos_a, (long)snap.tar_pos_a);
        OLED_ShowString16(0, 32, line);

        if (snap.mode < 4)
            sprintf(line, "Yt:%+4d %s", (int)(snap.tar_yaw * 10.0f), mode_str[snap.mode]);
        else
            sprintf(line, "Yt:%+4d ?", (int)(snap.tar_yaw * 10.0f));
        OLED_ShowString16(0, 48, line);


        OLED_Refresh();

        if (!uart_tx_busy)
        {
            // int n = sprintf(uart_tx_buf, "%d,%d,%d,%d,%d,%d,%d,%d\n",
            //     (int)snap.yaw, (int)snap.tar_yaw,
            //     snap.target_speed_a, snap.target_speed_b,
            //     snap.speed_a, snap.speed_b,
            //     snap.ccr_a, snap.ccr_b);
            int n = sprintf(uart_tx_buf, "%d\n",(int)snap.yaw);
            uart_tx_busy = 1U;
            if (HAL_UART_Transmit_DMA(&huart1, (uint8_t*)uart_tx_buf, n) != HAL_OK)
                uart_tx_busy = 0U;
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
        uart_tx_busy = 0U;
}

/**************************************************************************
函数名：RectPath_WrapAngle
函数功能：将角度误差归一化到 -180°~+180°
入口参数：angle - 待归一化的角度，单位：度
返回值：归一化后的角度
**************************************************************************/
static float RectPath_WrapAngle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**************************************************************************
函数名：RectPath_SetSpeed
函数功能：仅在矩形路径目标速度变化时更新双轮速度
入口参数：speed - 双轮目标速度，单位：脉冲/10ms
返回值：无
**************************************************************************/
static void RectPath_SetSpeed(int16_t speed)
{
    if (speed != rect_speed_command)
    {
        rect_speed_command = speed;
        Ctrl_SetTargetSpeed(speed, speed);
    }
}

/**************************************************************************
函数名：RectPath_StartSide
函数功能：清零本边里程，保留规划航向并启动直线巡航
入口参数：无
返回值：无
**************************************************************************/
static void RectPath_StartSide(void)
{
    Ctrl_SetMode(STOP);
    Ctrl_ZeroPos();

    rect_speed_command = RECT_SPEED_FAST;
    Ctrl_SetTargetSpeed(RECT_SPEED_FAST, RECT_SPEED_FAST);
    rect_path_state = RECT_PATH_SIDE;
}

/**************************************************************************
函数名：RectPath_Start
函数功能：从第一条边启动四边等长的顺时针矩形路径
入口参数：无
返回值：无
**************************************************************************/
static void RectPath_Start(void)
{
    rect_side_index = 0U;
    rect_wait_ticks = 0U;
    rect_turn_settle_ticks = 0U;
    Ctrl_SetMode(STOP);
    Ctrl_CalibrateYaw();
    RectPath_StartSide();
}

/**************************************************************************
函数名：RectPath_Stop
函数功能：取消矩形路径并立即主动刹车
入口参数：无
返回值：无
**************************************************************************/
static void RectPath_Stop(void)
{
    rect_path_state = RECT_PATH_IDLE;
    rect_side_index = 0U;
    rect_wait_ticks = 0U;
    rect_turn_settle_ticks = 0U;
    rect_speed_command = 0;
    Ctrl_SetMode(STOP);
}

/**************************************************************************
函数名：RectPath_Update
函数功能：每 10ms 推进直行、刹车、顺时针转向和下一边状态
入口参数：无
返回值：无
**************************************************************************/
static void RectPath_Update(void)
{
    Ctrl_Status_t snap;
    int32_t travelled;
    int32_t remaining;
    float yaw_error;
    float wz_abs;

    if (rect_path_state == RECT_PATH_IDLE)
        return;

    taskENTER_CRITICAL();
    snap = g_ctrl;
    taskEXIT_CRITICAL();

    switch (rect_path_state)
    {
    case RECT_PATH_SIDE:
        if (snap.mode != FORWARD)
        {
            RectPath_Stop();
            break;
        }

        travelled = (snap.pos_a + snap.pos_b) / 2;
        if (travelled < 0) travelled = -travelled;
        remaining = RECT_SIDE_PULSES - travelled;

        if (remaining <= RECT_STOP_MARGIN)
        {
            Ctrl_SetMode(STOP);
            rect_speed_command = 0;
            rect_wait_ticks = RECT_BRAKE_TICKS;
            rect_path_state = RECT_PATH_BRAKE;
        }
        else if (remaining <= RECT_DECEL_SLOW)
        {
            RectPath_SetSpeed(RECT_SPEED_SLOW);
        }
        else if (remaining <= RECT_DECEL_MEDIUM)
        {
            RectPath_SetSpeed(RECT_SPEED_MEDIUM);
        }
        else
        {
            RectPath_SetSpeed(RECT_SPEED_FAST);
        }
        break;

    case RECT_PATH_BRAKE:
        if (rect_wait_ticks > 0U)
        {
            rect_wait_ticks--;
        }
        else
        {
            rect_turn_settle_ticks = 0U;
            Ctrl_AddTargetYaw(RECT_CLOCKWISE_ANGLE);
            rect_path_state = RECT_PATH_TURN;
        }
        break;

    case RECT_PATH_TURN:
        if (snap.mode != TURN)
        {
            RectPath_Stop();
            break;
        }

        yaw_error = RectPath_WrapAngle(snap.tar_yaw - snap.yaw);
        if (yaw_error < 0.0f) yaw_error = -yaw_error;
        wz_abs = (snap.wz < 0.0f) ? -snap.wz : snap.wz;

        if (yaw_error <= RECT_YAW_TOLERANCE && wz_abs <= RECT_WZ_TOLERANCE)
        {
            if (rect_turn_settle_ticks < RECT_TURN_SETTLE_TICKS)
                rect_turn_settle_ticks++;
        }
        else
        {
            rect_turn_settle_ticks = 0U;
        }

        if (rect_turn_settle_ticks >= RECT_TURN_SETTLE_TICKS)
        {
            Ctrl_SetMode(STOP);
            rect_side_index++;

            if (rect_side_index >= 4U)
            {
                rect_path_state = RECT_PATH_IDLE;
                rect_speed_command = 0;
            }
            else
            {
                rect_wait_ticks = RECT_AFTER_TURN_TICKS;
                rect_path_state = RECT_PATH_AFTER_TURN;
            }
        }
        break;

    case RECT_PATH_AFTER_TURN:
        if (rect_wait_ticks > 0U)
        {
            rect_wait_ticks--;
        }
        else
        {
            RectPath_StartSide();
        }
        break;

    default:
        RectPath_Stop();
        break;
    }
}

/**************************************************************************
函数名：Key_ScanTask
函数功能：每 10ms 扫描 K1~K4 和串口命令；K1 前进、K2 后退、K3 急停
          K4 启动或取消顺时针固定矩形路径，K5~K6 保持禁用
入口参数：argument - FreeRTOS 任务参数，当前未使用
返回值：无
**************************************************************************/
void Key_ScanTask(void const * argument)
{
    (void)argument;

    for (;;)
    {
        uint8_t scmd = SerialCmd_GetCmd();
        uint8_t k1_pressed;
        uint8_t k2_pressed;
        uint8_t k3_pressed;
        uint8_t k4_pressed;

        Key_Process();
        k1_pressed = Key_GetPress(KEY_K1);
        k2_pressed = Key_GetPress(KEY_K2);
        k3_pressed = Key_GetPress(KEY_K3);
        k4_pressed = Key_GetPress(KEY_K4);

        /* 急停优先于同周期到达的任何运动命令 */
        if (scmd == 3U || k3_pressed)
        {
            RectPath_Stop();
        }
        else if (scmd == 1U || k1_pressed)
        {
            RectPath_Stop();
            /* 双轮闭环前进：锁定当前航向，目标 +30 脉冲/10ms */
            Ctrl_CalibrateYaw();
            Ctrl_SetMode(FORWARD);
            Ctrl_SetTargetSpeed(KEY_SPEED_TARGET, KEY_SPEED_TARGET);
        }
        else if (scmd == 2U || k2_pressed)
        {
            RectPath_Stop();
            /* 双轮闭环后退：锁定当前航向，目标 -30 脉冲/10ms */
            Ctrl_CalibrateYaw();
            Ctrl_SetMode(FORWARD);
            Ctrl_SetTargetSpeed(-KEY_SPEED_TARGET, -KEY_SPEED_TARGET);
        }
        else if (scmd == 4U || k4_pressed)
        {
            if (rect_path_state == RECT_PATH_IDLE)
                RectPath_Start();
            else
                RectPath_Stop();
        }
        else
        {
            RectPath_Update();
        }
        osDelay(10);
    }
}

/* USER CODE END Application */
