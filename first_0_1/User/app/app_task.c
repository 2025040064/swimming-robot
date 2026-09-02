#include "app_task.h"
#include "app_state_machine.h"
#include "app_protocol.h"
#include "app_control.h"
#include "bsp_ultrasonic.h"
#include "bsp_mpu6050.h"
#include "bsp_iic.h"
#include "bsp_led.h"
#include "bsp_usart.h"
#include "bsp_systick.h"
#include "bsp_adc.h"
#include "bsp_debug.h"
#include "algorithm/algo_filter.h"

static uint32_t g_lastRun[TASK_COUNT];
static const uint32_t g_period[TASK_COUNT] =
{
    10U, 20U, 50U, 100U, 200U, 500U, 1000U
};

static int16_t g_accel[3];
static int16_t g_gyro[3];
static uint32_t g_lastAttiUpdate = 0U;

#define K230_RX_PROCESS_LIMIT  128U

void App_Task_Init(void)
{
    uint8_t i;
    uint32_t now = BSP_GetTick();

    for (i = 0U; i < TASK_COUNT; i++)
        g_lastRun[i] = now;
    g_lastAttiUpdate = now;
}

void App_Task_Scheduler(void)
{
    uint32_t now = BSP_GetTick();
    uint8_t ch;
    uint16_t processed;

    if ((now - g_lastRun[TASK_10MS]) >= g_period[TASK_10MS])
    {
        g_lastRun[TASK_10MS] += g_period[TASK_10MS];
        processed = 0U;
        while (BSP_K230_RxAvailable() && (processed < K230_RX_PROCESS_LIMIT))
        {
            ch = BSP_K230_GetRxByte();
            App_Protocol_ParseByte(ch);
            processed++;
        }

        if (BSP_MPU6050_ReadData(g_accel, g_gyro) == BSP_IIC_OK)
        {
            float dt = (now - g_lastAttiUpdate) * 0.001f;
            if (dt > 0.1f) dt = 0.1f;
            g_lastAttiUpdate = now;
            Algo_Filter_Update(g_accel, g_gyro, dt);
        }
    }

    if ((now - g_lastRun[TASK_20MS]) >= g_period[TASK_20MS])
    {
        g_lastRun[TASK_20MS] += g_period[TASK_20MS];
        App_SM_Run();
    }

    if ((now - g_lastRun[TASK_50MS]) >= g_period[TASK_50MS])
        g_lastRun[TASK_50MS] += g_period[TASK_50MS];

    if ((now - g_lastRun[TASK_100MS]) >= g_period[TASK_100MS])
    {
        g_lastRun[TASK_100MS] += g_period[TASK_100MS];
        BSP_LED_Toggle();
    }

    if ((now - g_lastRun[TASK_200MS]) >= g_period[TASK_200MS])
    {
        g_lastRun[TASK_200MS] += g_period[TASK_200MS];
        App_Protocol_SendStatus(App_SM_GetStateName(),
                                Algo_Filter_GetPitch(),
                                Algo_Filter_GetRoll(),
                                BSP_Ultrasonic_GetFront(),
                                BSP_Ultrasonic_GetLeft(),
                                BSP_Ultrasonic_GetRight());
    }

    if ((now - g_lastRun[TASK_500MS]) >= g_period[TASK_500MS])
        g_lastRun[TASK_500MS] += g_period[TASK_500MS];

    if ((now - g_lastRun[TASK_1000MS]) >= g_period[TASK_1000MS])
    {
        static uint32_t lastOre = 0U;
        uint32_t ore;

        g_lastRun[TASK_1000MS] += g_period[TASK_1000MS];
        ore = BSP_K230_GetOreCount();
        if (ore != lastOre)
        {
            DBG_PRINT("[WARN] K230 ORE: %lu\n", (unsigned long)(ore - lastOre));
            lastOre = ore;
        }
    }
}

uint32_t App_Task_GetDelay(void)
{
    return 1U;
}
