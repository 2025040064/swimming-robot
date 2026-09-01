#include "app_task.h"
#include "app_state_machine.h"
#include "app_protocol.h"
#include "app_control.h"
#include "bsp_ultrasonic.h"
#include "bsp_mpu6050.h"
#include "bsp_led.h"
#include "bsp_usart.h"
#include "bsp_systick.h"
#include "bsp_adc.h"
#include "bsp_debug.h"
#include "algorithm/algo_filter.h"
#include "algorithm/algo_pid.h"

static uint32_t g_lastRun[TASK_COUNT];
static const uint32_t g_period[TASK_COUNT] =
{
    10,  20,  50,  100,  200,  500,  1000
};

static int16_t g_accel[3], g_gyro[3];
static uint32_t g_lastAttiUpdate = 0;

void App_Task_Init(void)
{
    uint8_t i;
    uint32_t now = BSP_GetTick();
    for (i = 0; i < TASK_COUNT; i++)
        g_lastRun[i] = now;
    g_lastAttiUpdate = now;
}

void App_Task_Scheduler(void)
{
    uint32_t now = BSP_GetTick();
    uint8_t ch;

    /* ---- 10ms: USART poll + MPU6050 + attitude update ---- */
    if (now - g_lastRun[TASK_10MS] >= g_period[TASK_10MS])
    {
        g_lastRun[TASK_10MS] += g_period[TASK_10MS];  /* use += to prevent burst on catch-up */

        while (BSP_USART_RxAvailable())
        {
            ch = BSP_USART_GetRxByte();
            App_Protocol_ParseByte(ch);
        }

        if (BSP_MPU6050_ReadData(g_accel, g_gyro) == 0)
        {
            float dt = (now - g_lastAttiUpdate) * 0.001f;
            if (dt > 0.1f) dt = 0.1f;
            g_lastAttiUpdate = now;
            Algo_Filter_Update(g_accel, g_gyro, dt);
        }
    }

    /* ---- 20ms: state machine ---- */
    if (now - g_lastRun[TASK_20MS] >= g_period[TASK_20MS])
    {
        g_lastRun[TASK_20MS] += g_period[TASK_20MS];
        App_SM_Run();
    }

    /* ---- 50ms: attitude correction on motors (reserved) ---- */
    if (now - g_lastRun[TASK_50MS] >= g_period[TASK_50MS])
    {
        g_lastRun[TASK_50MS] += g_period[TASK_50MS];
    }

    /* ---- 100ms: LED heartbeat ---- */
    if (now - g_lastRun[TASK_100MS] >= g_period[TASK_100MS])
    {
        g_lastRun[TASK_100MS] += g_period[TASK_100MS];
        BSP_LED_Toggle();
    }

    /* ---- 200ms: status telemetry ---- */
    if (now - g_lastRun[TASK_200MS] >= g_period[TASK_200MS])
    {
        g_lastRun[TASK_200MS] += g_period[TASK_200MS];
        App_Protocol_SendStatus(
            App_SM_GetStateName(),
            Algo_Filter_GetPitch(),
            Algo_Filter_GetRoll(),
            BSP_Ultrasonic_GetFront(),
            BSP_Ultrasonic_GetLeft(),
            BSP_Ultrasonic_GetRight()
        );
    }

    /* ---- 500ms: reserved ---- */
    if (now - g_lastRun[TASK_500MS] >= g_period[TASK_500MS])
    {
        g_lastRun[TASK_500MS] += g_period[TASK_500MS];
    }

    /* ---- 1000ms: battery ADC + ORE monitor ---- */
    if (now - g_lastRun[TASK_1000MS] >= g_period[TASK_1000MS])
    {
        g_lastRun[TASK_1000MS] += g_period[TASK_1000MS];

        /* Battery ADC is disabled (PA8 was not a valid ADC pin; see bsp_adc.c).
         * No battery read here until the hardware is rewired to a real ADC pin. */

        /* USART overrun watchdog: warn if bytes were lost since last check */
        {
            static uint32_t g_lastOre = 0;
            uint32_t ore = BSP_USART_GetOreCount();
            if (ore != g_lastOre)
            {
                DBG_PRINT("[WARN] USART ORE: %lu overruns\n",
                          (unsigned long)(ore - g_lastOre));
                g_lastOre = ore;
            }
        }
    }
}

uint32_t App_Task_GetDelay(void)
{
    return 1;
}
