#include "app_task.h"
#include "app_state_machine.h"
#include "app_protocol.h"
#include "app_control.h"
#include "app_navigation.h"
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
static int16_t g_mag[3];
static uint32_t g_lastAttiUpdate = 0;

#define K230_RX_PROCESS_LIMIT   128U
#define GPS_RX_PROCESS_LIMIT    128U

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
    uint16_t processed;

    /* ---- 10ms: K230/GPS poll + MPU6050 + attitude update ---- */
    if (now - g_lastRun[TASK_10MS] >= g_period[TASK_10MS])
    {
        g_lastRun[TASK_10MS] += g_period[TASK_10MS];  /* use += to prevent burst on catch-up */

        processed = 0;
        while (BSP_K230_RxAvailable() && (processed < K230_RX_PROCESS_LIMIT))
        {
            ch = BSP_K230_GetRxByte();
            App_Protocol_ParseByte(ch);
            processed++;
        }

        processed = 0;
        while (BSP_GPS_RxAvailable() && (processed < GPS_RX_PROCESS_LIMIT))
        {
            ch = BSP_GPS_GetRxByte();
            App_Nav_GPS_FeedByte(ch);
            processed++;
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

    /* ---- 50ms: QMC5883L raw sample; heading awaits mounting calibration ---- */
    if (now - g_lastRun[TASK_50MS] >= g_period[TASK_50MS])
    {
        g_lastRun[TASK_50MS] += g_period[TASK_50MS];
        (void)App_Nav_QMC5883_Read(g_mag);
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

    /* ---- 1000ms: battery ADC + UART health monitor ---- */
    if (now - g_lastRun[TASK_1000MS] >= g_period[TASK_1000MS])
    {
        g_lastRun[TASK_1000MS] += g_period[TASK_1000MS];

        /* Battery ADC is disabled (PA8 was not a valid ADC pin; see bsp_adc.c).
         * No battery read here until the hardware is rewired to a real ADC pin. */

        /* K230/GPS overrun and ring-overflow watchdogs. */
        {
            static uint32_t g_lastK230Ore = 0;
            static uint32_t g_lastGPSOre = 0;
            static uint32_t g_lastK230Drop = 0;
            static uint32_t g_lastGPSDrop = 0;
            uint32_t k230Ore = BSP_K230_GetOreCount();
            uint32_t gpsOre = BSP_GPS_GetOreCount();
            uint32_t k230Drop = BSP_K230_GetRxDropCount();
            uint32_t gpsDrop = BSP_GPS_GetRxDropCount();

            if (k230Ore != g_lastK230Ore)
            {
                DBG_PRINT("[WARN] K230 ORE: %lu\n",
                          (unsigned long)(k230Ore - g_lastK230Ore));
                g_lastK230Ore = k230Ore;
            }
            if (gpsOre != g_lastGPSOre)
            {
                DBG_PRINT("[WARN] GPS ORE: %lu\n",
                          (unsigned long)(gpsOre - g_lastGPSOre));
                g_lastGPSOre = gpsOre;
            }
            if (k230Drop != g_lastK230Drop)
            {
                DBG_PRINT("[WARN] K230 RX drop: %lu\n",
                          (unsigned long)(k230Drop - g_lastK230Drop));
                g_lastK230Drop = k230Drop;
            }
            if (gpsDrop != g_lastGPSDrop)
            {
                DBG_PRINT("[WARN] GPS RX drop: %lu\n",
                          (unsigned long)(gpsDrop - g_lastGPSDrop));
                g_lastGPSDrop = gpsDrop;
            }
        }
    }
}

uint32_t App_Task_GetDelay(void)
{
    return 1;
}
