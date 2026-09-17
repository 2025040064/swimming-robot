#include "app_task.h"
#include "robot_config.h"
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

/* Read failures since task init; separate from filter sample rejection. */
volatile uint32_t g_debugImuReadFailures;
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

    g_debugImuReadFailures = 0U;
    for (i = 0U; i < TASK_COUNT; i++)
        g_lastRun[i] = now;
    g_lastAttiUpdate = now;
}

void App_Task_Scheduler(void)
{
    uint32_t now = BSP_GetTick();
    uint8_t ch;
    uint16_t processed;
    uint8_t sampleValid;
    int16_t mappedAccel[3];
    int16_t mappedGyro[3];

    App_Ctrl_SafetyUpdate(0U);

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

        sampleValid = 0U;
        if (BSP_MPU6050_ReadData(g_accel, g_gyro) == BSP_IIC_OK)
        {
            float dt = (now - g_lastAttiUpdate) * 0.001f;
            if (dt > 0.1f) dt = 0.1f;
            g_lastAttiUpdate = now;
#if ROBOT_IMU_X_UP
            /* Right-handed basis: mapped X=sensor Y (bow),
             * mapped Y=sensor Z, mapped Z=sensor X (up).
             * The legacy filter names pitch about mapped X and roll about Y. */
            mappedAccel[0] = g_accel[1];
            mappedAccel[1] = g_accel[2];
            mappedAccel[2] = g_accel[0];
            mappedGyro[0] = g_gyro[1];
            mappedGyro[1] = g_gyro[2];
            mappedGyro[2] = g_gyro[0];
#else
            mappedAccel[0] = g_accel[0];
            mappedAccel[1] = g_accel[1];
            mappedAccel[2] = g_accel[2];
            mappedGyro[0] = g_gyro[0];
            mappedGyro[1] = g_gyro[1];
            mappedGyro[2] = g_gyro[2];
#endif
            Algo_Filter_Update(mappedAccel, mappedGyro, dt);
            sampleValid = Algo_Filter_SampleValid();
        }
        else
            g_debugImuReadFailures++;
        if (!sampleValid && App_SM_GetState() == STATE_INIT)
            Algo_Filter_StartCalibration();
        App_Ctrl_SafetyUpdate(sampleValid);
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
                                Algo_Filter_GetRoll(),
                                Algo_Filter_GetPitch(),
                                BSP_Ultrasonic_GetFront(),
                                BSP_Ultrasonic_GetLeft(),
                                BSP_Ultrasonic_GetRight());
        BSP_MpuMotorTest_Report();
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
            DBG_PRINT("[WARN] K230 ORE: ");
            DBG_U32(ore - lastOre);
            DBG_PRINT("\r\n");
            lastOre = ore;
        }
    }
}

uint32_t App_Task_GetDelay(void)
{
    return 1U;
}
