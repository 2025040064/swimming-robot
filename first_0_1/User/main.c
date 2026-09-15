#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "system_stm32f10x.h"
#include "bsp/bsp_board.h"
#include "bsp/bsp_systick.h"
#include "bsp/bsp_led.h"
#include "bsp/bsp_usart.h"
#include "bsp/bsp_iic.h"
#include "bsp/bsp_mpu6050.h"
#include "bsp/bsp_ultrasonic.h"
#include "bsp/bsp_debug.h"
#include "bsp/bsp_adc.h"
#include "driver/drv_tb6612.h"
#include "algorithm/algo_filter.h"
#include "algorithm/algo_pid.h"
#include "app/app_task.h"
#include "app/app_oled.h"
#include "app/app_state_machine.h"
#include "app/app_protocol.h"
#include "app/app_control.h"
#include "app/app_navigation.h"
#include "Delay.h"
#include "../tast_dianji.h"
#include "robot_config.h"

#if MOTOR_TEST_ENABLE && ROBOT_IMU_MOTOR_TEST_ENABLE
#error "Select either the fixed-speed motor test or the protected IMU test."
#endif

#define IWDG_TIMEOUT_MS  1600U
#define IWDG_PRESCALER   IWDG_Prescaler_64
#define IWDG_RELOAD_VAL  ((IWDG_TIMEOUT_MS * 625U) / 1000U)

/*
 * Standalone propulsion test. Leave disabled for normal robot operation.
 * When enabled, only the board, SysTick, LED and TB6612 are initialized;
 * both propulsion motors run at MOTOR_TEST_SPEED for MOTOR_TEST_DURATION_MS,
 * then stop. Secure the robot before enabling this test.
 */
 

static void IWDG_Init(void)
{
    DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_PRESCALER);
    IWDG_SetReload(IWDG_RELOAD_VAL);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

static void IWDG_Feed(void)
{
    IWDG_ReloadCounter();
}

/* Test complete: stay here with motors stopped. */

/* Public volatile values for Keil Watch; 255 means not checked yet.
 * Stage: 10=board init, 20=I2C init, 30=MPU init, 40=ID check,
 * 100=scheduler, 101=MPU init failed, 102=MPU ID check failed. */
volatile uint32_t g_debugBootStage = 0U;
volatile uint32_t g_debugImuInit = 255U;
volatile uint32_t g_debugImuIdStatus = 255U;
volatile uint32_t g_debugImuId = 255U;

/* Runtime snapshots after each scheduler pass; PWM values are commands,
 * not measured motor RPM. StopTilt preserves the first latched fault angle. */
volatile uint32_t g_debugState = 255U;
volatile uint32_t g_debugFilterReady = 0U;
volatile uint32_t g_debugRunTick = 0U;
volatile int32_t g_debugLeftPwm = 0;
volatile int32_t g_debugRightPwm = 0;
volatile float g_debugTilt = 0.0f;
volatile float g_debugStopTilt = 0.0f;
static uint8_t g_debugFaultSaved = 0U;

int main(void)
{
    uint8_t imuInitResult;
    uint8_t imuId = 0U;
    uint8_t imuIdStatus = 255U;
    uint32_t faultLedTick = 0U;
    g_debugBootStage = 10U;
    SystemCoreClockUpdate();
    BSP_Board_Init();
    BSP_SysTick_Init();
    BSP_LED_Init();

#if MOTOR_TEST_ENABLE
    Motor_Test_Run();
#endif


    DRV_TB6612_Init();
    BSP_MpuMotorDirectionDiagnostic();
    DRV_TB6612_SetStandby(1, 0);
    App_OLED_Init();
    App_OLED_ShowBootStage(11U);
    BSP_K230_Init(K230_DEFAULT_BAUDRATE);
    Crash_ReportAndClear();
    BSP_Ultrasonic_Init();
    g_debugBootStage = 20U;
    App_OLED_ShowBootStage(20U);
    BSP_IIC_Init();
    g_debugBootStage = 30U;
    App_OLED_ShowBootStage(30U);
    imuInitResult = BSP_MPU6050_Init();
    g_debugImuInit = imuInitResult;
    App_OLED_SetInitResult(imuInitResult);
    BSP_ADC_Init();

    Algo_Filter_Init();
    App_Ctrl_Init();
    App_Protocol_Init();
    App_SM_Init();
    App_Nav_Init();
    App_Task_Init();

    g_debugBootStage = 40U;
    if (imuInitResult == BSP_IIC_OK)
    {
        imuIdStatus = BSP_IIC_ReadAddr(MPU6050_ADDR, 0x75U, &imuId, 1U);
        g_debugImuIdStatus = imuIdStatus;
        g_debugImuId = imuId;
    }
    if (imuInitResult != BSP_IIC_OK ||
        imuIdStatus != BSP_IIC_OK || imuId != 0x68U)
    {
        g_debugBootStage = (imuInitResult != BSP_IIC_OK) ? 101U : 102U;
        if (imuInitResult == BSP_IIC_OK) App_OLED_SetInitResult(0x82U);
        App_Ctrl_StopAll();
        DRV_TB6612_SetStandby(1, 0);
        (void)BSP_K230_SendString("IMU_INIT_FAILED\n");
        while (1)
        {
            if ((uint32_t)(BSP_GetTick() - faultLedTick) >= 200U)
            {
                faultLedTick = BSP_GetTick();
                BSP_LED_Toggle();
            }
            App_OLED_Task();
            __WFI();
        }
    }

    /* Do not enable the watchdog until bounded hardware bring-up succeeds. */
    IWDG_Init();
    (void)BSP_K230_SendString("ROBOT_READY\n");

    g_debugBootStage = 100U;
    while (1)
    {
        App_Task_Scheduler();
        g_debugState = (uint32_t)App_SM_GetState();
        g_debugFilterReady = Algo_Filter_IsReady();
        g_debugRunTick = BSP_GetTick();
        g_debugLeftPwm = App_Ctrl_GetLeftPwm();
        g_debugRightPwm = App_Ctrl_GetRightPwm();
        g_debugTilt = Algo_Filter_GetTilt();
        if (!g_debugFaultSaved &&
            (App_Ctrl_GetSafetyState() == STATE_TILT_STOP ||
             App_Ctrl_GetSafetyState() == STATE_IMU_STOP))
        {
            g_debugStopTilt = g_debugTilt;
            g_debugFaultSaved = 1U;
        }
        IWDG_Feed();
        App_OLED_Task();
        __WFI();
    }
}

void SysTick_Handler(void)
{
    BSP_SysTick_Handler();
}
