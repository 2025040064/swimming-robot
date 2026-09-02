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
#include "app/app_state_machine.h"
#include "app/app_protocol.h"
#include "app/app_control.h"
#include "app/app_navigation.h"
#include "Delay.h"

#define IWDG_TIMEOUT_MS  1600U
#define IWDG_PRESCALER   IWDG_Prescaler_64
#define IWDG_RELOAD_VAL  ((IWDG_TIMEOUT_MS * 625U) / 1000U)

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

int main(void)
{
    SystemCoreClockUpdate();
    BSP_Board_Init();
    BSP_SysTick_Init();
    BSP_LED_Init();
    BSP_K230_Init(K230_DEFAULT_BAUDRATE);
    Crash_ReportAndClear();
    BSP_Ultrasonic_Init();
    BSP_IIC_Init();
    (void)BSP_MPU6050_Init();
    BSP_ADC_Init();
    DRV_TB6612_Init();

    Algo_Filter_Init();
    App_Ctrl_Init();
    App_Protocol_Init();
    App_SM_Init();
    App_Nav_Init();
    App_Task_Init();

    if (BSP_MPU6050_Test() != 0x68U)
    {
        while (1)
        {
            BSP_LED_Toggle();
            Delay_ms(200U);
        }
    }

    /* Do not enable the watchdog until bounded hardware bring-up succeeds. */
    IWDG_Init();
    (void)BSP_K230_SendString("ROBOT_READY\n");

    while (1)
    {
        App_Task_Scheduler();
        IWDG_Feed();
        __WFI();
    }
}

void SysTick_Handler(void)
{
    BSP_SysTick_Handler();
}
