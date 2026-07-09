/**
 * Water Surface Garbage Cleaning Robot
 * STM32F103C8T6 + K230 + TB6612 + AJ-SRP04M + MPU6050
 *
 * Clock: startup_stm32f10x_md.s -> SystemInit() -> 72MHz HSE+PLL (already done before main)
 *
 * Watchdog: IWDG with 1.6s timeout. Fed in main loop. If any module hangs,
 * the system auto-resets. Debug: DBGMCU_IWDG_STOP freezes IWDG at breakpoints.
 */

#include "stm32f10x.h"
#include "system_stm32f10x.h"
#include "bsp/bsp_systick.h"
#include "bsp/bsp_led.h"
#include "bsp/bsp_usart.h"
#include "bsp/bsp_iic.h"
#include "bsp/bsp_mpu6050.h"
#include "bsp/bsp_ultrasonic.h"
#include "bsp/bsp_debug.h"
#include "driver/drv_tb6612.h"
#include "algorithm/algo_filter.h"
#include "algorithm/algo_pid.h"
#include "app/app_task.h"
#include "app/app_state_machine.h"
#include "app/app_protocol.h"
#include "app/app_control.h"
#include "app/app_navigation.h"

/* ---- IWDG configuration ---- */
#define IWDG_TIMEOUT_MS     1600    /* 1.6s — must be fed before this expires */
#define IWDG_PRESCALER      IWDG_Prescaler_64   /* 40kHz/64 = 625 Hz */
#define IWDG_RELOAD_VAL     ((IWDG_TIMEOUT_MS * 625U) / 1000U)  /* ~1000 */

static void IWDG_Init(void)
{
    /* Freeze IWDG during debug halt (prevents unwanted resets at breakpoints) */
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

    /* Watchdog first — protects all subsequent init */
    IWDG_Init();

    BSP_SysTick_Init();
    BSP_LED_Init();
    BSP_USART_Init(115200);
    BSP_Ultrasonic_Init();
    BSP_MPU6050_Init();
    DRV_TB6612_Init();

    Algo_Filter_Init();
    App_Ctrl_Init();
    App_Protocol_Init();
    App_SM_Init();
    App_Nav_Init();
    App_Task_Init();

    BSP_USART_SendString("ROBOT_READY\n");
    DBG_PRINT("Boot OK, IWDG=%ums\n", IWDG_TIMEOUT_MS);

    while (1)
    {
        App_Task_Scheduler();
        IWDG_Feed();
    }
}

void SysTick_Handler(void)
{
    BSP_SysTick_Handler();
}
