#include "stm32f10x.h"
#include "system_stm32f10x.h"
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

#define IWDG_TIMEOUT_MS     1600    /* 1.6s — must be fed before this expires */
#define IWDG_PRESCALER      IWDG_Prescaler_64   /* 40kHz/64 = 625 Hz */
#define IWDG_RELOAD_VAL     ((IWDG_TIMEOUT_MS * 625U) / 1000U)  /* ~1000 */

static void IWDG_Init(void)
{
    DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);

    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);//看门狗
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

    IWDG_Init();

    BSP_SysTick_Init();
    BSP_LED_Init();
    BSP_USART_Init(115200);
    BSP_Ultrasonic_Init();
    BSP_MPU6050_Init();
    BSP_ADC_Init();
    DRV_TB6612_Init();

    Algo_Filter_Init();
    App_Ctrl_Init();
    App_Protocol_Init();
    App_SM_Init();
    App_Nav_Init();
    App_Task_Init();

    /* ---- WHO_AM_I check ---- */
    {
        uint8_t whoami = BSP_MPU6050_Test();
        if (whoami != 0x68)
        {
            /* MPU6050 not responding — fast-blink forever */
            while (1)
            {
                BSP_LED_Toggle();
                Delay_ms(200);
            }
        }
    }

    BSP_USART_SendString("ROBOT_READY\n");
    DBG_PRINT("Boot OK, IWDG=%ums, MPU6050=0x%02X\n", IWDG_TIMEOUT_MS,
              BSP_MPU6050_Test());

    while (1)
    {
        App_Task_Scheduler();
        IWDG_Feed();
        __WFI();  /* sleep until next interrupt, saves battery */
    }
}

void SysTick_Handler(void)
{
    BSP_SysTick_Handler();
}
