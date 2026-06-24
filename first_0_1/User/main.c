/**
 * Water Surface Garbage Cleaning Robot
 * STM32F103C8T6 + K230 + TB6612 + AJ-SRP04M + MPU6050
 *
 * Clock: startup_stm32f10x_md.s -> SystemInit() -> 72MHz HSE+PLL (already done before main)
 */

#include "stm32f10x.h"
#include "system_stm32f10x.h"
#include "bsp/bsp_systick.h"
#include "bsp/bsp_led.h"
#include "bsp/bsp_usart.h"
#include "bsp/bsp_iic.h"
#include "bsp/bsp_mpu6050.h"
#include "bsp/bsp_ultrasonic.h"
#include "driver/drv_tb6612.h"
#include "algorithm/algo_filter.h"
#include "algorithm/algo_pid.h"
#include "app/app_task.h"
#include "app/app_state_machine.h"
#include "app/app_protocol.h"
#include "app/app_control.h"
#include "app/app_navigation.h"

int main(void)
{
    SystemCoreClockUpdate();
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

    while (1)
    {
        App_Task_Scheduler();
    }
}

void SysTick_Handler(void)
{
    BSP_SysTick_Handler();
}
