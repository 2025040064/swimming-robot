#include "tast_dianji.h"
#include "bsp/bsp_systick.h"
#include "bsp/bsp_led.h"
#include "driver/drv_tb6612.h"
void Motor_Test_Run(void)
{
    uint32_t startTick;

    DRV_TB6612_Init();
    BSP_LED_Off();
    startTick = BSP_GetTick();

    DRV_TB6612_SetSpeed(MOTOR_LEFT, MOTOR_TEST_SPEED);
    DRV_TB6612_SetSpeed(MOTOR_RIGHT, MOTOR_TEST_SPEED);

    while ((BSP_GetTick() - startTick) < MOTOR_TEST_DURATION_MS)
    {
        __WFI();
    }

    DRV_TB6612_StopAll();
    BSP_LED_On();  
    while (1)
    {
        __WFI();
    }
}
