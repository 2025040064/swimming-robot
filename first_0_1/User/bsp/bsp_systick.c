#include "bsp_systick.h"
#include "system_stm32f10x.h"

static volatile uint32_t g_sysTick = 0;

void BSP_SysTick_Init(void)
{
    SysTick_Config(SystemCoreClock / 1000);
}

uint32_t BSP_GetTick(void)
{
    return g_sysTick;
}

void BSP_SysTick_Handler(void)
{
    g_sysTick++;
}
