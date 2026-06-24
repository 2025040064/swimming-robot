#ifndef __BSP_SYSTICK_H
#define __BSP_SYSTICK_H

#include "stm32f10x.h"

void BSP_SysTick_Init(void);
void BSP_DelayMs(uint32_t ms);
uint32_t BSP_GetTick(void);
void BSP_SysTick_Handler(void);

#endif
