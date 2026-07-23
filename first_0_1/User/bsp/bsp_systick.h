#ifndef __BSP_SYSTICK_H
#define __BSP_SYSTICK_H

#include "stm32f10x.h"

void BSP_SysTick_Init(void);
uint32_t BSP_GetTick(void);
void BSP_SysTick_Handler(void);

#endif
