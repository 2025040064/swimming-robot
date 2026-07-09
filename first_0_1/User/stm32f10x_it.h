#ifndef __STM32F10x_IT_H
#define __STM32F10x_IT_H

#include "stm32f10x.h"

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

/* C-level entry points to read stacked register frame for crash dump */
void HardFault_Handler_C(uint32_t *stack);
void MemManage_Handler_C(uint32_t *stack);
void BusFault_Handler_C(uint32_t *stack);
void UsageFault_Handler_C(uint32_t *stack);

#endif
