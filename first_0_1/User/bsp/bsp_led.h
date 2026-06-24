#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "stm32f10x.h"

#define LED_PORT    GPIOC
#define LED_PIN     GPIO_Pin_13
#define LED_CLK     RCC_APB2Periph_GPIOC

void BSP_LED_Init(void);
void BSP_LED_On(void);
void BSP_LED_Off(void);
void BSP_LED_Toggle(void);

#endif
