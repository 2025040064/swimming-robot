#ifndef APP_OLED_H
#define APP_OLED_H

#include "stm32f10x.h"

void App_OLED_Init(void);
void App_OLED_ShowBootStage(uint8_t stage);
void App_OLED_SetInitResult(uint8_t result);
void App_OLED_Task(void);

#endif
