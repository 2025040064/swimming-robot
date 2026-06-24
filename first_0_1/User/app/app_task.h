#ifndef __APP_TASK_H
#define __APP_TASK_H

#include "stm32f10x.h"

typedef enum
{
    TASK_10MS  = 0,
    TASK_20MS,
    TASK_50MS,
    TASK_100MS,
    TASK_200MS,
    TASK_500MS,
    TASK_1000MS,
    TASK_COUNT
} TaskSlot_t;

void App_Task_Init(void);
void App_Task_Scheduler(void);

/* Called in main loop, returns ms until next task */
uint32_t App_Task_GetDelay(void);

#endif
