#ifndef __APP_STATE_MACHINE_H
#define __APP_STATE_MACHINE_H

#include "stm32f10x.h"

typedef enum
{
    STATE_INIT = 0,
    STATE_SEARCH,
    STATE_DETECT,
    STATE_APPROACH,
    STATE_COLLECT,
    STATE_AVOID,
    STATE_RETURN,
    STATE_COUNT
} RobotState_t;

void App_SM_Init(void);
void App_SM_Run(void);
RobotState_t App_SM_GetState(void);
const char *App_SM_GetStateName(void);

#endif
