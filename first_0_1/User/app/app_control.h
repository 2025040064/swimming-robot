#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include "stm32f10x.h"
#include "algorithm/algo_pid.h"

#define IMG_CENTER_X    0
#define IMG_CENTER_Y    0
#define IMG_DEADBAND    20

#define CRUISE_SPEED    2500
#define APPROACH_SPEED  3000
#define TURN_SPEED      2000
#define COLLECT_SPEED   4000
#define AVOID_SPEED     2800

void App_Ctrl_Init(void);
void App_Ctrl_SetTarget(int16_t x, int16_t y);
void App_Ctrl_SearchCruise(void);
void App_Ctrl_ApproachTarget(void);
void App_Ctrl_StartCollection(void);
void App_Ctrl_StopCollection(void);
void App_Ctrl_AvoidTurn(uint8_t direction);
void App_Ctrl_StopAll(void);
void App_Ctrl_ReturnBase(void);
void App_Ctrl_OnStateChange(uint8_t newState);
void App_Ctrl_UpdateMotors(int16_t left, int16_t right);

#endif
