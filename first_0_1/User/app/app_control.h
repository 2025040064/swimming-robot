#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include "stm32f10x.h"
#include "algorithm/algo_pid.h"
#include "app_state_machine.h"   /* RobotState_t — used to decide when to clear the target */

/* K230 image center reference (640x640 frame -> center = 320,320) */
#define IMG_CENTER_X    320
#define IMG_CENTER_Y    320
//输入输出均为640（640*640）

/* Deadband: target within +/-20 pixels = straight-ahead, no PID correction */
#define IMG_DEADBAND    20  //死区 （变大减少微调抖动）

/* Motor speeds (PWM values, range 0~TB_PWM_MAX_DUTY) */
#define CRUISE_SPEED    2500  //巡航速度
#define APPROACH_SPEED  3000  //接近目标时的速度
#define TURN_SPEED      2000  //
#define COLLECT_SPEED   4000  //收集滚轮速度
#define AVOID_SPEED     2800  //原地转弯速度

/* Motor ramp rate: PWM increment per 10ms call (200/10ms = 20% per second at 7000) */
#define RAMP_STEP       200  //改小转弯更平滑但反应慢

void App_Ctrl_Init(void);
void App_Ctrl_SetTarget(int16_t x, int16_t y);
void App_Ctrl_SearchCruise(void);
uint8_t App_Ctrl_ApproachTarget(void);   /* returns 0 when the target was lost and motors are stopped */
void App_Ctrl_StartCollection(void);
void App_Ctrl_StopCollection(void);
void App_Ctrl_AvoidTurn(uint8_t direction);
void App_Ctrl_StopAll(void);
void App_Ctrl_ReturnBase(void);
void App_Ctrl_OnStateChange(RobotState_t newState);
void App_Ctrl_UpdateMotors(int16_t left, int16_t right);
uint8_t App_Ctrl_GetCruisePhase(void);

#endif
