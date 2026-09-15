#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include "stm32f10x.h"
#include "algorithm/algo_pid.h"
#include "app_state_machine.h"

/* K230 image centre for a 640 x 640 frame. */
#define IMG_CENTER_X    320
#define IMG_CENTER_Y    320

/* Pixels accepted as centred; no yaw PID correction within this range. */
#define IMG_DEADBAND    20

/* Motor speeds (PWM values, range 0 to TB_PWM_MAX_DUTY). */
#define CRUISE_SPEED    2000
#define APPROACH_SPEED  3000
#define TURN_SPEED      2000
#define AVOID_SPEED     2800

/* PWM increment per 20 ms state-machine update. */
#define RAMP_STEP       200

int16_t App_Ctrl_GetLeftPwm(void);
int16_t App_Ctrl_GetRightPwm(void);
void App_Ctrl_SafetyUpdate(uint8_t sampleValid);
uint8_t App_Ctrl_SafetyReady(void);
RobotState_t App_Ctrl_GetSafetyState(void);
void App_Ctrl_Init(void);
void App_Ctrl_SetTarget(int16_t x, int16_t y);
void App_Ctrl_SearchCruise(void);
uint8_t App_Ctrl_ApproachTarget(void);
void App_Ctrl_AvoidTurn(uint8_t direction);
void App_Ctrl_StopAll(void);
void App_Ctrl_ReturnBase(void);
void App_Ctrl_OnStateChange(RobotState_t newState);
void App_Ctrl_UpdateMotors(int16_t left, int16_t right);
uint8_t App_Ctrl_GetCruisePhase(void);

#endif
