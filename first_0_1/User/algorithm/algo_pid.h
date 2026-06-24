#ifndef __ALGO_PID_H
#define __ALGO_PID_H

#include "stm32f10x.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float prevError;
    float integralMax;
    float outputMin;
    float outputMax;
} PID_t;

void Algo_PID_Init(PID_t *pid, float kp, float ki, float kd, float outMin, float outMax);
float Algo_PID_Compute(PID_t *pid, float input, float dt);
void Algo_PID_Reset(PID_t *pid);
void Algo_PID_SetSetpoint(PID_t *pid, float setpoint);

#endif
