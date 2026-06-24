#include "algo_pid.h"

void Algo_PID_Init(PID_t *pid, float kp, float ki, float kd, float outMin, float outMax)
{
    pid->kp          = kp;
    pid->ki          = ki;
    pid->kd          = kd;
    pid->setpoint    = 0.0f;
    pid->integral    = 0.0f;
    pid->prevError   = 0.0f;
    pid->integralMax = 1000.0f;
    pid->outputMin   = outMin;
    pid->outputMax   = outMax;
}

float Algo_PID_Compute(PID_t *pid, float input, float dt)
{
    float error, derivative, output;

    if (dt < 0.001f) dt = 0.001f;

    error = pid->setpoint - input;

    pid->integral += error * dt;
    if (pid->integral > pid->integralMax)
        pid->integral = pid->integralMax;
    else if (pid->integral < -pid->integralMax)
        pid->integral = -pid->integralMax;

    derivative = (error - pid->prevError) / dt;

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    if (output > pid->outputMax)       output = pid->outputMax;
    else if (output < pid->outputMin)  output = pid->outputMin;

    pid->prevError = error;
    return output;
}

void Algo_PID_Reset(PID_t *pid)
{
    pid->integral  = 0.0f;
    pid->prevError = 0.0f;
}

void Algo_PID_SetSetpoint(PID_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}
