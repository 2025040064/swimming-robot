#include "stm32f10x.h"                  // Device header
#include "algo_pid.h"

void Algo_PID_Init(PID_t *pid, float kp, float ki, float kd,
                   float integralMax, float outMin, float outMax)
{
    pid->kp             = kp;
    pid->ki             = ki;
    pid->kd             = kd;
    pid->setpoint       = 0.0f;
    pid->integral       = 0.0f;
    pid->prevError      = 0.0f;
    pid->integralMax    = integralMax;
    pid->outputMin      = outMin;
    pid->outputMax      = outMax;
    pid->prevDerivative = 0.0f;
    pid->derivAlpha     = 0.1f;  /* strong filtering — good for noisy pixel input */
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

    /* Derivative on error with 1-pole low-pass filter to suppress
     * pixel-jitter noise from K230 that would otherwise amplify
     * through the KD term at 20ms sampling rate. */
    {
        float rawDeriv = (error - pid->prevError) / dt;
        pid->prevDerivative = pid->derivAlpha * rawDeriv
                            + (1.0f - pid->derivAlpha) * pid->prevDerivative;
        derivative = pid->prevDerivative;
    }

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
    pid->prevDerivative = 0.0f;   /* clear filtered D term to avoid a stale D kick */
}

void Algo_PID_SetSetpoint(PID_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}
