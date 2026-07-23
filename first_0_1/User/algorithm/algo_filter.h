#ifndef __ALGO_FILTER_H
#define __ALGO_FILTER_H

#include "stm32f10x.h"

#define FILTER_ALPHA    0.96f
#define GYRO_GAIN       0.060975f
#define RAD_TO_DEG      57.29578f

/*
 * Fast atan2 approximation — cubic rational, max error ~0.001 rad.
 * Static inline: zero call overhead on Cortex-M3 (no FPU),
 * single source of truth shared by filter + navigation.
 */
static inline float fast_atan2(float y, float x)
{
    float r, angle, abs_y;
    abs_y = (y < 0.0f) ? -y : y;

    if (x >= 0.0f)
    {
        r = (x - abs_y) / (x + abs_y + 0.000001f);
        angle = 0.1963f * r * r * r - 0.9817f * r + 3.14159265f / 4.0f;
    }
    else
    {
        r = (x + abs_y) / (abs_y - x + 0.000001f);
        angle = 0.1963f * r * r * r - 0.9817f * r + 3.0f * 3.14159265f / 4.0f;
    }
    return (y < 0.0f) ? -angle : angle;
}

typedef struct
{
    float pitch;
    float roll;
    float pitchAccel;
    float rollAccel;
    float gyroOffsetX;
    float gyroOffsetY;
    float gyroOffsetZ;
} Attitude_t;

void Algo_Filter_Init(void);
void Algo_Filter_Update(int16_t *accel, int16_t *gyro, float dt);
void Algo_Filter_StartCalibration(void);
void Algo_Filter_FinishCalibration(void);
float Algo_Filter_GetPitch(void);
float Algo_Filter_GetRoll(void);
const Attitude_t *Algo_Filter_GetAttitude(void);

#endif
