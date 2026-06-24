#ifndef __ALGO_FILTER_H
#define __ALGO_FILTER_H

#include "stm32f10x.h"

#define FILTER_ALPHA    0.96f
#define GYRO_GAIN       0.060975f
#define RAD_TO_DEG      57.29578f

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
float Algo_Filter_GetPitch(void);
float Algo_Filter_GetRoll(void);
const Attitude_t *Algo_Filter_GetAttitude(void);

#endif
