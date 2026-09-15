#ifndef __BSP_MPU6050_H
#define __BSP_MPU6050_H

#include "stm32f10x.h"

#define MPU6050_ADDR    0x68

#define MPU6050_PWR_MGMT1   0x6B
#define MPU6050_PWR_MGMT2   0x6C

/* Device-level errors; I2C transport errors retain their BSP_IIC values. */
#define MPU6050_ERR_VERIFY  0x80U
#define MPU6050_ERR_NO_DATA 0x81U
#define MPU6050_SMPLRT_DIV  0x19
#define MPU6050_CONFIG      0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48

/* BSP_IIC_Init() must be called once before this shared-bus device is used. */
typedef struct
{
    int16_t accel[3];
    int16_t gyro[3];
    uint32_t sampleTick;
    uint8_t readStatus;
    uint8_t hasSample;
} MPU6050_Data_t;

const MPU6050_Data_t *BSP_MPU6050_GetLatest(void);
uint8_t BSP_MPU6050_Init(void);
uint8_t BSP_MPU6050_ReadData(int16_t *accel, int16_t *gyro);
uint8_t BSP_MPU6050_Test(void);

#endif
