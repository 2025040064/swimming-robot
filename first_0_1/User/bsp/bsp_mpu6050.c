#include "bsp_mpu6050.h"
#include "bsp_iic.h"

void BSP_MPU6050_Init(void)
{
    BSP_IIC_Init();
    BSP_IIC_WriteAddr(MPU6050_ADDR, MPU6050_PWR_MGMT1, 0x00);
    BSP_IIC_WriteAddr(MPU6050_ADDR, MPU6050_SMPLRT_DIV, 0x07);
    BSP_IIC_WriteAddr(MPU6050_ADDR, MPU6050_CONFIG, 0x06);
    BSP_IIC_WriteAddr(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x18);
    BSP_IIC_WriteAddr(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x18);
}

uint8_t BSP_MPU6050_ReadData(int16_t *accel, int16_t *gyro)
{
    uint8_t buf[14];
    if (BSP_IIC_ReadAddr(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buf, 14) != 0)
        return 1;

    accel[0] = ((int16_t)buf[0]  << 8) | buf[1];
    accel[1] = ((int16_t)buf[2]  << 8) | buf[3];
    accel[2] = ((int16_t)buf[4]  << 8) | buf[5];
    gyro[0]  = ((int16_t)buf[8]  << 8) | buf[9];
    gyro[1]  = ((int16_t)buf[10] << 8) | buf[11];
    gyro[2]  = ((int16_t)buf[12] << 8) | buf[13];

    return 0;
}

uint8_t BSP_MPU6050_Test(void)
{
    uint8_t buf;
    if (BSP_IIC_ReadAddr(MPU6050_ADDR, 0x75, &buf, 1) != 0)
        return 0;
    return buf;
}
