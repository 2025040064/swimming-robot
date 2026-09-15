#include "bsp_mpu6050.h"
#include "bsp_iic.h"
#include "Delay.h"
#include "bsp_systick.h"
#include <string.h>

/* Keil diagnostics: last checked register and its read-back value.
 * Init stage: 1=reset, 2=wake, 3=axes, 4=rate, 5=filter, 6=gyro range,
 * 7=accel range, 8=wait for nonzero acceleration, 9=ready. */
volatile uint32_t g_debugMpuInitStage;
volatile uint32_t g_debugMpuCheckReg;
volatile uint32_t g_debugMpuExpected;
volatile uint32_t g_debugMpuActual;
volatile int32_t g_debugMpuRawAx;
volatile int32_t g_debugMpuRawAy;
volatile int32_t g_debugMpuRawAz;
volatile uint32_t g_debugMpuZeroSamples;
static MPU6050_Data_t g_latest;

const MPU6050_Data_t *BSP_MPU6050_GetLatest(void) { return &g_latest; }

static uint8_t MPU_WriteChecked(uint8_t reg, uint8_t value)
{
    uint8_t actual = 0U;
    uint8_t result;
    g_debugMpuCheckReg = reg;
    g_debugMpuExpected = value;
    g_debugMpuActual = 255U;
    result = BSP_IIC_WriteAddr(MPU6050_ADDR, reg, value);
    if (result != BSP_IIC_OK) return result;
    result = BSP_IIC_ReadAddr(MPU6050_ADDR, reg, &actual, 1U);
    if (result != BSP_IIC_OK) return result;
    g_debugMpuActual = actual;
    return (actual == value) ? BSP_IIC_OK : MPU6050_ERR_VERIFY;
}

uint8_t BSP_MPU6050_Init(void)
{
    uint8_t result;
    uint8_t attempt;
    int16_t accel[3];
    int16_t gyro[3];

    memset(&g_latest, 0, sizeof(g_latest));
    g_latest.readStatus = 255U;
    g_debugMpuInitStage = 1U;
    g_debugMpuCheckReg = MPU6050_PWR_MGMT1;
    g_debugMpuExpected = 0x80U;
    g_debugMpuActual = 255U;
    g_debugMpuRawAx = g_debugMpuRawAy = g_debugMpuRawAz = 0;
    g_debugMpuZeroSamples = 0U;
    /* Reset the sensor too, so an MCU-only reset cannot retain axis standby. */
    result = BSP_IIC_WriteAddr(MPU6050_ADDR, MPU6050_PWR_MGMT1, 0x80U);
    if (result != BSP_IIC_OK) return result;
    Delay_ms(100U);

    g_debugMpuInitStage = 2U;
    result = MPU_WriteChecked(MPU6050_PWR_MGMT1, 0x01U);
    if (result != BSP_IIC_OK) return result;
    g_debugMpuInitStage = 3U;
    result = MPU_WriteChecked(MPU6050_PWR_MGMT2, 0x00U);
    if (result != BSP_IIC_OK) return result;
    g_debugMpuInitStage = 4U;
    result = MPU_WriteChecked(MPU6050_SMPLRT_DIV, 0x07U);
    if (result != BSP_IIC_OK) return result;
    g_debugMpuInitStage = 5U;
    result = MPU_WriteChecked(MPU6050_CONFIG, 0x06U);
    if (result != BSP_IIC_OK) return result;
    g_debugMpuInitStage = 6U;
    result = MPU_WriteChecked(MPU6050_GYRO_CONFIG, 0x18U);
    if (result != BSP_IIC_OK) return result;
    g_debugMpuInitStage = 7U;
    result = MPU_WriteChecked(MPU6050_ACCEL_CONFIG, 0x18U);
    if (result != BSP_IIC_OK) return result;
    Delay_ms(100U);

    /* This only screens startup zeros. Normal sample validity, calibration
     * and tilt protection remain mandatory in the application. */
    g_debugMpuInitStage = 8U;
    for (attempt = 0U; attempt < 20U; attempt++)
    {
        result = BSP_MPU6050_ReadData(accel, gyro);
        if (result != BSP_IIC_OK) return result;
        if (accel[0] != 0 || accel[1] != 0 || accel[2] != 0)
        {
            g_debugMpuInitStage = 9U;
            return BSP_IIC_OK;
        }
        Delay_ms(10U);
    }
    return MPU6050_ERR_NO_DATA;
}

uint8_t BSP_MPU6050_ReadData(int16_t *accel, int16_t *gyro)
{
    uint8_t buf[14];
    uint8_t result = BSP_IIC_ReadAddr(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buf, 14);
    g_latest.readStatus = result;
    if (result != BSP_IIC_OK) return result;

    accel[0] = ((int16_t)buf[0]  << 8) | buf[1];
    accel[1] = ((int16_t)buf[2]  << 8) | buf[3];
    accel[2] = ((int16_t)buf[4]  << 8) | buf[5];
    gyro[0]  = ((int16_t)buf[8]  << 8) | buf[9];
    gyro[1]  = ((int16_t)buf[10] << 8) | buf[11];
    gyro[2]  = ((int16_t)buf[12] << 8) | buf[13];

    memcpy(g_latest.accel, accel, sizeof(g_latest.accel));
    memcpy(g_latest.gyro, gyro, sizeof(g_latest.gyro));
    g_latest.sampleTick = BSP_GetTick();
    g_latest.hasSample = 1U;
    g_debugMpuRawAx = accel[0];
    g_debugMpuRawAy = accel[1];
    g_debugMpuRawAz = accel[2];
    if (accel[0] == 0 && accel[1] == 0 && accel[2] == 0)
        g_debugMpuZeroSamples++;

    return 0;
}

uint8_t BSP_MPU6050_Test(void)
{
    uint8_t buf;
    if (BSP_IIC_ReadAddr(MPU6050_ADDR, 0x75, &buf, 1) != 0)
        return 0;
    return buf;
}
