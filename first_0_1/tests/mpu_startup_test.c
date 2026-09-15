/* Host sensor model: delayed wake-up, retained standby and bad read-back. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "bsp_mpu6050.h"
#include "bsp_iic.h"

static uint8_t registers[128];
static uint32_t elapsed, wakeTime, firstWake, readyDelay;
static unsigned int sampleReads, resetWrites;
static int ignoreRegister, failWriteRegister, failReadRegister, forceZero;
static int16_t modelAccel[3], modelGyro[3];

uint32_t BSP_GetTick(void) { return elapsed; }
void Delay_ms(uint32_t ms) { elapsed += ms; }
uint8_t BSP_IIC_WriteAddr(uint8_t address, uint8_t reg, uint8_t value)
{
    assert(address == 0x68U && reg < sizeof(registers));
    if (reg == failWriteRegister) return BSP_IIC_ERR_NACK;
    if (reg == ignoreRegister) return BSP_IIC_OK;
    if (reg == MPU6050_PWR_MGMT1 && value == 0x80U)
    {
        memset(registers, 0, sizeof(registers));
        registers[MPU6050_PWR_MGMT1] = 0x40U;
        resetWrites++;
        return BSP_IIC_OK;
    }
    registers[reg] = value;
    if (reg == MPU6050_PWR_MGMT1 && !(value & 0x40U))
    {
        wakeTime = elapsed;
        firstWake = elapsed;
    }
    return BSP_IIC_OK;
}
static void pack(uint8_t *out, int16_t value)
{
    out[0] = (uint8_t)((uint16_t)value >> 8);
    out[1] = (uint8_t)value;
}
uint8_t BSP_IIC_ReadAddr(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len)
{
    unsigned int i;
    assert(address == 0x68U && buf);
    if (reg == failReadRegister) return BSP_IIC_ERR_TIMEOUT;
    if (reg == MPU6050_ACCEL_XOUT_H)
    {
        assert(len == 14U);
        sampleReads++;
        memset(buf, 0, len);
        if (forceZero || (registers[MPU6050_PWR_MGMT1] & 0x40U) ||
            registers[MPU6050_PWR_MGMT2] != 0U || elapsed - wakeTime < readyDelay)
            return BSP_IIC_OK;
        for (i = 0U; i < 3U; i++)
        {
            pack(buf + i * 2U, modelAccel[i]);
            pack(buf + 8U + i * 2U, modelGyro[i]);
        }
        return BSP_IIC_OK;
    }
    assert(len == 1U && reg < sizeof(registers));
    *buf = registers[reg];
    return BSP_IIC_OK;
}

#include "../User/bsp/bsp_mpu6050.c"

static void resetModel(void)
{
    memset(registers, 0, sizeof(registers));
    registers[MPU6050_PWR_MGMT1] = 0x40U;
    registers[MPU6050_PWR_MGMT2] = 0x3FU;
    elapsed = wakeTime = firstWake = 0U;
    readyDelay = 100U;
    resetWrites = sampleReads = 0U;
    ignoreRegister = failWriteRegister = failReadRegister = -1;
    forceZero = 0;
    modelAccel[0] = 2048;
    modelAccel[1] = -100;
    modelAccel[2] = 75;
    modelGyro[0] = -32768;
    modelGyro[1] = 32767;
    modelGyro[2] = -179;
}
int main(void)
{
    int16_t accel[3], gyro[3];
    resetModel();
    assert(BSP_MPU6050_Init() == BSP_IIC_OK);
    assert(resetWrites == 1U && firstWake >= 100U && elapsed >= 200U);
    assert(registers[MPU6050_PWR_MGMT1] == 1U);
    assert(registers[MPU6050_PWR_MGMT2] == 0U);
    assert(registers[MPU6050_ACCEL_CONFIG] == 0x18U);
    assert(g_debugMpuInitStage == 9U && sampleReads == 1U);
    assert(BSP_MPU6050_ReadData(accel, gyro) == BSP_IIC_OK);
    assert(memcmp(accel, modelAccel, sizeof(accel)) == 0);
    assert(memcmp(gyro, modelGyro, sizeof(gyro)) == 0);
    assert(g_debugMpuRawAx == 2048 && g_debugMpuRawAy == -100);

    resetModel();
    readyDelay = 130U;
    assert(BSP_MPU6050_Init() == BSP_IIC_OK);
    assert(sampleReads == 4U && g_debugMpuZeroSamples == 3U);

    resetModel();
    forceZero = 1;
    assert(BSP_MPU6050_Init() == MPU6050_ERR_NO_DATA);
    assert(sampleReads == 20U && elapsed == 400U);
    assert(g_debugMpuInitStage == 8U && g_debugMpuZeroSamples == 20U);

    resetModel();
    ignoreRegister = MPU6050_ACCEL_CONFIG;
    assert(BSP_MPU6050_Init() == MPU6050_ERR_VERIFY);
    assert(g_debugMpuCheckReg == 0x1CU && g_debugMpuExpected == 0x18U);
    assert(g_debugMpuActual == 0U && sampleReads == 0U);

    resetModel();
    failWriteRegister = MPU6050_PWR_MGMT2;
    assert(BSP_MPU6050_Init() == BSP_IIC_ERR_NACK);
    assert(g_debugMpuInitStage == 3U && sampleReads == 0U);

    resetModel();
    failReadRegister = MPU6050_CONFIG;
    assert(BSP_MPU6050_Init() == BSP_IIC_ERR_TIMEOUT);
    assert(g_debugMpuActual == 255U && sampleReads == 0U);

    resetModel();
    failReadRegister = MPU6050_ACCEL_XOUT_H;
    assert(BSP_MPU6050_Init() == BSP_IIC_ERR_TIMEOUT);
    assert(g_debugMpuInitStage == 8U);
    puts("PASS: reset/wake/axes, delayed data, persistent zeros, configuration mismatch, bus failures, signed burst decoding");
    return 0;
}
