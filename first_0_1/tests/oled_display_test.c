#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "bsp_mpu6050.h"
#include "bsp/oled/bsp_oled.h"
#include "app/app_oled.h"

static uint32_t tick, sendCalls, initCalls;
static uint8_t attached = 1U, present, chunks, ready, valid;
static uint16_t count;
static MPU6050_Data_t data;
static char rows[8][22];
static const char *state = "INIT";

uint32_t BSP_GetTick(void) { return tick; }
void Delay_ms(uint32_t ms) { tick += ms; }
const MPU6050_Data_t *BSP_MPU6050_GetLatest(void) { return &data; }
uint8_t Algo_Filter_IsReady(void) { return ready; }
uint8_t Algo_Filter_SampleValid(void) { return valid; }
uint16_t Algo_Filter_GetCalibCount(void) { return count; }
float Algo_Filter_GetRoll(void) { return 1.5f; }
float Algo_Filter_GetPitch(void) { return -2.5f; }
float Algo_Filter_GetTilt(void) { return 3.0f; }
const char *App_SM_GetStateName(void) { return state; }
int16_t App_Ctrl_GetLeftPwm(void) { return ready ? 3600 : 0; }
int16_t App_Ctrl_GetRightPwm(void) { return ready ? 3600 : 0; }
uint8_t DRV_TB6612_GetDirectionOutput(void) { return 5U; }
uint8_t DRV_TB6612_GetDirectionInput(void) { return 5U; }
uint8_t BSP_OLED_Init(void) { initCalls++; chunks = 0U; present = attached; return present; }
uint8_t BSP_OLED_IsPresent(void) { return present; }
uint8_t BSP_OLED_IsBusy(void) { return chunks != 0U; }
void BSP_OLED_Clear(void) { assert(!chunks); memset(rows, 0, sizeof(rows)); }
void BSP_OLED_Text(uint8_t row, const char *text)
{
    assert(!chunks && row < 8U && strlen(text) <= 21U);
    strcpy(rows[row], text);
}
void BSP_OLED_Present(void) { assert(!chunks); chunks = 64U; }
void BSP_OLED_Task(void) { sendCalls++; if (chunks) chunks--; }
static void finishFrame(void)
{
    unsigned int i;
    for (i = 0U; i < 64U; i++) { tick++; App_OLED_Task(); }
    assert(!chunks);
}
static void nextFrame(void) { tick += 250U; App_OLED_Task(); }

int main(void)
{
    uint32_t calls;
    App_OLED_Init();
    assert(strcmp(rows[0], "MPU STARTING") == 0);
    assert(sendCalls == 64U);
    data.hasSample = 1U;
    data.readStatus = 0U;
    data.sampleTick = tick;
    data.accel[0] = -32768; data.accel[1] = 32767; data.accel[2] = 2000;
    data.gyro[0] = -179; data.gyro[1] = 17; data.gyro[2] = 257;
    valid = 1U; count = 19U;
    App_OLED_SetInitResult(0U);
    App_OLED_Task();
    assert(strcmp(rows[0], "MPU6050 LIVE") == 0);
    assert(strcmp(rows[1], "AX:-32768 AY:32767") == 0);
    assert(strcmp(rows[3], "GY:17 GZ:257") == 0);
    assert(strcmp(rows[4], "P:---- R:----") == 0);
    assert(strstr(rows[5], "CAL:19"));
    calls = sendCalls;
    App_OLED_Task();
    assert(sendCalls == calls);
    finishFrame();

    ready = 1U; count = 200U; state = "IMU_TEST";
    data.sampleTick = tick + 250U;
    nextFrame();
    assert(strcmp(rows[4], "P:+1.5 R:-2.5") == 0);
    assert(strstr(rows[5], "CAL:OK"));
    assert(strcmp(rows[6], "STATE:IMU_TEST") == 0);
    assert(strcmp(rows[7], "L3600 R3600 D5/5") == 0);
    finishFrame();

    nextFrame();
    assert(strcmp(rows[0], "MPU DATA STALE") == 0);
    assert(strcmp(rows[1], "AX:---- AY:----") == 0);
    assert(strcmp(rows[4], "P:---- R:----") == 0);
    finishFrame();

    data.readStatus = 5U;
    nextFrame();
    assert(strcmp(rows[0], "MPU READ ERR:05") == 0);
    finishFrame();
    data.readStatus = 0U;
    memset(data.accel, 0, sizeof(data.accel));
    data.sampleTick = tick + 250U;
    nextFrame();
    assert(strcmp(rows[0], "MPU ZERO DATA") == 0);
    finishFrame();
    App_OLED_SetInitResult(0x81U);
    nextFrame();
    assert(strcmp(rows[0], "MPU INIT ERR:81") == 0);
    finishFrame();

    attached = present = 0U;
    calls = initCalls;
    tick += 1000U;
    App_OLED_Task();
    assert(initCalls == calls + 1U);
    calls = initCalls;
    tick += 500U;
    App_OLED_Task();
    assert(initCalls == calls);
    attached = 1U;
    tick += 501U;
    App_OLED_Task();
    assert(present);
    tick++;
    App_OLED_Task();
    assert(chunks == 63U);
    finishFrame();

    tick = 0xFFFFFFF0U;
    App_OLED_SetInitResult(0U);
    data.sampleTick = tick;
    App_OLED_Task();
    finishFrame();
    assert(tick < 100U);
    puts("PASS: OLED raw data, calibration/angles, fault/stale labels, row bounds, incremental refresh, disconnect/reconnect and clock wrap");
    return 0;
}
