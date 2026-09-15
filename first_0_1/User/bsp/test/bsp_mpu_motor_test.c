#include "robot_config.h"
#include "bsp_mpu_motor_test.h"
#include "app/app_control.h"
#include "app/app_state_machine.h"
#include "algorithm/algo_filter.h"
#include "driver/drv_tb6612.h"
#include "bsp_systick.h"
#include "bsp_usart.h"
#include <string.h>

/* Bench-test sequencing reuses the production IMU protection and PWM path. */
#if ROBOT_IMU_MOTOR_TEST_ENABLE
static uint8_t g_testStarted;
static uint8_t g_testFinished;
static uint32_t g_testStartTick;
#if ROBOT_IMU_TEST_PWM <= 0 || ROBOT_IMU_TEST_PWM > TB_PWM_MAX_DUTY
#error "IMU test PWM must be within the positive timer duty range."
#endif
#endif

void BSP_MpuMotorDirectionDiagnostic(void)
{
#if ROBOT_IMU_MOTOR_TEST_ENABLE && ROBOT_MOTOR_DIRECTION_DIAG_ENABLE
    DRV_TB6612_RunDirectionDiagnostic(ROBOT_MOTOR_DIRECTION_DIAG_PWM);
#endif
}

void BSP_MpuMotorTest_Init(void)
{
#if ROBOT_IMU_MOTOR_TEST_ENABLE
    g_testStarted = 0U;
    g_testFinished = 0U;
    g_testStartTick = 0U;
#endif
}

uint8_t BSP_MpuMotorTest_IsFinished(void)
{
#if ROBOT_IMU_MOTOR_TEST_ENABLE
    return g_testFinished;
#else
    return 0U;
#endif
}

void BSP_MpuMotorTest_Run(void)
{
#if ROBOT_IMU_MOTOR_TEST_ENABLE
    uint32_t now = BSP_GetTick();

    if (g_testStarted &&
        (uint32_t)(now - g_testStartTick) >= ROBOT_IMU_TEST_DURATION_MS)
        g_testFinished = 1U;

    if (g_testFinished)
    {
        App_Ctrl_StopAll();
        DRV_TB6612_SetStandby(1, 0);
        if (App_SM_GetState() != STATE_TEST_DONE)
            App_SM_SetState(STATE_TEST_DONE, 0U);
        return;
    }
    if (!App_Ctrl_SafetyReady())
    {
        App_Ctrl_StopAll();
        return;
    }
    if (!g_testStarted)
    {
        g_testStarted = 1U;
        g_testStartTick = now;
        App_SM_SetState(STATE_IMU_TEST, 0U);
    }
    App_Ctrl_UpdateMotors(ROBOT_IMU_TEST_PWM, ROBOT_IMU_TEST_PWM);
#endif
}

static uint8_t AppendTenths(char *buf, uint8_t idx, int32_t value)
{
    uint32_t whole;
    if (value < 0) { buf[idx++] = '-'; value = -value; }
    whole = (uint32_t)value / 10U;
    if (whole >= 100U) buf[idx++] = (char)('0' + whole / 100U);
    if (whole >= 10U) buf[idx++] = (char)('0' + (whole / 10U) % 10U);
    buf[idx++] = (char)('0' + whole % 10U);
    buf[idx++] = '.';
    buf[idx++] = (char)('0' + (uint32_t)value % 10U);
    return idx;
}

static uint8_t AppendPwm(char *buf, uint8_t idx, int16_t value)
{
    uint16_t number;
    if (value < 0) { buf[idx++] = '-'; value = -value; }
    number = (uint16_t)value;
    if (number >= 1000U) buf[idx++] = (char)('0' + number / 1000U);
    if (number >= 100U) buf[idx++] = (char)('0' + (number / 100U) % 10U);
    if (number >= 10U) buf[idx++] = (char)('0' + (number / 10U) % 10U);
    buf[idx++] = (char)('0' + number % 10U);
    return idx;
}

void BSP_MpuMotorTest_SendStatus(float tilt, int16_t left, int16_t right)
{
    char buf[40];
    uint8_t idx = 8U;
    memcpy(buf, "MPUTEST,", 8U);
    idx = AppendTenths(buf, idx, (int32_t)(tilt * 10.0f));
    buf[idx++] = ',';
    idx = AppendPwm(buf, idx, left);
    buf[idx++] = ',';
    idx = AppendPwm(buf, idx, right);
    buf[idx++] = '\n';
    buf[idx] = '\0';
    BSP_K230_SendString(buf);
}

void BSP_MpuMotorTest_Report(void)
{
#if ROBOT_IMU_MOTOR_TEST_ENABLE
    BSP_MpuMotorTest_SendStatus(Algo_Filter_GetTilt(),
                               App_Ctrl_GetLeftPwm(), App_Ctrl_GetRightPwm());
#endif
}
