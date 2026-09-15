#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "robot_config.h"
#include "algorithm/algo_filter.h"
#include "app/app_control.h"
#include "app/app_protocol.h"
#include "app/app_task.h"
#include "bsp/bsp_ultrasonic.h"

static uint32_t tick;
static int16_t rawAccel[3];
static int16_t rawGyro[3];
static uint8_t imuFail;
static int16_t motorSpeed[4];
static uint8_t standby[3];
static float distanceCm[3];
static uint8_t distanceValid[3];
static char tx[256];

uint32_t BSP_GetTick(void) { return tick; }
void BSP_LED_On(void) { }
void BSP_LED_Off(void) { }
void BSP_LED_Toggle(void) { }
void BSP_Ultrasonic_Update(void) { }
uint8_t BSP_Ultrasonic_IsValid(uint8_t sensor) { return distanceValid[sensor]; }
float BSP_Ultrasonic_GetFront(void) { return distanceCm[0]; }
float BSP_Ultrasonic_GetLeft(void) { return distanceCm[1]; }
float BSP_Ultrasonic_GetRight(void) { return distanceCm[2]; }
uint8_t BSP_K230_RxAvailable(void) { return 0U; }
uint8_t BSP_K230_GetRxByte(void) { return 0U; }
uint32_t BSP_K230_GetOreCount(void) { return 0U; }
uint8_t BSP_K230_SendString(const char *s)
{
    assert(strlen(s) < sizeof(tx));
    strcpy(tx, s);
    return 0U;
}
uint8_t BSP_MPU6050_ReadData(int16_t *accel, int16_t *gyro)
{
    memcpy(accel, rawAccel, sizeof(rawAccel));
    memcpy(gyro, rawGyro, sizeof(rawGyro));
    return imuFail;
}
void DRV_TB6612_SetStandby(uint8_t bridge, uint8_t enabled)
{
    standby[bridge] = enabled;
}
void DRV_TB6612_SetSpeed(uint8_t motor, int16_t speed)
{
    motorSpeed[motor] = speed;
}
void DRV_TB6612_StopAll(void) { memset(motorSpeed, 0, sizeof(motorSpeed)); }
void DRV_TB6612_RunDirectionDiagnostic(int16_t speed) { (void)speed; }

static void runMs(uint32_t ms)
{
    uint32_t i;
    for (i = 0U; i < ms; i++)
    {
        tick++;
        App_Task_Scheduler();
    }
}

static void resetAt(uint32_t start)
{
    uint8_t i;
    tick = start;
    imuFail = 0U;
    memset(rawAccel, 0, sizeof(rawAccel));
    memset(rawGyro, 0, sizeof(rawGyro));
    memset(motorSpeed, 0, sizeof(motorSpeed));
    memset(standby, 0, sizeof(standby));
    rawAccel[0] = 2048; /* User installation: sensor X points upward. */
    for (i = 0U; i < 3U; i++)
    {
        distanceCm[i] = 200.0f;
        distanceValid[i] = 1U;
    }
    Algo_Filter_Init();
    App_Ctrl_Init();
    App_Protocol_Init();
    App_SM_Init();
    App_Task_Init();
}

static void tilt(float degrees, uint8_t horizontalAxis)
{
    double radians = degrees * 3.141592653589793 / 180.0;
    memset(rawAccel, 0, sizeof(rawAccel));
    rawAccel[0] = (int16_t)(2048.0 * cos(radians));
    rawAccel[horizontalAxis] = (int16_t)(2048.0 * sin(radians));
}

static void packet(const char *s)
{
    while (*s) App_Protocol_ParseByte((uint8_t)*s++);
}

static void stopped(void)
{
    assert(motorSpeed[0] == 0 && motorSpeed[1] == 0);
    assert(motorSpeed[2] == 0 && motorSpeed[3] == 0);
}

static void testBootAndMapping(void)
{
    resetAt(0U);
    runMs(2990U);
    stopped();
    assert(!App_Ctrl_SafetyReady());
    runMs(50U);
    assert(Algo_Filter_IsReady());
    assert(fabs(Algo_Filter_GetTilt()) < 0.2f);
    assert(fabs(Algo_Filter_GetPitch()) < 0.2f);
    assert(fabs(Algo_Filter_GetRoll()) < 0.2f);
    assert(App_Ctrl_SafetyReady() == ROBOT_RANGING_VALIDATED);
    assert(motorSpeed[2] == 0 && motorSpeed[3] == 0 && standby[2] == 0U);
    if (!ROBOT_RANGING_VALIDATED)
    {
        assert(App_SM_GetState() == STATE_RANGE_HOLD);
        App_Ctrl_UpdateMotors(3000, 3000);
        stopped();
    }
}

static void testTiltLatch(void)
{
    uint8_t axis;
    for (axis = 1U; axis <= 2U; axis++)
    {
        resetAt(0U);
        runMs(3040U);
        tilt(30.0f, axis);
        runMs(190U);
        assert(App_Ctrl_GetSafetyState() != STATE_TILT_STOP);
        runMs(30U);
        assert(App_SM_GetState() == STATE_TILT_STOP);
        assert(standby[1] == 0U && standby[2] == 0U);
        stopped();
        tilt(0.0f, axis);
        runMs(1000U);
        App_SM_SetState(STATE_SEARCH, 0U);
        App_Ctrl_UpdateMotors(3000, 3000);
        assert(App_SM_GetState() == STATE_TILT_STOP);
        stopped();
    }
    resetAt(0U);
    runMs(3040U);
    tilt(50.0f, 2U);
    runMs(10U);
    assert(App_Ctrl_GetSafetyState() == STATE_TILT_STOP);
    stopped();
    resetAt(0U);
    tilt(90.0f, 1U);
    runMs(20U);
    assert(App_SM_GetState() == STATE_TILT_STOP);
    stopped();
    resetAt(0U);
    runMs(3040U);
    tilt(-30.0f, 1U);
    runMs(220U);
    assert(App_SM_GetState() == STATE_TILT_STOP);
    resetAt(0U);
    rawAccel[0] = -2048;
    runMs(20U);
    assert(App_SM_GetState() == STATE_TILT_STOP);
}

static void testTransientAndDerating(void)
{
    int16_t levelOutput;
    uint8_t i;
    resetAt(0U);
    runMs(3040U);
    for (i = 0U; i < 20U; i++) App_Ctrl_UpdateMotors(3000, 3000);
    levelOutput = motorSpeed[0];
    tilt(20.0f, 1U);
    runMs(10U);
    App_Ctrl_UpdateMotors(3000, 3000);
    if (ROBOT_RANGING_VALIDATED)
        assert(motorSpeed[0] > 0 && motorSpeed[0] < levelOutput);
    tilt(30.0f, 1U);
    runMs(150U);
    tilt(0.0f, 1U);
    runMs(100U);
    tilt(30.0f, 1U);
    runMs(150U);
    assert(App_Ctrl_GetSafetyState() != STATE_TILT_STOP);
}

static void testImuFailure(void)
{
    resetAt(0U);
    runMs(3040U);
    imuFail = 1U;
    runMs(99U);
    assert(App_Ctrl_GetSafetyState() != STATE_IMU_STOP);
    runMs(1U);
    assert(App_Ctrl_GetSafetyState() == STATE_IMU_STOP);
    assert(standby[1] == 0U);
    stopped();
    imuFail = 0U;
    runMs(200U);
    assert(App_SM_GetState() == STATE_IMU_STOP);
    resetAt(0U);
    memset(rawAccel, 0, sizeof(rawAccel));
    runMs(120U);
    assert(App_SM_GetState() == STATE_IMU_STOP);
    resetAt(0U);
    rawGyro[1] = 300; /* Calibration while moving must not arm. */
    runMs(3520U);
    assert(App_SM_GetState() == STATE_IMU_STOP);
    stopped();
    resetAt(0U);
    runMs(2500U);
    memset(rawAccel, 0, sizeof(rawAccel));
    runMs(10U);
    rawAccel[0] = 2048;
    runMs(1010U);
    assert(App_SM_GetState() == STATE_IMU_STOP);
    resetAt(0xFFFFF000U);
    runMs(3040U);
    runMs(1100U);
    imuFail = 1U;
    runMs(100U);
    assert(App_Ctrl_GetSafetyState() == STATE_IMU_STOP);
    resetAt(0U);
    runMs(3040U);
    tick += 150U; /* A late successful read must not hide a scheduler gap. */
    App_Task_Scheduler();
    assert(App_Ctrl_GetSafetyState() == STATE_IMU_STOP);
}

static void testTelemetry(void)
{
    App_Protocol_SendStatus("TILT_STOP", -25.3f, 179.9f, 200, 200, 200);
    assert(strncmp(tx, "STAT,TILT_STOP,-25.3,179.9,", 25U) == 0);
}

static void testVisionFrames(void)
{
    unsigned int i;
    const char *bad[] = {"TRASH,,\n", "TRASH,320,\n", "TRASH,320,123junk\n",
                         "TRASH,640,320\n", "TRASH,320,-1\n",
                         "TRASH,99999999999999999999,0\n"};
    for (i = 0U; i < sizeof(bad) / sizeof(bad[0]); i++)
    {
        App_Protocol_Init();
        packet(bad[i]);
        assert(!App_Protocol_PacketReady());
    }
    App_Protocol_Init();
    packet("TRASH,320,320");
    for (i = 0U; i < 100U; i++) App_Protocol_ParseByte(' ');
    packet("\n");
    assert(!App_Protocol_PacketReady());
    packet("TRASH,320,320\n");
    assert(App_Protocol_PacketReady());
    packet("TRASH,,\n");
    assert(App_Protocol_GetPacket()->x == 320);
}

static void testVisionAndAvoidance(void)
{
    if (!ROBOT_RANGING_VALIDATED) return;
    resetAt(0U);
    runMs(3040U);
    packet("TRASH,320,320\n");
    runMs(20U);
    assert(App_SM_GetState() == STATE_DETECT);
    packet("TRASH,320,320\n");
    runMs(20U);
    assert(App_SM_GetState() == STATE_APPROACH);
    packet("TRASH,320,320\n");
    runMs(20U);
    assert(motorSpeed[0] > 0 && motorSpeed[0] == motorSpeed[1]);
    distanceCm[0] = 25.0f;
    runMs(20U);
    assert(App_SM_GetState() == STATE_AVOID);
    assert(motorSpeed[2] == 0 && motorSpeed[3] == 0);
    distanceCm[1] = distanceCm[2] = 40.0f;
    runMs(20U);
    stopped();
    distanceCm[2] = 200.0f;
    runMs(20U);
    assert(motorSpeed[0] > 0 && motorSpeed[1] < 0);
    distanceValid[0] = 0U;
    runMs(20U);
    stopped();

    resetAt(0U);
    runMs(3040U);
    App_Ctrl_SetTarget(320, 320);
    App_SM_SetState(STATE_APPROACH, 10000U);
    runMs(300U);
    assert(motorSpeed[0] > 0);
    runMs(220U);
    assert(App_SM_GetState() == STATE_SEARCH);
    stopped();
    distanceCm[1] = 30.0f;
    App_Ctrl_UpdateMotors(-2000, 2000);
    stopped();
}

static void testImuMotorBench(void)
{
    uint8_t i;
    resetAt(0U);
    for (i = 0U; i < 3U; i++)
    {
        distanceValid[i] = 0U;
        distanceCm[i] = 10.0f;
    }
    runMs(2990U);
    stopped();
    assert(standby[1] == 0U);
    runMs(50U);
    assert(App_SM_GetState() == STATE_IMU_TEST);
    assert(motorSpeed[0] > 0 && motorSpeed[0] < ROBOT_IMU_TEST_PWM);
    runMs(((ROBOT_IMU_TEST_PWM + RAMP_STEP - 1U) / RAMP_STEP) * 20U);
    assert(motorSpeed[0] == ROBOT_IMU_TEST_PWM);
    assert(motorSpeed[0] == motorSpeed[1]);
    assert(App_Ctrl_GetLeftPwm() == motorSpeed[0]);
    assert(App_Ctrl_GetRightPwm() == motorSpeed[1]);
    tilt(20.0f, 1U);
    runMs(200U);
    assert(motorSpeed[0] > ROBOT_IMU_TEST_PWM * 0.575f &&
           motorSpeed[0] < ROBOT_IMU_TEST_PWM * 0.625f);
    assert(motorSpeed[0] == motorSpeed[1]);
    assert(strncmp(tx, "MPUTEST,", 8U) == 0);
    tilt(0.0f, 1U);
    runMs(((ROBOT_IMU_TEST_PWM + RAMP_STEP - 1U) / RAMP_STEP) * 20U);
    assert(motorSpeed[0] == ROBOT_IMU_TEST_PWM);
    packet("TRASH,0,639\n");
    runMs(20U);
    assert(App_SM_GetState() == STATE_IMU_TEST);
    assert(motorSpeed[0] == motorSpeed[1]);
    assert(motorSpeed[2] == 0 && motorSpeed[3] == 0 && standby[2] == 0U);
    tilt(30.0f, 2U);
    runMs(220U);
    assert(App_SM_GetState() == STATE_TILT_STOP);
    assert(App_Ctrl_GetLeftPwm() == 0 && App_Ctrl_GetRightPwm() == 0);
    stopped();

    resetAt(0U);
    runMs(33000U);
    assert(App_SM_GetState() == STATE_IMU_TEST);
    runMs(20U);
    assert(App_SM_GetState() == STATE_TEST_DONE);
    assert(!App_Ctrl_SafetyReady());
    assert(standby[1] == 0U && standby[2] == 0U);
    stopped();
    runMs(2000U);
    App_Ctrl_UpdateMotors(3000, 3000);
    assert(App_SM_GetState() == STATE_TEST_DONE);
    stopped();
    resetAt(0xFFFFF000U);
    runMs(33020U);
    assert(App_SM_GetState() == STATE_TEST_DONE);
    stopped();
    BSP_MpuMotorTest_SendStatus(20.0f, 1200, 1200);
    assert(strcmp(tx, "MPUTEST,20.0,1200,1200\n") == 0);
}

static void testCalibrationTolerance(void)
{
    uint32_t i;
    const Attitude_t *att;

    /* Reproduce the measured ~1.10 g level sample and raw gyro offset. */
    resetAt(0U);
    rawAccel[0] = 2253;
    rawGyro[1] = 179;
    runMs(3040U);
    assert(Algo_Filter_IsReady());
    assert(Algo_Filter_GetCalibCount() >= 200U);
    assert(App_Ctrl_GetSafetyState() != STATE_IMU_STOP);
    att = Algo_Filter_GetAttitude();
    assert(fabs(att->gyroOffsetX - 179.0f) < 0.01f);
    if (ROBOT_IMU_MOTOR_TEST_ENABLE)
    {
        runMs(500U);
        assert(motorSpeed[0] == ROBOT_IMU_TEST_PWM);
        assert(standby[1] == 1U);
    }

    resetAt(0U);
    rawAccel[0] = 2253;
    rawGyro[2] = 257;
    runMs(3040U);
    assert(Algo_Filter_IsReady());

    /* Both endpoints meet the absolute limit, but shaking must not arm. */
    resetAt(0U);
    for (i = 0U; i < 352U; i++)
    {
        rawGyro[1] = (i & 1U) ? 100 : -100;
        runMs(10U);
    }
    assert(App_SM_GetState() == STATE_IMU_STOP);
    stopped();

    resetAt(0U);
    for (i = 0U; i < 352U; i++)
    {
        rawAccel[0] = (i & 1U) ? 2270 : 1980;
        runMs(10U);
    }
    assert(App_SM_GetState() == STATE_IMU_STOP);
    stopped();

    /* Small per-sample changes still accumulate to a rejected window. */
    resetAt(0U);
    for (i = 0U; i < 352U; i++)
    {
        rawGyro[1] = (int16_t)((int32_t)(i % 200U) - 100);
        runMs(10U);
    }
    assert(App_SM_GetState() == STATE_IMU_STOP);
    stopped();

    /* Allow a late stationary window without extending the 3.5 s deadline. */
    resetAt(0U);
    rawGyro[1] = 300;
    runMs(1200U);
    rawGyro[1] = 179;
    runMs(1800U);
    assert(App_SM_GetState() == STATE_INIT);
    assert(!Algo_Filter_IsReady());
    stopped();
    runMs(240U);
    assert(Algo_Filter_IsReady());
    assert(App_Ctrl_GetSafetyState() != STATE_IMU_STOP);

    resetAt(0U);
    rawAccel[0] = 2500;
    runMs(3520U);
    assert(App_SM_GetState() == STATE_IMU_STOP);
    stopped();
}

int main(void)
{
    testCalibrationTolerance();
    if (ROBOT_IMU_MOTOR_TEST_ENABLE)
    {
        testImuMotorBench();
        testTiltLatch();
        testImuFailure();
        testTelemetry();
        testVisionFrames();
        printf("PASS: isolated IMU motor test, PWM/telemetry, no sonar/vision dependency, timeout, protection/latch, timer wrap\n");
        return 0;
    }
    testBootAndMapping();
    testTiltLatch();
    testTransientAndDerating();
    testImuFailure();
    testTelemetry();
    testVisionFrames();
    testVisionAndAvoidance();
    printf("PASS: mapping, startup, tilt timing/latch, derating, IMU loss, calibration, wrap, telemetry, net/avoidance (ranging=%d)\n", ROBOT_RANGING_VALIDATED);
    return 0;
}
