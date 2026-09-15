/**
 * Motion control: differential thrust, PID-guided approach, cruise, collection.
 *
 * Key improvements:
 *   - Deadband: no PID correction when target is within IMG_DEADBAND pixels of center
 *   - Ramp: motor speeds change gradually (RAMP_STEP per call) to prevent current spikes
 *   - OnStateChange: stops ALL motors including propulsion
 */

#include "app_control.h"
#include "robot_config.h"
#include "bsp_ultrasonic.h"
#include "driver/drv_tb6612.h"
#include "algorithm/algo_pid.h"
#include "algorithm/algo_filter.h"
#include "bsp_systick.h"

#define TARGET_TIMEOUT_MS   500   /* 500ms without update → target lost */

/* First IMU stop evidence: reason 1=valid sample age, 2=startup readiness. */
volatile uint32_t g_debugImuStopReason;
volatile uint32_t g_debugStopCalibCount;
volatile uint32_t g_debugStopSampleAge;
static uint32_t g_lastImuTick;
static uint32_t g_safetyStartTick;
static uint32_t g_tiltStartTick;
static uint8_t g_tiltPending;
static uint8_t g_safetyReady;
static RobotState_t g_safetyState;
static float g_thrustScale;
static int16_t g_outputLeft;
static int16_t g_outputRight;

static PID_t g_pidX;
static int16_t g_targetX = 0;
static int16_t g_targetY = 0;
static uint8_t  g_targetValid     = 0;
static uint32_t g_targetTimestamp = 0;
static uint32_t g_cruisePhase     = 0;
static uint32_t g_lastCruiseTick  = 0;
static int16_t g_cruiseLeftTarget  = 0;
static int16_t g_cruiseRightTarget = 0;

/* Ramp state: current vs target for left/right motors */
static int16_t g_rampLeft  = 0;
static int16_t g_rampRight = 0;

void App_Ctrl_Init(void)
{
    g_debugImuStopReason = 0U;
    g_debugStopCalibCount = 0U;
    g_debugStopSampleAge = 0U;
    g_lastImuTick = BSP_GetTick();
    g_safetyStartTick = g_lastImuTick;
    g_outputLeft = 0;
    g_outputRight = 0;
    BSP_MpuMotorTest_Init();
    g_tiltPending = 0U;
    g_safetyReady = 0U;
    g_safetyState = STATE_INIT;
    g_thrustScale = 1.0f;
    DRV_TB6612_SetStandby(1, 0);
    /* Yaw-only PID. kp=10 -> max P = 10*320 = 3200 < 4000 clamp, so steering
     * stays proportional across the full 0~320px error instead of saturating
     * to bang-bang (kp=18 saturated at ~222px). */
    Algo_PID_Init(&g_pidX, 10.0f, 0.2f, 2.0f, 4000.0f, -4000.0f, 4000.0f);
    g_targetX = 0;
    g_targetY = 0;
    g_targetValid = 0;
    g_cruisePhase = 0;
    g_lastCruiseTick = 0;
    g_cruiseLeftTarget = 0;
    g_cruiseRightTarget = 0;
    g_rampLeft = 0;
    g_rampRight = 0;
}

void App_Ctrl_SetTarget(int16_t x, int16_t y)
{
    g_targetX = x;
    g_targetY = y;   /* retained for future distance control; not used for steering */
    (void)g_targetY;
    g_targetValid     = 1;
    g_targetTimestamp = BSP_GetTick();
}

/* ---- Ramped motor update: smooth transition to target speed ---- */
static int16_t RampTo(int16_t current, int16_t target)
{
    int16_t diff = target - current;
    if (diff > RAMP_STEP)
        return current + RAMP_STEP;
    else if (diff < -RAMP_STEP)
        return current - RAMP_STEP;
    else
        return target;
}

void App_Ctrl_UpdateMotors(int16_t left, int16_t right)
{
    if (!App_Ctrl_SafetyReady())
    {
        App_Ctrl_StopAll();
        return;
    }

#if !ROBOT_IMU_MOTOR_TEST_ENABLE
    /* Left/right beams cover the forward quarters, not the stern.
     * Do not turn into a blocked or unmeasured forward quarter. */
    if (!BSP_Ultrasonic_IsValid(US_FRONT) ||
        (left >= 0 && right >= 0 &&
         BSP_Ultrasonic_GetFront() < ROBOT_FRONT_AVOID_CM) ||
        (left < right && (!BSP_Ultrasonic_IsValid(US_LEFT) ||
                         BSP_Ultrasonic_GetLeft() < ROBOT_SIDE_AVOID_CM)) ||
        (right < left && (!BSP_Ultrasonic_IsValid(US_RIGHT) ||
                         BSP_Ultrasonic_GetRight() < ROBOT_SIDE_AVOID_CM)))
    {
        App_Ctrl_StopAll();
        return;
    }

#endif

    /* Apply ramp */
    g_rampLeft  = RampTo(g_rampLeft,  left);
    g_rampRight = RampTo(g_rampRight, right);

    g_outputLeft = (int16_t)(g_rampLeft * g_thrustScale);
    g_outputRight = (int16_t)(g_rampRight * g_thrustScale);
    DRV_TB6612_SetSpeed(MOTOR_LEFT, g_outputLeft);
    DRV_TB6612_SetSpeed(MOTOR_RIGHT, g_outputRight);
}

void App_Ctrl_SearchCruise(void)
{
    uint32_t now = BSP_GetTick();

    /* Change the serpentine direction only every two seconds. */
    if (now - g_lastCruiseTick >= 2000)
    {
        g_lastCruiseTick = now;

        switch (g_cruisePhase & 0x03)
        {
        case 0:
            g_cruiseLeftTarget = CRUISE_SPEED;
            g_cruiseRightTarget = CRUISE_SPEED;
            break;
        case 1:
            g_cruiseLeftTarget = CRUISE_SPEED + 1000;
            g_cruiseRightTarget = CRUISE_SPEED - 500;
            break;
        case 2:
            g_cruiseLeftTarget = CRUISE_SPEED;
            g_cruiseRightTarget = CRUISE_SPEED;
            break;
        default:
            g_cruiseLeftTarget = CRUISE_SPEED - 500;
            g_cruiseRightTarget = CRUISE_SPEED + 1000;
            break;
        }
        g_cruisePhase++;
    }

    /* App_SM_Run calls this every 20 ms: keep advancing the PWM ramp toward
     * the selected target instead of advancing it only once every two seconds. */
    App_Ctrl_UpdateMotors(g_cruiseLeftTarget, g_cruiseRightTarget);
}

uint8_t App_Ctrl_ApproachTarget(void)
{
    static uint32_t g_lastPidTick = 0;
    uint32_t now = BSP_GetTick();
    float dt, pidX;
    float errorX;
    int16_t left, right;

    /* Actual dt since last call — capped for safety */
    dt = (float)(now - g_lastPidTick) * 0.001f;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.1f)   dt = 0.1f;
    g_lastPidTick = now;

    /* Target lost or stale: STOP, do not keep driving straight ahead.
     * (Previously this fell through to full approach speed for up to 10s.) */
    if (!g_targetValid || (now - g_targetTimestamp) > TARGET_TIMEOUT_MS)
    {
        g_targetValid = 0;
        App_Ctrl_StopAll();
        return 0;
    }

    /* Deadband: target horizontally centered -> straight ahead. Only x is a
     * steering (yaw) error for a differential surface robot. */
    errorX = IMG_CENTER_X - (float)g_targetX;
    if (errorX < 0.0f) errorX = -errorX;

    if (errorX < (float)IMG_DEADBAND)
    {
        App_Ctrl_UpdateMotors(APPROACH_SPEED, APPROACH_SPEED);
        return 1;
    }

    /* Yaw-only PID. Vertical (y) offset is distance/pitch, not a steering error. */
    Algo_PID_SetSetpoint(&g_pidX, IMG_CENTER_X);
    pidX = Algo_PID_Compute(&g_pidX, (float)g_targetX, dt);

    left  = APPROACH_SPEED - (int16_t)pidX;
    right = APPROACH_SPEED + (int16_t)pidX;

    if (left  > TB_PWM_MAX_DUTY) left  = TB_PWM_MAX_DUTY;
    if (left  < -TB_PWM_MAX_DUTY) left  = -TB_PWM_MAX_DUTY;
    if (right > TB_PWM_MAX_DUTY) right = TB_PWM_MAX_DUTY;
    if (right < -TB_PWM_MAX_DUTY) right = -TB_PWM_MAX_DUTY;

    App_Ctrl_UpdateMotors(left, right);
    return 1;
}

void App_Ctrl_AvoidTurn(uint8_t direction)
{
    if (direction == 1)
    {
        App_Ctrl_UpdateMotors(-AVOID_SPEED, AVOID_SPEED);
    }
    else
    {
        App_Ctrl_UpdateMotors(AVOID_SPEED, -AVOID_SPEED);
    }
}

void App_Ctrl_StopAll(void)
{
    g_outputLeft = 0;
    g_outputRight = 0;
    g_rampLeft  = 0;
    g_rampRight = 0;
    DRV_TB6612_StopAll();
}

void App_Ctrl_ReturnBase(void)
{
    /* No position sensor is installed; returning home must remain a safe stop. */
    DRV_TB6612_StopAll();
}

uint8_t App_Ctrl_GetCruisePhase(void)
{
    return (uint8_t)((g_cruisePhase == 0U) ? 0U : ((g_cruisePhase - 1U) & 0x03));
}

static void Safety_Latch(RobotState_t fault)
{
    if (g_safetyState != STATE_TILT_STOP && g_safetyState != STATE_IMU_STOP)
        g_safetyState = fault;
    g_safetyReady = 0U;
    App_Ctrl_StopAll();
    DRV_TB6612_SetStandby(1, 0);
}

void App_Ctrl_SafetyUpdate(uint8_t sampleValid)
{
    uint32_t now = BSP_GetTick();
    float tilt;

    if (g_safetyState == STATE_TILT_STOP || g_safetyState == STATE_IMU_STOP)
    {
        Safety_Latch(g_safetyState);
        return;
    }
    if (sampleValid)
    {
        g_lastImuTick = now;
        tilt = Algo_Filter_GetTilt();
        if (tilt >= ROBOT_TILT_HARD_DEG)
        {
            Safety_Latch(STATE_TILT_STOP);
            return;
        }
        if (tilt >= ROBOT_TILT_STOP_DEG)
        {
            if (!g_tiltPending)
            {
                g_tiltPending = 1U;
                g_tiltStartTick = now;
            }
            if ((uint32_t)(now - g_tiltStartTick) >= ROBOT_TILT_STOP_MS)
            {
                Safety_Latch(STATE_TILT_STOP);
                return;
            }
        }
        else
            g_tiltPending = 0U;

        g_thrustScale = 1.0f;
        if (tilt > ROBOT_TILT_SLOW_DEG)
        {
            g_thrustScale = 1.0f - (1.0f - ROBOT_TILT_MIN_THRUST_SCALE) *
                (tilt - ROBOT_TILT_SLOW_DEG) /
                (ROBOT_TILT_STOP_DEG - ROBOT_TILT_SLOW_DEG);
            if (g_thrustScale < ROBOT_TILT_MIN_THRUST_SCALE)
                g_thrustScale = ROBOT_TILT_MIN_THRUST_SCALE;
        }
    }
    if ((uint32_t)(now - g_lastImuTick) >= ROBOT_IMU_STALE_MS ||
        (!Algo_Filter_IsReady() &&
         (uint32_t)(now - g_safetyStartTick) >= ROBOT_IMU_STARTUP_MS))
    {
        g_debugStopSampleAge = (uint32_t)(now - g_lastImuTick);
        g_debugImuStopReason = (g_debugStopSampleAge >= ROBOT_IMU_STALE_MS) ? 1U : 2U;
        g_debugStopCalibCount = Algo_Filter_GetCalibCount();
        Safety_Latch(STATE_IMU_STOP);
        return;
    }
    if (!g_safetyReady && sampleValid && Algo_Filter_IsReady() &&
        Algo_Filter_GetTilt() < ROBOT_TILT_STOP_DEG)
    {
        g_safetyReady = 1U;
        g_safetyState = STATE_SEARCH;
#if ROBOT_RANGING_VALIDATED || ROBOT_IMU_MOTOR_TEST_ENABLE
        DRV_TB6612_SetStandby(1, 1);
#endif
    }
}

uint8_t App_Ctrl_SafetyReady(void)
{
#if ROBOT_IMU_MOTOR_TEST_ENABLE
    if (BSP_MpuMotorTest_IsFinished()) return 0U;
#endif
    return g_safetyReady && (ROBOT_RANGING_VALIDATED || ROBOT_IMU_MOTOR_TEST_ENABLE) &&
           (uint32_t)(BSP_GetTick() - g_lastImuTick) < ROBOT_IMU_STALE_MS;
}

RobotState_t App_Ctrl_GetSafetyState(void) { return g_safetyState; }
int16_t App_Ctrl_GetLeftPwm(void) { return g_outputLeft; }
int16_t App_Ctrl_GetRightPwm(void) { return g_outputRight; }

void App_Ctrl_OnStateChange(RobotState_t newState)
{
    Algo_PID_Reset(&g_pidX);
    g_rampLeft  = 0;
    g_rampRight = 0;

    /* Also stop propulsion motors on state change to prevent runaway */
    App_Ctrl_StopAll();

    /* Only clear the target when entering a state that has no target.
     * DETECT / APPROACH keep the freshly-set target so SetTarget() -> SetState()
     * no longer loses it. */
    switch (newState)
    {
    case STATE_INIT:
    case STATE_SEARCH:
        g_cruisePhase = 0;
        g_lastCruiseTick = 0;
        g_cruiseLeftTarget = 0;
        g_cruiseRightTarget = 0;
        g_targetValid = 0;
        break;
    case STATE_COLLECT:
    case STATE_AVOID:
    case STATE_RETURN:
        g_targetValid = 0;
        break;
    default:
        break;
    }
}
