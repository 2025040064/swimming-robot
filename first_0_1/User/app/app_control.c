/**
 * Motion control: differential thrust, PID-guided approach, cruise, collection.
 *
 * Key improvements:
 *   - Deadband: no PID correction when target is within IMG_DEADBAND pixels of center
 *   - Ramp: motor speeds change gradually (RAMP_STEP per call) to prevent current spikes
 *   - OnStateChange: stops ALL motors including propulsion
 */

#include "app_control.h"
#include "driver/drv_tb6612.h"
#include "algorithm/algo_pid.h"
#include "algorithm/algo_filter.h"
#include "bsp_systick.h"

#define TARGET_TIMEOUT_MS   500   /* 500ms without update → target lost */

static PID_t g_pidX;
static int16_t g_targetX = 0;
static int16_t g_targetY = 0;
static uint8_t  g_targetValid     = 0;
static uint32_t g_targetTimestamp = 0;
static uint32_t g_cruisePhase     = 0;
static uint32_t g_lastCruiseTick  = 0;

/* Ramp state: current vs target for left/right motors */
static int16_t g_rampLeft  = 0;
static int16_t g_rampRight = 0;

void App_Ctrl_Init(void)
{
    /* Yaw-only PID. kp=10 -> max P = 10*320 = 3200 < 4000 clamp, so steering
     * stays proportional across the full 0~320px error instead of saturating
     * to bang-bang (kp=18 saturated at ~222px). */
    Algo_PID_Init(&g_pidX, 10.0f, 0.2f, 2.0f, 4000.0f, -4000.0f, 4000.0f);
    g_targetX = 0;
    g_targetY = 0;
    g_targetValid = 0;
    g_cruisePhase = 0;
    g_lastCruiseTick = 0;
    g_rampLeft = 0;
    g_rampRight = 0;
}

void App_Ctrl_SetTarget(int16_t x, int16_t y)
{
    g_targetX = x;
    g_targetY = y;   /* retained for future distance control; not used for steering */
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
    /* Apply ramp */
    g_rampLeft  = RampTo(g_rampLeft,  left);
    g_rampRight = RampTo(g_rampRight, right);

    DRV_TB6612_SetSpeed(MOTOR_LEFT,  g_rampLeft);
    DRV_TB6612_SetSpeed(MOTOR_RIGHT, g_rampRight);
}

void App_Ctrl_SearchCruise(void)
{
    uint32_t now = BSP_GetTick();
    if (now - g_lastCruiseTick < 2000) return;
    g_lastCruiseTick = now;

    switch (g_cruisePhase & 0x03)
    {
    case 0:
        App_Ctrl_UpdateMotors(CRUISE_SPEED, CRUISE_SPEED);
        break;
    case 1:
        App_Ctrl_UpdateMotors(CRUISE_SPEED + 1000, CRUISE_SPEED - 500);
        break;
    case 2:
        App_Ctrl_UpdateMotors(CRUISE_SPEED, CRUISE_SPEED);
        break;
    case 3:
        App_Ctrl_UpdateMotors(CRUISE_SPEED - 500, CRUISE_SPEED + 1000);
        break;
    }
    g_cruisePhase++;
}

void App_Ctrl_ApproachTarget(void)
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

    /* Target age timeout: if no update from K230 for >500ms, coast */
    if (g_targetValid && (now - g_targetTimestamp) > TARGET_TIMEOUT_MS)
    {
        g_targetValid = 0;
    }

    if (!g_targetValid)
    {
        App_Ctrl_UpdateMotors(APPROACH_SPEED, APPROACH_SPEED);
        return;
    }

    /* Deadband: target horizontally centered -> straight ahead. Only x is a
     * steering (yaw) error for a differential surface robot. */
    errorX = IMG_CENTER_X - (float)g_targetX;
    if (errorX < 0.0f) errorX = -errorX;

    if (errorX < (float)IMG_DEADBAND)
    {
        App_Ctrl_UpdateMotors(APPROACH_SPEED, APPROACH_SPEED);
        return;
    }

    /* Yaw-only PID. Vertical (y) offset is distance/pitch, not a steering error. */
    Algo_PID_SetSetpoint(&g_pidX, IMG_CENTER_X);
    pidX = Algo_PID_Compute(&g_pidX, (float)g_targetX, dt);

    left  = APPROACH_SPEED - (int16_t)pidX;
    right = APPROACH_SPEED + (int16_t)pidX;

    if (left  > 7200) left  = 7200;
    if (left  < -7200) left  = -7200;
    if (right > 7200) right = 7200;
    if (right < -7200) right = -7200;

    App_Ctrl_UpdateMotors(left, right);
}

void App_Ctrl_StartCollection(void)
{
    DRV_TB6612_SetSpeed(MOTOR_ROLLER, COLLECT_SPEED);
    DRV_TB6612_SetSpeed(MOTOR_CONVEYOR, COLLECT_SPEED / 2);
}

void App_Ctrl_StopCollection(void)
{
    DRV_TB6612_SetSpeed(MOTOR_ROLLER, 0);
    DRV_TB6612_SetSpeed(MOTOR_CONVEYOR, 0);
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
    g_rampLeft  = 0;
    g_rampRight = 0;
    DRV_TB6612_StopAll();
}

void App_Ctrl_ReturnBase(void)
{
    /* Reserved: will use GPS + compass to navigate home */
    DRV_TB6612_StopAll();
}

uint8_t App_Ctrl_GetCruisePhase(void)
{
    return (uint8_t)(g_cruisePhase & 0x03);
}

void App_Ctrl_OnStateChange(uint8_t newState)
{
    Algo_PID_Reset(&g_pidX);
    g_targetValid = 0;
    g_rampLeft  = 0;
    g_rampRight = 0;
    App_Ctrl_StopCollection();

    /* Also stop propulsion motors on state change to prevent runaway */
    DRV_TB6612_StopAll();
}
