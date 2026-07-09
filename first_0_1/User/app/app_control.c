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

static PID_t g_pidX;
static PID_t g_pidY;
static int16_t g_targetX = 0;
static int16_t g_targetY = 0;
static uint8_t g_targetValid = 0;
static uint32_t g_cruisePhase = 0;
static uint32_t g_lastCruiseTick = 0;

/* Ramp state: current vs target for left/right motors */
static int16_t g_rampLeft  = 0;
static int16_t g_rampRight = 0;

void App_Ctrl_Init(void)
{
    Algo_PID_Init(&g_pidX, 18.0f, 0.2f, 2.0f, -4000.0f, 4000.0f);
    Algo_PID_Init(&g_pidY, 6.0f, 0.05f, 0.5f, -2000.0f, 2000.0f);
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
    g_targetY = y;
    g_targetValid = 1;
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
    float dt = 0.02f;
    float pidX, pidY;
    float errorX, errorY;
    int16_t left, right;

    if (!g_targetValid)
    {
        App_Ctrl_UpdateMotors(APPROACH_SPEED, APPROACH_SPEED);
        return;
    }

    /* Deadband check: if target is near center, go straight */
    errorX = IMG_CENTER_X - (float)g_targetX;
    errorY = IMG_CENTER_Y - (float)g_targetY;

    if (errorX < 0.0f) errorX = -errorX;
    if (errorY < 0.0f) errorY = -errorY;

    if (errorX < (float)IMG_DEADBAND && errorY < (float)IMG_DEADBAND)
    {
        App_Ctrl_UpdateMotors(APPROACH_SPEED, APPROACH_SPEED);
        return;
    }

    /* PID correction when target is outside deadband */
    Algo_PID_SetSetpoint(&g_pidX, IMG_CENTER_X);
    Algo_PID_SetSetpoint(&g_pidY, IMG_CENTER_Y);

    pidX = Algo_PID_Compute(&g_pidX, (float)g_targetX, dt);
    pidY = Algo_PID_Compute(&g_pidY, (float)g_targetY, dt);

    left  = APPROACH_SPEED + (int16_t)pidX - (int16_t)pidY;
    right = APPROACH_SPEED - (int16_t)pidX + (int16_t)pidY;

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

void App_Ctrl_OnStateChange(uint8_t newState)
{
    Algo_PID_Reset(&g_pidX);
    Algo_PID_Reset(&g_pidY);
    g_targetValid = 0;
    g_rampLeft  = 0;
    g_rampRight = 0;
    App_Ctrl_StopCollection();

    /* Also stop propulsion motors on state change to prevent runaway */
    DRV_TB6612_StopAll();
}
