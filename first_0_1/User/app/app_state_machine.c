#include "app_state_machine.h"
#include "app_control.h"
#include "app_protocol.h"
#include "bsp_ultrasonic.h"
#include "bsp_led.h"
#include "bsp_systick.h"

static RobotState_t g_state = STATE_INIT;
static uint32_t g_stateEnterTick = 0;
static uint32_t g_stateTimeout = 0;

static const char *g_stateNames[] =
{
    "INIT", "SEARCH", "DETECT", "APPROACH", "COLLECT", "AVOID", "RETURN"
};

static uint8_t g_avoidDir = 0;

void App_SM_Init(void)
{
    g_state = STATE_INIT;
    g_stateEnterTick = BSP_GetTick();
    g_stateTimeout = 3000;
    g_avoidDir = 0;
}

void App_SM_SetState(RobotState_t newState, uint32_t timeoutMs)
{
    g_state = newState;
    g_stateEnterTick = BSP_GetTick();
    g_stateTimeout = timeoutMs;
    App_Ctrl_OnStateChange(newState);
}

static uint8_t SM_Timeout(void)
{
    return (BSP_GetTick() - g_stateEnterTick) >= g_stateTimeout;
}

void App_SM_Run(void)
{
    uint8_t packetReady = App_Protocol_PacketReady();
    K230Packet_t *pkt;

    switch (g_state)
    {
    /* ---- INIT: sensor calibration, gyro bias ---- */
    case STATE_INIT:
        BSP_LED_On();
        if (SM_Timeout())
        {
            App_SM_SetState(STATE_SEARCH, 0);
            BSP_LED_Off();
        }
        break;

    /* ---- SEARCH: serpentine cruise, periodic ultrasonic ---- */
    case STATE_SEARCH:
        BSP_Ultrasonic_Update();

        if (BSP_Ultrasonic_GetFront() < 50.0f)
        {
            App_SM_SetState(STATE_AVOID, 3000);
        }
        else if (packetReady)
        {
            pkt = App_Protocol_GetPacket();
            if (pkt->type == PKT_TRASH)
            {
                App_Ctrl_SetTarget(pkt->x, pkt->y);
                App_SM_SetState(STATE_DETECT, 500);
            }
        }
        else
        {
            App_Ctrl_SearchCruise();
        }
        break;

    /* ---- DETECT: confirm target exists ---- */
    case STATE_DETECT:
        BSP_Ultrasonic_Update();

        if (BSP_Ultrasonic_GetFront() < 50.0f)
        {
            App_SM_SetState(STATE_AVOID, 3000);
        }
        else if (packetReady)
        {
            pkt = App_Protocol_GetPacket();
            if (pkt->type == PKT_TRASH)
            {
                App_Ctrl_SetTarget(pkt->x, pkt->y);
                App_SM_SetState(STATE_APPROACH, 10000);
            }
        }
        else if (SM_Timeout())
        {
            App_SM_SetState(STATE_SEARCH, 0);
        }
        break;

    /* ---- APPROACH: PID-guided approach to trash ---- */
    case STATE_APPROACH:
        BSP_Ultrasonic_Update();

        if (BSP_Ultrasonic_GetFront() < 30.0f)
        {
            App_SM_SetState(STATE_COLLECT, 5000);
        }
        else if (BSP_Ultrasonic_GetFront() < 50.0f)
        {
            App_SM_SetState(STATE_AVOID, 3000);
        }
        else if (SM_Timeout())
        {
            /* Stop all motors before returning to search */
            App_Ctrl_StopAll();
            App_SM_SetState(STATE_SEARCH, 0);
        }
        else if (packetReady)
        {
            pkt = App_Protocol_GetPacket();
            if (pkt->type == PKT_TRASH)
            {
                App_Ctrl_SetTarget(pkt->x, pkt->y);
            }
            App_Ctrl_ApproachTarget();
        }
        else
        {
            App_Ctrl_ApproachTarget();
        }
        break;

    /* ---- COLLECT: activate roller + conveyor ---- */
    case STATE_COLLECT:
        App_Ctrl_StartCollection();
        if (SM_Timeout())
        {
            App_Ctrl_StopCollection();
            App_SM_SetState(STATE_SEARCH, 0);
        }
        break;

    /* ---- AVOID: turn 60 toward open direction ---- */
    case STATE_AVOID:
    {
        float leftDist  = BSP_Ultrasonic_GetLeft();
        float rightDist = BSP_Ultrasonic_GetRight();
        g_avoidDir = (leftDist >= rightDist) ? 1 : 2;
        App_Ctrl_AvoidTurn(g_avoidDir);

        if (SM_Timeout())
        {
            App_Ctrl_StopAll();
            App_SM_SetState(STATE_SEARCH, 0);
        }
        break;
    }

    /* ---- RETURN: return-to-base (reserved for GPS) ---- */
    case STATE_RETURN:
        App_Ctrl_ReturnBase();
        break;

    default:
        App_SM_SetState(STATE_SEARCH, 0);
        break;
    }
}

RobotState_t App_SM_GetState(void) { return g_state; }

const char *App_SM_GetStateName(void)
{
    if (g_state < STATE_COUNT)
        return g_stateNames[g_state];
    return "UNKNOWN";
}
