#include "app_state_machine.h"
#include "app_control.h"
#include "app_protocol.h"
#include "bsp_ultrasonic.h"
#include "bsp_led.h"
#include "bsp_systick.h"
#include "algorithm/algo_filter.h"

static RobotState_t g_state = STATE_INIT;
static uint32_t g_stateEnterTick = 0;
static uint32_t g_stateTimeout = 0;

static const char *g_stateNames[] =
{
    "INIT", "SEARCH", "DETECT", "APPROACH", "COLLECT", "AVOID", "RETURN"
};

static uint8_t  g_avoidDir = 0;
static uint8_t  g_avoidClearCnt = 0;    /* consecutive cycles with front > 80cm */
static uint8_t  g_searchBlockCnt = 0;   /* consecutive cycles with front < 50cm in SEARCH */

void App_SM_Init(void)
{
    g_state = STATE_INIT;
    g_stateEnterTick = BSP_GetTick();
    g_stateTimeout = 3000;
    g_avoidDir = 0;
    Algo_Filter_StartCalibration();  /* start gyro bias collection */
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
            Algo_Filter_FinishCalibration();  /* compute gyro offsets */
            App_SM_SetState(STATE_SEARCH, 0);
            BSP_LED_Off();
        }
        break;

    /* ---- SEARCH: serpentine cruise, periodic ultrasonic ---- */
    case STATE_SEARCH:
        BSP_Ultrasonic_Update();

        /*
         * Debounce obstacle detection: require 2 consecutive <50cm
         * readings before triggering AVOID, to avoid re-entering
         * immediately after a rotation-only exit.
         */
        if (BSP_Ultrasonic_GetFront() < 50.0f)
        {
            g_searchBlockCnt++;
            if (g_searchBlockCnt >= 2)
            {
                g_searchBlockCnt = 0;
                App_SM_SetState(STATE_AVOID, 3000);
            }
        }
        else
        {
            g_searchBlockCnt = 0;
        }

        if (packetReady && g_state == STATE_SEARCH)
        {
            pkt = App_Protocol_GetPacket();
            if (pkt->type == PKT_TRASH)
            {
                g_searchBlockCnt = 0;
                App_Ctrl_SetTarget(pkt->x, pkt->y);
                App_SM_SetState(STATE_DETECT, 500);
            }
        }
        else if (g_state == STATE_SEARCH)
        {
            if (!BSP_Ultrasonic_IsValid(US_FRONT))
            {
                /* Front ranging unavailable — hold position instead of cruising blind. */
                App_Ctrl_UpdateMotors(0, 0);
            }
            else
            {
                /*
                 * Before executing cruise turn, check the corresponding
                 * side ultrasonic. If blocked, override with straight-line
                 * to avoid turning into a wall.
                 */
                uint8_t phase = App_Ctrl_GetCruisePhase();

                if (phase == 1 && BSP_Ultrasonic_GetRight() < 60.0f)
                {
                    App_Ctrl_UpdateMotors(CRUISE_SPEED, CRUISE_SPEED);
                }
                else if (phase == 3 && BSP_Ultrasonic_GetLeft() < 60.0f)
                {
                    App_Ctrl_UpdateMotors(CRUISE_SPEED, CRUISE_SPEED);
                }
                else
                {
                    App_Ctrl_SearchCruise();
                }
            }
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

        if (!BSP_Ultrasonic_IsValid(US_FRONT))
        {
            /* Front ranging unavailable — unknown obstacle ahead: stop safely
             * instead of driving forward blind. */
            App_Ctrl_StopAll();
            App_SM_SetState(STATE_SEARCH, 0);
        }
        else if (BSP_Ultrasonic_GetFront() < 30.0f)
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
            if (!App_Ctrl_ApproachTarget())
            {
                App_Ctrl_StopAll();
                App_SM_SetState(STATE_SEARCH, 0);
            }
        }
        else
        {
            if (!App_Ctrl_ApproachTarget())
            {
                App_Ctrl_StopAll();
                App_SM_SetState(STATE_SEARCH, 0);
            }
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

    /* ---- AVOID: turn toward open direction with debounced early-exit ---- */
    case STATE_AVOID:
    {
        uint32_t elapsed = BSP_GetTick() - g_stateEnterTick;

        BSP_Ultrasonic_Update();

        /* Re-evaluate which side has more room every cycle */
        {
            float leftDist  = BSP_Ultrasonic_GetLeft();
            float rightDist = BSP_Ultrasonic_GetRight();
            g_avoidDir = (leftDist > rightDist) ? 1 : 2;
            App_Ctrl_AvoidTurn(g_avoidDir);
        }

        /*
         * Debounced early-exit: only check after minimum 500ms dwell,
         * and require 3 consecutive "clear" readings (>80cm).
         * This prevents the front sensor sweeping across obstacles
         * during rotation from causing state oscillation.
         */
        if (elapsed > 500)
        {
            if (BSP_Ultrasonic_GetFront() > 80.0f)
            {
                g_avoidClearCnt++;
                if (g_avoidClearCnt >= 3)
                {
                    App_Ctrl_StopAll();
                    g_avoidClearCnt = 0;
                    App_SM_SetState(STATE_SEARCH, 0);
                    break;
                }
            }
            else
            {
                g_avoidClearCnt = 0;  /* reset — still blocked */
            }
        }

        if (SM_Timeout())
        {
            App_Ctrl_StopAll();
            g_avoidClearCnt = 0;
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
