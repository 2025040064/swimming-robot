/**
 * Non-blocking ultrasonic driver for 3x AJ-SRP04M sensors.
 *
 * Each sensor cycles through a 4-phase state machine:
 *   IDLE -> TRIG_PULSE -> WAIT_ECHO_HIGH -> MEASURE_ECHO -> DONE
 * One phase advances per BSP_Ultrasonic_Update() call (~10ms),
 * so a full 3-sensor sweep takes ~12 calls = 120ms, without
 * ever blocking the scheduler.
 *
 * A first-order low-pass filter smooths measurements to reject
 * single-sample outliers caused by wave reflections.
 */

#include "bsp_ultrasonic.h"
#include "bsp_systick.h"

#define US_TIMEOUT_MS   60

/* --- Non-blocking state machine --- */
typedef enum
{
    US_PHASE_IDLE = 0,
    US_PHASE_TRIG_PULSE,
    US_PHASE_WAIT_ECHO_HIGH,
    US_PHASE_MEASURE_ECHO,
    US_PHASE_DONE
} US_Phase_t;

typedef struct
{
    GPIO_TypeDef *trigPort;
    uint16_t      trigPin;
    GPIO_TypeDef *echoPort;
    uint16_t      echoPin;
    US_Phase_t    phase;
    uint32_t      echoStartTick;
    uint32_t      timeoutTick;
    float         rawDistance;
} US_Sensor_t;

static US_Sensor_t g_sensors[3];
static uint8_t     g_sensorIdx = 0;     /* round-robin: 0=front, 1=left, 2=right */

static float g_frontDist = 999.0f;
static float g_leftDist  = 999.0f;
static float g_rightDist = 999.0f;

/* ---- Initialize hardware GPIOs ---- */
void BSP_Ultrasonic_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* Trig pins: Out_PP, low */
    GPIO_InitStructure.GPIO_Pin   = US_F_TRIG_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(US_F_TRIG_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(US_F_TRIG_PORT, US_F_TRIG_PIN);

    GPIO_InitStructure.GPIO_Pin   = US_L_TRIG_PIN | US_R_TRIG_PIN;
    GPIO_Init(US_L_TRIG_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(US_L_TRIG_PORT, US_L_TRIG_PIN);
    GPIO_ResetBits(US_R_TRIG_PORT, US_R_TRIG_PIN);

    /* Echo pins: IN_FLOATING */
    GPIO_InitStructure.GPIO_Pin   = US_F_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(US_F_ECHO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = US_L_ECHO_PIN | US_R_ECHO_PIN;
    GPIO_Init(US_L_ECHO_PORT, &GPIO_InitStructure);

    /* Init sensor control blocks */
    g_sensors[0].trigPort = US_F_TRIG_PORT; g_sensors[0].trigPin  = US_F_TRIG_PIN;
    g_sensors[0].echoPort = US_F_ECHO_PORT; g_sensors[0].echoPin  = US_F_ECHO_PIN;
    g_sensors[0].phase    = US_PHASE_IDLE;

    g_sensors[1].trigPort = US_L_TRIG_PORT; g_sensors[1].trigPin  = US_L_TRIG_PIN;
    g_sensors[1].echoPort = US_L_ECHO_PORT; g_sensors[1].echoPin  = US_L_ECHO_PIN;
    g_sensors[1].phase    = US_PHASE_IDLE;

    g_sensors[2].trigPort = US_R_TRIG_PORT; g_sensors[2].trigPin  = US_R_TRIG_PIN;
    g_sensors[2].echoPort = US_R_ECHO_PORT; g_sensors[2].echoPin  = US_R_ECHO_PIN;
    g_sensors[2].phase    = US_PHASE_IDLE;

    g_sensorIdx = 0;
}

/* ---- Advance ONE sensor by one phase (non-blocking) ---- */
static void Ultrasonic_Tick(US_Sensor_t *s)
{
    switch (s->phase)
    {
    case US_PHASE_IDLE:
        /* Start a new measurement cycle */
        s->phase = US_PHASE_TRIG_PULSE;
        break;

    case US_PHASE_TRIG_PULSE:
        /* Send 10us trigger pulse */
        GPIO_ResetBits(s->trigPort, s->trigPin);
        for (volatile uint8_t i = 0; i < 5; i++) __NOP();
        GPIO_SetBits(s->trigPort, s->trigPin);
        for (volatile uint8_t i = 0; i < 20; i++) __NOP();
        GPIO_ResetBits(s->trigPort, s->trigPin);
        s->timeoutTick = BSP_GetTick() + US_TIMEOUT_MS;
        s->phase = US_PHASE_WAIT_ECHO_HIGH;
        break;

    case US_PHASE_WAIT_ECHO_HIGH:
        if (GPIO_ReadInputDataBit(s->echoPort, s->echoPin) == 1)
        {
            s->echoStartTick = BSP_GetTick();
            s->timeoutTick   = s->echoStartTick + US_TIMEOUT_MS;
            s->phase = US_PHASE_MEASURE_ECHO;
        }
        else if (BSP_GetTick() > s->timeoutTick)
        {
            s->rawDistance = 999.0f;
            s->phase = US_PHASE_DONE;
        }
        break;

    case US_PHASE_MEASURE_ECHO:
        if (GPIO_ReadInputDataBit(s->echoPort, s->echoPin) == 0)
        {
            float dist = (BSP_GetTick() - s->echoStartTick) * 0.017f * 100.0f;
            if (dist < 2.0f)   dist = 2.0f;
            if (dist > 400.0f) dist = 400.0f;
            s->rawDistance = dist;
            s->phase = US_PHASE_DONE;
        }
        else if (BSP_GetTick() > s->timeoutTick)
        {
            s->rawDistance = 999.0f;
            s->phase = US_PHASE_DONE;
        }
        break;

    case US_PHASE_DONE:
        /* Will be re-triggered after all sensors finish */
        break;
    }
}

/* ---- First-order low-pass filter ---- */
static float LowPass(float prev, float raw)
{
    return prev * (1.0f - US_FILTER_ALPHA) + raw * US_FILTER_ALPHA;
}

/* ---- Called at ~10ms from task scheduler ---- */
void BSP_Ultrasonic_Update(void)
{
    static uint32_t g_lastUpdate = 0;
    uint32_t now = BSP_GetTick();

    /* Throttle to ~10ms between ticks */
    if (now - g_lastUpdate < 10) return;
    g_lastUpdate = now;

    /* Advance the currently selected sensor by one phase */
    Ultrasonic_Tick(&g_sensors[g_sensorIdx]);

    /* If this sensor is done, move to the next */
    if (g_sensors[g_sensorIdx].phase == US_PHASE_DONE)
    {
        g_sensorIdx++;
    }

    /* All 3 sensors done? Apply low-pass filter and restart */
    if (g_sensorIdx >= 3)
    {
        g_frontDist = LowPass(g_frontDist, g_sensors[0].rawDistance);
        g_leftDist  = LowPass(g_leftDist,  g_sensors[1].rawDistance);
        g_rightDist = LowPass(g_rightDist, g_sensors[2].rawDistance);

        g_sensors[0].phase = US_PHASE_IDLE;
        g_sensors[1].phase = US_PHASE_IDLE;
        g_sensors[2].phase = US_PHASE_IDLE;
        g_sensorIdx = 0;
    }
}

float BSP_Ultrasonic_GetDistance(uint8_t sensor)
{
    switch (sensor)
    {
    case US_FRONT: return g_frontDist;
    case US_LEFT:  return g_leftDist;
    case US_RIGHT: return g_rightDist;
    default:       return 999.0f;
    }
}

float BSP_Ultrasonic_GetFront(void) { return g_frontDist; }
float BSP_Ultrasonic_GetLeft(void)  { return g_leftDist; }
float BSP_Ultrasonic_GetRight(void) { return g_rightDist; }
