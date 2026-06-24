#include "bsp_ultrasonic.h"
#include "bsp_systick.h"

#define US_TIMEOUT_MS   60

static float g_frontDist = 999.0f;
static float g_leftDist  = 999.0f;
static float g_rightDist = 999.0f;

static uint32_t g_lastUpdate = 0;

void BSP_Ultrasonic_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* Trig pins: Out_PP, low */
    GPIO_InitStructure.GPIO_Pin  = US_F_TRIG_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(US_F_TRIG_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(US_F_TRIG_PORT, US_F_TRIG_PIN);

    GPIO_InitStructure.GPIO_Pin  = US_L_TRIG_PIN | US_R_TRIG_PIN;
    GPIO_Init(US_L_TRIG_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(US_L_TRIG_PORT, US_L_TRIG_PIN);
    GPIO_ResetBits(US_R_TRIG_PORT, US_R_TRIG_PIN);

    /* Echo pins: IN_FLOATING */
    GPIO_InitStructure.GPIO_Pin  = US_F_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(US_F_ECHO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = US_L_ECHO_PIN | US_R_ECHO_PIN;
    GPIO_Init(US_L_ECHO_PORT, &GPIO_InitStructure);
}

static float Ultrasonic_Measure(GPIO_TypeDef *trigPort, uint16_t trigPin,
                                GPIO_TypeDef *echoPort, uint16_t echoPin)
{
    uint32_t startTick, timeoutTick;
    float dist;

    /* 10us trigger pulse */
    GPIO_ResetBits(trigPort, trigPin);
    for (volatile uint8_t i = 0; i < 5; i++);
    GPIO_SetBits(trigPort, trigPin);
    for (volatile uint8_t i = 0; i < 20; i++);
    GPIO_ResetBits(trigPort, trigPin);

    /* Wait for echo high */
    timeoutTick = BSP_GetTick() + US_TIMEOUT_MS;
    while (GPIO_ReadInputDataBit(echoPort, echoPin) == 0)
    {
        if (BSP_GetTick() > timeoutTick) return 999.0f;
    }

    /* Measure echo pulse width */
    startTick = BSP_GetTick();
    timeoutTick = startTick + US_TIMEOUT_MS;
    while (GPIO_ReadInputDataBit(echoPort, echoPin) == 1)
    {
        if (BSP_GetTick() > timeoutTick) return 999.0f;
    }

    dist = (BSP_GetTick() - startTick) * 0.017f * 100.0f;
    if (dist < 2.0f)  dist = 2.0f;
    if (dist > 400.0f) dist = 400.0f;
    return dist;
}

void BSP_Ultrasonic_Update(void)
{
    uint32_t now = BSP_GetTick();
    if (now - g_lastUpdate < 60) return;
    g_lastUpdate = now;

    g_frontDist = Ultrasonic_Measure(US_F_TRIG_PORT, US_F_TRIG_PIN, US_F_ECHO_PORT, US_F_ECHO_PIN);
    g_leftDist  = Ultrasonic_Measure(US_L_TRIG_PORT, US_L_TRIG_PIN, US_L_ECHO_PORT, US_L_ECHO_PIN);
    g_rightDist = Ultrasonic_Measure(US_R_TRIG_PORT, US_R_TRIG_PIN, US_R_ECHO_PORT, US_R_ECHO_PIN);
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
