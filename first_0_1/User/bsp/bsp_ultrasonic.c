/**
 * Ultrasonic driver for 3x AJ-SRP04M sensors, using microsecond-level timing.
 *
 * Timing: TIM3 runs as a free-running 1MHz (1 us/tick) counter. The Echo pins
 * (PA7/EXTI7, PB6/EXTI6, PB8/EXTI8) trigger EXTI interrupts on BOTH edges:
 *   - rising edge  -> record TIM3->CNT as the echo start
 *   - falling edge -> delta_us * 0.017 = distance in cm
 *
 * A round-robin scheduler fires one sensor at a time (one Trig pulse per
 * ~60 ms) so the three sensors never echo over each other. Each sensor has a
 * 30 ms timeout: if no echo arrives, it is marked invalid and reads 999.0 cm.
 *
 * A reading is "valid" only if the last echo succeeded AND it is not stale
 * (see BSP_Ultrasonic_IsValid). 999.0 cm is "no echo", not "clear ahead".
 */

#include "bsp_ultrasonic.h"
#include "bsp_systick.h"

/* TIM3 as the microsecond timebase (72 MHz / (71+1) = 1 MHz). */
#define US_TIM          TIM3
#define US_TIM_CLK      RCC_APB1Periph_TIM3
#define US_TIM_PSC      71

typedef struct
{
    GPIO_TypeDef *trigPort;
    uint16_t      trigPin;
    GPIO_TypeDef *echoPort;
    uint16_t      echoPin;

    volatile uint8_t  measuring;       /* 1 = waiting for this sensor's echo */
    volatile uint8_t  echoHigh;        /* 1 = echo currently high (measuring pulse) */
    volatile uint16_t echoStartUs;     /* TIM3->CNT at the rising edge */
    volatile uint32_t trigTick;        /* BSP_GetTick() when Trig was pulsed */
    volatile uint32_t lastUpdateTick;  /* BSP_GetTick() at last successful echo */

    volatile float    distance;        /* cm */
    volatile uint8_t  valid;           /* 1 = last echo succeeded */
} US_Sensor_t;

static US_Sensor_t g_sensors[3];
static volatile uint8_t  g_sensorIdx = 0;
static volatile uint32_t g_nextTrigTick = 0;

/* ---- microsecond busy delay based on the TIM3 free-running counter ---- */
static void delay_us(uint16_t us)
{
    uint16_t start = US_TIM->CNT;
    while ((uint16_t)(US_TIM->CNT - start) < us) { }
}

/* ---- 10 us Trigger pulse ---- */
static void TriggerPulse(US_Sensor_t *s)
{
    GPIO_ResetBits(s->trigPort, s->trigPin);
    delay_us(2);
    GPIO_SetBits(s->trigPort, s->trigPin);
    delay_us(10);
    GPIO_ResetBits(s->trigPort, s->trigPin);
}

/* ---- Start a measurement on one sensor ---- */
static void Ultrasonic_Trigger(US_Sensor_t *s, uint32_t now)
{
    s->measuring = 1;
    s->echoHigh  = 0;
    s->trigTick  = now;
    /* keep the previous distance/valid until a new echo or a timeout */
    TriggerPulse(s);
}

/* ---- Process one Echo edge (called from EXTI9_5_IRQHandler) ---- */
static void US_CaptureEdge(US_Sensor_t *s, uint32_t now)
{
    uint8_t level = GPIO_ReadInputDataBit(s->echoPort, s->echoPin);

    if (level)   /* rising edge: start the pulse width measurement */
    {
        s->echoStartUs = US_TIM->CNT;
        s->echoHigh = 1;
    }
    else if (s->echoHigh)   /* falling edge after a rising edge: pulse done */
    {
        uint16_t elapsed = (uint16_t)(US_TIM->CNT - s->echoStartUs);
        float d = (float)elapsed * 0.017f;   /* cm (340 m/s, round trip) */

        if (d < US_MIN_DIST_CM) d = US_MIN_DIST_CM;
        if (d > US_MAX_DIST_CM) d = US_MAX_DIST_CM;

        s->distance       = d;
        s->valid          = 1;
        s->lastUpdateTick = now;
        s->measuring      = 0;
        s->echoHigh       = 0;
    }
}

/* ---- Initialize hardware GPIOs, TIM3 timebase and EXTI ---- */
void BSP_Ultrasonic_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(US_TIM_CLK, ENABLE);

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

    /* TIM3: 1MHz free-running counter (no interrupt, no output). */
    TIM_TimeBaseStructure.TIM_Prescaler     = US_TIM_PSC;
    TIM_TimeBaseStructure.TIM_Period        = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(US_TIM, &TIM_TimeBaseStructure);
    TIM_Cmd(US_TIM, ENABLE);

    /* Map echo pins to EXTI lines, trigger on both edges. */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource7);   /* front PA7 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6);   /* left  PB6 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);   /* right PB8 */

    EXTI_InitStructure.EXTI_Line    = EXTI_Line6 | EXTI_Line7 | EXTI_Line8;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel                   = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Init sensor control blocks */
    g_sensors[0].trigPort = US_F_TRIG_PORT; g_sensors[0].trigPin = US_F_TRIG_PIN;
    g_sensors[0].echoPort = US_F_ECHO_PORT; g_sensors[0].echoPin = US_F_ECHO_PIN;
    g_sensors[0].distance = US_NO_ECHO;

    g_sensors[1].trigPort = US_L_TRIG_PORT; g_sensors[1].trigPin = US_L_TRIG_PIN;
    g_sensors[1].echoPort = US_L_ECHO_PORT; g_sensors[1].echoPin = US_L_ECHO_PIN;
    g_sensors[1].distance = US_NO_ECHO;

    g_sensors[2].trigPort = US_R_TRIG_PORT; g_sensors[2].trigPin = US_R_TRIG_PIN;
    g_sensors[2].echoPort = US_R_ECHO_PORT; g_sensors[2].echoPin = US_R_ECHO_PIN;
    g_sensors[2].distance = US_NO_ECHO;

    g_sensorIdx    = 0;
    g_nextTrigTick = 0;
}

/* ---- Called at ~10ms from the task scheduler ---- */
void BSP_Ultrasonic_Update(void)
{
    uint32_t now = BSP_GetTick();
    uint8_t i;

    /* Timeout: invalidate any sensor whose echo never came (or never ended). */
    for (i = 0; i < 3; i++)
    {
        US_Sensor_t *s = &g_sensors[i];
        if (s->measuring && (now - s->trigTick) > US_ECHO_TIMEOUT_MS)
        {
            s->measuring = 0;
            s->echoHigh  = 0;
            s->valid     = 0;
            s->distance  = US_NO_ECHO;
        }
    }

    /* Round-robin: fire the next sensor every US_TRIG_INTERVAL_MS. */
    if (now - g_nextTrigTick < US_TRIG_INTERVAL_MS)
        return;

    g_nextTrigTick = now;
    Ultrasonic_Trigger(&g_sensors[g_sensorIdx], now);

    g_sensorIdx++;
    if (g_sensorIdx >= 3) g_sensorIdx = 0;
}

/* ---- EXTI9_5 shared IRQ: PA7(EXTI7) / PB6(EXTI6) / PB8(EXTI8) ---- */
void EXTI9_5_IRQHandler(void)
{
    uint32_t now = BSP_GetTick();

    if (EXTI_GetITStatus(EXTI_Line7) != RESET)   /* front PA7 */
    {
        EXTI_ClearITPendingBit(EXTI_Line7);
        US_CaptureEdge(&g_sensors[0], now);
    }
    if (EXTI_GetITStatus(EXTI_Line6) != RESET)   /* left PB6 */
    {
        EXTI_ClearITPendingBit(EXTI_Line6);
        US_CaptureEdge(&g_sensors[1], now);
    }
    if (EXTI_GetITStatus(EXTI_Line8) != RESET)   /* right PB8 */
    {
        EXTI_ClearITPendingBit(EXTI_Line8);
        US_CaptureEdge(&g_sensors[2], now);
    }
}

float BSP_Ultrasonic_GetDistance(uint8_t sensor)
{
    if (sensor < 3)
        return g_sensors[sensor].distance;
    return US_NO_ECHO;
}

float BSP_Ultrasonic_GetFront(void) { return g_sensors[0].distance; }
float BSP_Ultrasonic_GetLeft(void)  { return g_sensors[1].distance; }
float BSP_Ultrasonic_GetRight(void) { return g_sensors[2].distance; }

uint8_t BSP_Ultrasonic_IsValid(uint8_t sensor)
{
    if (sensor >= 3)
        return 0;
    return g_sensors[sensor].valid &&
           ((BSP_GetTick() - g_sensors[sensor].lastUpdateTick) < US_STALE_MS);
}

uint32_t BSP_Ultrasonic_GetAgeMs(uint8_t sensor)
{
    if (sensor >= 3)
        return 0xFFFFFFFF;
    return BSP_GetTick() - g_sensors[sensor].lastUpdateTick;
}
