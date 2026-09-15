#include "app_oled.h"
#include "bsp/oled/bsp_oled.h"
#include "bsp_mpu6050.h"
#include "bsp_systick.h"
#include "algorithm/algo_filter.h"
#include "app_control.h"
#include "app_state_machine.h"
#include "driver/drv_tb6612.h"
#include "robot_config.h"
#include "Delay.h"

static uint8_t g_initResult, g_forceFrame;
static uint32_t g_frameTick, g_sendTick, g_retryTick;
static char g_line[OLED_TEXT_COLUMNS + 1U];
static uint8_t g_length;

static void Begin(void) { g_length = 0U; g_line[0] = '\0'; }
static void Char(char c)
{
    if (g_length < OLED_TEXT_COLUMNS)
    {
        g_line[g_length++] = c;
        g_line[g_length] = '\0';
    }
}
static void Text(const char *s) { while (*s) Char(*s++); }
static void Number(int32_t value)
{
    char digits[10];
    uint8_t n = 0U;
    uint32_t magnitude;
    if (value < 0) { Char('-'); magnitude = 0U - (uint32_t)value; }
    else magnitude = (uint32_t)value;
    do { digits[n++] = (char)('0' + magnitude % 10U); magnitude /= 10U; } while (magnitude);
    while (n) Char(digits[--n]);
}
static void Angle(float value)
{
    int32_t tenths = (int32_t)(value * 10.0f);
    if (tenths < 0) { Char('-'); tenths = -tenths; }
    else Char('+');
    Number(tenths / 10);
    Char('.');
    Char((char)('0' + tenths % 10));
}
static void Hex(uint8_t value)
{
    static const char digits[] = "0123456789ABCDEF";
    Char(digits[value >> 4]);
    Char(digits[value & 15U]);
}
static void Nibble(uint8_t value)
{
    static const char digits[] = "0123456789ABCDEF";
    Char(digits[value & 15U]);
}
static void Pair(uint8_t row, const char *a, int16_t av,
                 const char *b, int16_t bv, uint8_t fresh)
{
    Begin(); Text(a);
    if (fresh) Number(av); else Text("----");
    Char(' '); Text(b);
    if (fresh) Number(bv); else Text("----");
    BSP_OLED_Text(row, g_line);
}

static void Render(uint32_t now)
{
    const MPU6050_Data_t *data = BSP_MPU6050_GetLatest();
    uint8_t fresh = data->hasSample && data->readStatus == 0U &&
                    (uint32_t)(now - data->sampleTick) < ROBOT_IMU_STALE_MS;
    uint8_t valid = fresh && Algo_Filter_SampleValid() && g_initResult == 0U;
    BSP_OLED_Clear();
    Begin();
    if (g_initResult == 255U) Text("MPU STARTING");
    else if (g_initResult != 0U) { Text("MPU INIT ERR:"); Hex(g_initResult); }
    else if (data->readStatus != 0U) { Text("MPU READ ERR:"); Hex(data->readStatus); }
    else if (!fresh) Text("MPU DATA STALE");
    else if (data->accel[0] == 0 && data->accel[1] == 0 && data->accel[2] == 0)
        Text("MPU ZERO DATA");
    else if (!valid) Text("MPU DATA INVALID");
    else Text("MPU6050 LIVE");
    BSP_OLED_Text(0U, g_line);

    Pair(1U, "AX:", data->accel[0], "AY:", data->accel[1], fresh);
    Pair(2U, "AZ:", data->accel[2], "GX:", data->gyro[0], fresh);
    Pair(3U, "GY:", data->gyro[1], "GZ:", data->gyro[2], fresh);
    Begin(); Text("P:");
    if (valid && Algo_Filter_IsReady()) Angle(Algo_Filter_GetRoll()); else Text("----");
    Text(" R:");
    if (valid && Algo_Filter_IsReady()) Angle(Algo_Filter_GetPitch()); else Text("----");
    BSP_OLED_Text(4U, g_line);
    Begin(); Text("T:");
    if (valid) Angle(Algo_Filter_GetTilt()); else Text("----");
    Text(" CAL:");
    if (Algo_Filter_IsReady() && g_initResult == 0U) Text("OK");
    else Number(Algo_Filter_GetCalibCount());
    BSP_OLED_Text(5U, g_line);
    Begin(); Text("STATE:");
    if (g_initResult != 0U) Text("INIT"); else Text(App_SM_GetStateName());
    BSP_OLED_Text(6U, g_line);
    Begin(); Text("L"); Number(App_Ctrl_GetLeftPwm());
    Text(" R"); Number(App_Ctrl_GetRightPwm());
    Text(" D"); Nibble(DRV_TB6612_GetDirectionOutput());
    Char('/'); Nibble(DRV_TB6612_GetDirectionInput());
    BSP_OLED_Text(7U, g_line);
    BSP_OLED_Present();
}

void App_OLED_Init(void)
{
    g_initResult = 255U;
    g_forceFrame = 1U;
    g_frameTick = g_sendTick = g_retryTick = BSP_GetTick();
    Delay_ms(50U);
    if (!BSP_OLED_Init()) return;
    BSP_OLED_Text(0U, "MPU STARTING");
    BSP_OLED_Text(2U, "KEEP BOAT STILL");
    BSP_OLED_Text(4U, "X UP  Y FORWARD");
    BSP_OLED_Present();
    /* Only the pre-motor startup screen is flushed synchronously. */
    while (BSP_OLED_IsBusy()) BSP_OLED_Task();
}

void App_OLED_ShowBootStage(uint8_t stage)
{
    const char *name;
    if (!BSP_OLED_IsPresent()) return;
    while (BSP_OLED_IsBusy()) BSP_OLED_Task();
    if (stage == 11U) name = "K230 AND ULTRASONIC";
    else if (stage == 20U) name = "I2C SETUP";
    else if (stage == 30U) name = "MPU6050 INIT";
    else name = "UNKNOWN";
    BSP_OLED_Clear();
    BSP_OLED_Text(0U, "BOOT IN PROGRESS");
    BSP_OLED_Text(2U, name);
    BSP_OLED_Text(4U, "MOTORS HELD OFF");
    BSP_OLED_Present();
    while (BSP_OLED_IsBusy()) BSP_OLED_Task();
}

void App_OLED_SetInitResult(uint8_t result)
{
    uint32_t now;
    g_initResult = result;
    g_forceFrame = 1U;
    if (!BSP_OLED_IsPresent()) return;
    while (BSP_OLED_IsBusy()) BSP_OLED_Task();
    now = BSP_GetTick();
    Render(now);
    while (BSP_OLED_IsBusy()) BSP_OLED_Task();
    g_frameTick = now;
    g_forceFrame = 0U;
}

void App_OLED_Task(void)
{
    uint32_t now = BSP_GetTick();
    if (!BSP_OLED_IsPresent())
    {
        if ((uint32_t)(now - g_retryTick) >= 1000U)
        {
            g_retryTick = now;
            if (BSP_OLED_Init()) g_forceFrame = 1U;
        }
        return;
    }
    if (!BSP_OLED_IsBusy() &&
        (g_forceFrame || (uint32_t)(now - g_frameTick) >= OLED_FRAME_MS))
    {
        Render(now);
        g_frameTick = now;
        g_forceFrame = 0U;
    }
    if ((uint32_t)(now - g_sendTick) >= 1U)
    {
        g_sendTick = now;
        BSP_OLED_Task();
    }
}
