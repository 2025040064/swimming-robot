#include "bsp_oled.h"
#include "Delay.h"
#include <string.h>

/* Independent software I2C; each task call sends at most 16 display bytes. */
static uint8_t g_frame[1024];
static uint16_t g_position;
static uint8_t g_busy, g_present, g_address;
/* 0=not initialized, 1=OK, 2=no ACK, 3=bus held low. */
volatile uint32_t g_debugOledStatus;
volatile uint32_t g_debugOledAddress;
volatile uint32_t g_debugOledNoAckCount;

static const uint8_t g_digits[10][5] = {
    {0x3E,0x51,0x49,0x45,0x3E},{0,0x42,0x7F,0x40,0},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30},{1,0x71,9,5,3},
    {0x36,0x49,0x49,0x49,0x36},{6,0x49,0x49,0x29,0x1E}
};
static const uint8_t g_letters[26][5] = {
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41},{0x7F,9,9,9,1},
    {0x3E,0x41,0x49,0x49,0x7A},{0x7F,8,8,8,0x7F},
    {0,0x41,0x7F,0x41,0},{0x20,0x40,0x41,0x3F,1},
    {0x7F,8,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,2,0x0C,2,0x7F},{0x7F,4,8,0x10,0x7F},
    {0x3E,0x41,0x41,0x41,0x3E},{0x7F,9,9,9,6},
    {0x3E,0x41,0x51,0x21,0x5E},{0x7F,9,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{1,1,0x7F,1,1},
    {0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},
    {0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,8,0x14,0x63},
    {7,8,0x70,8,7},{0x61,0x51,0x49,0x45,0x43}
};

static uint8_t Glyph(char ch, uint8_t col)
{
    if (ch >= '0' && ch <= '9') return g_digits[ch - '0'][col];
    if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');
    if (ch >= 'A' && ch <= 'Z') return g_letters[ch - 'A'][col];
    switch (ch)
    {
    case ' ': return 0U;
    case '-': return 8U;
    case '_': return 0x40U;
    case '+': return col == 2U ? 0x3EU : 8U;
    case '.': return col == 2U ? 0x60U : 0U;
    case ':': return col == 2U ? 0x36U : 0U;
    case '/': return (uint8_t)(0x40U >> col);
    default: return (col == 0U || col == 4U) ? 0x7FU : 0x41U;
    }
}

static uint8_t ClockHigh(void)
{
    uint16_t timeout = 200U;
    GPIO_SetBits(OLED_GPIO_PORT, OLED_SCK_PIN);
    while (!GPIO_ReadInputDataBit(OLED_GPIO_PORT, OLED_SCK_PIN))
    {
        if (--timeout == 0U)
        {
            g_debugOledStatus = 3U;
            return 0U;
        }
    }
    Delay_us(OLED_I2C_DELAY_US);
    return 1U;
}

static void Stop(void)
{
    GPIO_ResetBits(OLED_GPIO_PORT, OLED_SCK_PIN | OLED_SDA_PIN);
    Delay_us(OLED_I2C_DELAY_US);
    (void)ClockHigh();
    GPIO_SetBits(OLED_GPIO_PORT, OLED_SDA_PIN);
    Delay_us(OLED_I2C_DELAY_US);
}

static uint8_t Start(void)
{
    GPIO_SetBits(OLED_GPIO_PORT, OLED_SDA_PIN);
    if (!ClockHigh()) return 0U;
    if (!GPIO_ReadInputDataBit(OLED_GPIO_PORT, OLED_SDA_PIN))
    {
        g_debugOledStatus = 3U;
        return 0U;
    }
    GPIO_ResetBits(OLED_GPIO_PORT, OLED_SDA_PIN);
    Delay_us(OLED_I2C_DELAY_US);
    GPIO_ResetBits(OLED_GPIO_PORT, OLED_SCK_PIN);
    return 1U;
}

static uint8_t SendByte(uint8_t value)
{
    uint8_t i, ack;
    for (i = 0U; i < 8U; i++)
    {
        GPIO_WriteBit(OLED_GPIO_PORT, OLED_SDA_PIN,
                      (value & 0x80U) ? Bit_SET : Bit_RESET);
        Delay_us(OLED_I2C_DELAY_US);
        if (!ClockHigh()) return 0U;
        GPIO_ResetBits(OLED_GPIO_PORT, OLED_SCK_PIN);
        value <<= 1;
    }
    GPIO_SetBits(OLED_GPIO_PORT, OLED_SDA_PIN);
    Delay_us(OLED_I2C_DELAY_US);
    if (!ClockHigh()) return 0U;
    ack = !GPIO_ReadInputDataBit(OLED_GPIO_PORT, OLED_SDA_PIN);
    GPIO_ResetBits(OLED_GPIO_PORT, OLED_SCK_PIN);
    if (!ack)
    {
        g_debugOledStatus = 4U;
        g_debugOledNoAckCount++;
    }
    return 1U;
}

static uint8_t WriteAttempt(uint8_t control, const uint8_t *data, uint8_t count)
{
    uint8_t i;
    if (!Start()) goto failed;
    if (!SendByte((uint8_t)(g_address << 1)) || !SendByte(control)) goto failed;
    for (i = 0U; i < count; i++)
        if (!SendByte(data[i])) goto failed;
    Stop();
    return 1U;
failed:
    Stop();
    return 0U;
}

static uint8_t Write(uint8_t control, const uint8_t *data, uint8_t count)
{
    if (WriteAttempt(control, data, count)) return 1U;
    Delay_us(OLED_RETRY_DELAY_US);
    if (WriteAttempt(control, data, count))
    {
        g_debugOledStatus = g_debugOledNoAckCount ? 4U : 1U;
        return 1U;
    }
    g_present = g_busy = 0U;
    return 0U;
}

uint8_t BSP_OLED_Init(void)
{
    GPIO_InitTypeDef gpio;
    static const uint8_t setup[] = {
        0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,0xA1,0xC8,
        0xDA,0x12,0x81,0x8F,0xD9,0xF1,0xDB,0x40,0xA4,0xA6,
        0x8D,0x14,0x20,0x02,0x2E,0xAF
    };
    g_present = g_busy = 0U;
    g_debugOledStatus = g_debugOledAddress = 0U;
    g_debugOledNoAckCount = 0U;
    RCC_APB2PeriphClockCmd(OLED_GPIO_CLK, ENABLE);
    GPIO_SetBits(OLED_GPIO_PORT, OLED_SCK_PIN | OLED_SDA_PIN);
    gpio.GPIO_Pin = OLED_SCK_PIN | OLED_SDA_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OLED_GPIO_PORT, &gpio);
    g_debugOledNoAckCount = 0U;
    g_address = 0x3CU;
    g_debugOledAddress = g_address;
    if (!(Write(0x00U, setup, (uint8_t)sizeof(setup)) && g_debugOledNoAckCount == 0U))
    {
        /* 0x3C did not acknowledge; retry at 0x3D (SA0 tied high on some modules). */
        g_address = 0x3DU;
        g_debugOledAddress = g_address;
        g_debugOledNoAckCount = 0U;
        if (!Write(0x00U, setup, (uint8_t)sizeof(setup))) return 0U;
    }
    if (g_debugOledNoAckCount != 0U) return 0U;
    g_present = 1U;
    g_debugOledStatus = 1U;
    BSP_OLED_Clear();
    return 1U;
}

uint8_t BSP_OLED_IsPresent(void) { return g_present; }
uint8_t BSP_OLED_IsBusy(void) { return g_busy; }
void BSP_OLED_Clear(void) { if (!g_busy) memset(g_frame, 0, sizeof(g_frame)); }

void BSP_OLED_Text(uint8_t row, const char *text)
{
    uint8_t x, col;
    if (row >= 8U || text == 0 || g_busy) return;
    for (x = 0U; x < OLED_TEXT_COLUMNS && *text; x++, text++)
        for (col = 0U; col < 5U; col++)
            g_frame[(uint16_t)row * 128U + x * 6U + col] = Glyph(*text, col);
}

void BSP_OLED_Present(void)
{
    if (g_present && !g_busy) { g_position = 0U; g_busy = 1U; }
}

void BSP_OLED_Task(void)
{
    uint8_t commands[3];
    uint8_t column;
    if (!g_present || !g_busy) return;
    column = (uint8_t)((g_position & 127U) + OLED_COLUMN_OFFSET);
    commands[0] = (uint8_t)(0xB0U | (g_position >> 7));
    commands[1] = (uint8_t)(column & 15U);
    commands[2] = (uint8_t)(0x10U | (column >> 4));
    if (!Write(0x00U, commands, 3U)) return;
    if (!Write(0x40U, g_frame + g_position, 16U)) return;
    g_position += 16U;
    if (g_position >= sizeof(g_frame)) g_busy = 0U;
}
