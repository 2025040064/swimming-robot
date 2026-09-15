/* Bit-level slave model: ACK, address selection, clock hold and display writes. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define GPIOB 1
#define RCC_APB2Periph_GPIOB 1
#define GPIO_Pin_10 0x400U
#define GPIO_Pin_11 0x800U
#define GPIO_Mode_Out_OD 1
#define GPIO_Speed_50MHz 3
#define ENABLE 1
#define Bit_SET 1
#define Bit_RESET 0
typedef struct { uint16_t GPIO_Pin; int GPIO_Mode, GPIO_Speed; } GPIO_InitTypeDef;
static uint16_t pins;
static uint8_t active, bit, byteValue, selected, deviceAddress;
static uint8_t transfers[160][64];
static unsigned int lengths[160], transferCount, clockReads;
static int clockHeld, attached;

static void update(uint16_t next)
{
    uint16_t old = pins;
    pins = next;
    if ((old & GPIO_Pin_10) && (next & GPIO_Pin_10))
    {
        if ((old & GPIO_Pin_11) && !(next & GPIO_Pin_11))
        {
            assert(transferCount < 160U);
            active = 1U; bit = byteValue = selected = 0U;
            lengths[transferCount] = 0U;
        }
        else if (!(old & GPIO_Pin_11) && (next & GPIO_Pin_11) && active)
        {
            active = 0U; transferCount++;
        }
    }
    if (active && !(old & GPIO_Pin_10) && (next & GPIO_Pin_10))
    {
        if (bit < 8U)
        {
            byteValue = (uint8_t)((byteValue << 1) | ((next & GPIO_Pin_11) ? 1U : 0U));
            bit++;
            if (bit == 8U)
            {
                unsigned int len = lengths[transferCount];
                assert(len < 64U);
                transfers[transferCount][len] = byteValue;
                lengths[transferCount]++;
                if (!len) selected = attached && byteValue == (uint8_t)(deviceAddress << 1);
            }
        }
        else bit = 9U;
    }
    if (active && (old & GPIO_Pin_10) && !(next & GPIO_Pin_10) && bit == 9U)
        bit = byteValue = 0U;
}
static void GPIO_SetBits(int port, uint16_t mask)
{ (void)port; assert(!(mask & ~0xC00U)); update((uint16_t)(pins | mask)); }
static void GPIO_ResetBits(int port, uint16_t mask)
{ (void)port; assert(!(mask & ~0xC00U)); update((uint16_t)(pins & ~mask)); }
static void GPIO_WriteBit(int port, uint16_t mask, int value)
{ if (value) GPIO_SetBits(port, mask); else GPIO_ResetBits(port, mask); }
static int GPIO_ReadInputDataBit(int port, uint16_t mask)
{
    (void)port;
    if (mask == GPIO_Pin_10)
    {
        assert(++clockReads < 100000U);
        return !clockHeld && (pins & mask) != 0U;
    }
    if (active && bit == 9U) return !selected;
    return (pins & mask) != 0U;
}
static void GPIO_Init(int port, GPIO_InitTypeDef *gpio)
{ (void)port; assert(gpio->GPIO_Pin == 0xC00U && gpio->GPIO_Mode == GPIO_Mode_Out_OD); }
static void RCC_APB2PeriphClockCmd(int clock, int on) { (void)clock; assert(on); }
void Delay_us(uint32_t us) { assert(us >= 5U); }

#include "../User/bsp/oled/bsp_oled.c"

static void resetModel(uint8_t address, int connected, int held)
{
    pins = 0xC00U;
    active = bit = byteValue = selected = 0U;
    transferCount = clockReads = 0U;
    attached = connected; clockHeld = held; deviceAddress = address;
    memset(transfers, 0, sizeof(transfers));
    memset(lengths, 0, sizeof(lengths));
}
int main(void)
{
    unsigned int i, first;
    resetModel(0x3CU, 1, 0);
    assert(BSP_OLED_Init());
    assert(g_debugOledAddress == 0x3CU && g_debugOledStatus == 1U);
    assert(transferCount == 1U);
    BSP_OLED_Clear();
    BSP_OLED_Text(0U, "MPU6050 LIVE");
    assert(g_frame[0] != 0U);
    BSP_OLED_Text(8U, "OUTSIDE");
    first = transferCount;
    BSP_OLED_Present();
    for (i = 0U; i < 64U; i++)
    {
        assert(BSP_OLED_IsBusy());
        BSP_OLED_Task();
        assert(transferCount == first + 2U * (i + 1U));
        assert(lengths[first + 2U * i] == 5U);
        assert(lengths[first + 2U * i + 1U] == 18U);
        assert(transfers[first + 2U * i + 1U][1] == 0x40U);
    }
    assert(!BSP_OLED_IsBusy());
    assert(transfers[first][2] == 0xB0U);
    assert(transfers[first + 126U][2] == 0xB7U);

    resetModel(0x3DU, 1, 0);
    assert(BSP_OLED_Init());
    assert(g_debugOledAddress == 0x3CU && g_debugOledStatus == 4U);

    resetModel(0x3CU, 0, 0);
    assert(BSP_OLED_Init() && g_debugOledStatus == 4U);
    assert(!BSP_OLED_IsBusy() && BSP_OLED_IsPresent());

    resetModel(0x3CU, 1, 1);
    assert(!BSP_OLED_Init() && g_debugOledStatus == 3U);
    assert(clockReads < 1000U && pins == 0xC00U);

    resetModel(0x3CU, 1, 0);
    assert(BSP_OLED_Init());
    BSP_OLED_Present();
    attached = 0;
    BSP_OLED_Task();
    assert(BSP_OLED_IsBusy() && BSP_OLED_IsPresent());
    assert(g_debugOledNoAckCount > 0U);
    puts("PASS: OLED fixed address, optional ACK, 16-byte page chunks, text bounds and bounded clock hold");
    return 0;
}
