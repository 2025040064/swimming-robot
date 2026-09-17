/* Shared-I2C model: address selection, retry/recovery and display writes. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bsp_iic.h"

typedef struct
{
    uint8_t address;
    uint8_t prefix;
    uint8_t count;
    uint8_t data[32];
} Transfer;

static Transfer transfers[160];
static unsigned int transferCount, recoverCount;
static uint8_t attached, deviceAddress, busError;

uint8_t BSP_IIC_WriteBuffer(uint8_t addr7, uint8_t prefix,
                            const uint8_t *buf, uint8_t len)
{
    Transfer *transfer;
    if (busError) return BSP_IIC_ERR_BUS;
    if (!attached || addr7 != deviceAddress) return BSP_IIC_ERR_NACK;
    assert(buf != 0 && len > 0U && len <= sizeof(transfers[0].data));
    assert(transferCount < sizeof(transfers) / sizeof(transfers[0]));
    transfer = &transfers[transferCount++];
    transfer->address = addr7;
    transfer->prefix = prefix;
    transfer->count = len;
    memcpy(transfer->data, buf, len);
    return BSP_IIC_OK;
}

void BSP_IIC_Recover(void) { recoverCount++; }
void Delay_us(uint32_t us) { assert(us == 50U); }

#include "../User/bsp/oled/bsp_oled.c"

static void resetModel(uint8_t address, int connected, int failedBus)
{
    transferCount = recoverCount = 0U;
    attached = (uint8_t)connected;
    deviceAddress = address;
    busError = (uint8_t)failedBus;
    memset(transfers, 0, sizeof(transfers));
}
int main(void)
{
    unsigned int i, first;

    resetModel(0x3CU, 1, 0);
    assert(BSP_OLED_Init());
    assert(g_debugOledAddress == 0x3CU && g_debugOledStatus == 1U);
    assert(transferCount == 1U);
    assert(transfers[0].address == 0x3CU);
    assert(transfers[0].prefix == 0x00U && transfers[0].count == 25U);

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
        assert(transfers[first + 2U * i].prefix == 0x00U);
        assert(transfers[first + 2U * i].count == 3U);
        assert(transfers[first + 2U * i + 1U].prefix == 0x40U);
        assert(transfers[first + 2U * i + 1U].count == 16U);
    }
    assert(!BSP_OLED_IsBusy());
    assert(transfers[first].data[0] == 0xB0U);
    assert(transfers[first + 126U].data[0] == 0xB7U);

    resetModel(0x3DU, 1, 0);
    assert(BSP_OLED_Init());
    assert(g_debugOledAddress == 0x3DU && g_debugOledStatus == 1U);
    assert(recoverCount == 0U);

    resetModel(0x3CU, 0, 0);
    assert(!BSP_OLED_Init() && g_debugOledStatus == 4U);
    assert(!BSP_OLED_IsBusy() && !BSP_OLED_IsPresent());
    assert(recoverCount == 0U);

    resetModel(0x3CU, 1, 1);
    assert(!BSP_OLED_Init() && g_debugOledStatus == 3U);
    assert(transferCount == 0U);
    assert(recoverCount == 2U);

    resetModel(0x3CU, 1, 0);
    assert(BSP_OLED_Init());
    BSP_OLED_Present();
    attached = 0;
    BSP_OLED_Task();
    assert(!BSP_OLED_IsBusy() && !BSP_OLED_IsPresent());
    assert(g_debugOledNoAckCount > 0U);
    assert(recoverCount == 0U);
    puts("PASS: OLED shared I2C address selection, recovery, page chunks and disconnect handling");
    return 0;
}
