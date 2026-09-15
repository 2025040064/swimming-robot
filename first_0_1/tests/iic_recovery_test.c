/* Host model: GPIO levels and a sticky analog-filter BUSY condition. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#define __BSP_IIC_H
#define ENABLE 1
#define DISABLE 0
#define RESET 0
#define BSP_IIC_OK 0U
#define BSP_IIC_ERR_PARAM 1U
#define BSP_IIC_ERR_TIMEOUT 2U
#define BSP_IIC_ERR_NACK 3U
#define BSP_IIC_ERR_BUS 4U
#define BSP_IIC_ERR_BUSY 5U
#define IIC_SCL_PIN 0x100U
#define IIC_SDA_PIN 0x200U
#define IIC_GPIO_PORT 1
#define IIC_GPIO_CLK 1
#define IIC_PERIPH_CLK 1
#define IIC_BUS_SPEED 100000U
#define GPIO_Mode_Out_OD 1
#define GPIO_Mode_AF_OD 2
#define GPIO_Speed_50MHz 3
#define I2C_Mode_I2C 1
#define I2C_DutyCycle_2 2
#define I2C_Ack_Enable 1
#define I2C_AcknowledgedAddress_7bit 1
#define I2C_NACKPosition_Current 0
#define I2C_NACKPosition_Next 1
#define I2C_Direction_Transmitter 0
#define I2C_Direction_Receiver 1
#define I2C_FLAG_BUSY 1U
#define I2C_FLAG_AF 2U
#define I2C_FLAG_BERR 3U
#define I2C_FLAG_ARLO 4U
#define I2C_FLAG_OVR 5U
#define I2C_FLAG_SB 6U
#define I2C_FLAG_ADDR 7U
#define I2C_FLAG_TXE 8U
#define I2C_FLAG_BTF 9U
#define I2C_FLAG_RXNE 10U

typedef struct { volatile uint16_t SR1, SR2; } Peripheral;
static Peripheral peripheral;
#define IIC_PERIPH (&peripheral)
typedef struct { uint16_t GPIO_Pin; int GPIO_Mode, GPIO_Speed; } GPIO_InitTypeDef;
typedef struct {
    int I2C_Mode, I2C_DutyCycle, I2C_OwnAddress1, I2C_Ack;
    int I2C_AcknowledgedAddress, I2C_ClockSpeed;
} I2C_InitTypeDef;

static uint16_t outputs, heldLow;
static int busy, permanentlyBusy, enabled, mode, resetActive, resetCount;
static unsigned int transitions, reads;
static uint16_t levels(void) { return outputs & (uint16_t)~heldLow; }
static void GPIO_Init(int port, GPIO_InitTypeDef *gpio)
{
    (void)port;
    assert(gpio->GPIO_Mode == GPIO_Mode_Out_OD || gpio->GPIO_Mode == GPIO_Mode_AF_OD);
    if (gpio->GPIO_Mode == GPIO_Mode_Out_OD) assert(!enabled);
    mode = gpio->GPIO_Mode;
}
static void GPIO_SetBits(int port, uint16_t pins)
{
    uint16_t before = levels();
    (void)port;
    outputs |= pins;
    if (mode == GPIO_Mode_Out_OD) transitions |= (unsigned int)((levels() & ~before) >> 8);
}
static void GPIO_ResetBits(int port, uint16_t pins)
{
    uint16_t before = levels();
    (void)port;
    assert(mode == GPIO_Mode_Out_OD && !enabled);
    outputs &= (uint16_t)~pins;
    transitions |= (unsigned int)((before & ~levels()) >> 6);
}
static uint16_t GPIO_ReadInputData(int port)
{
    (void)port;
    assert(++reads < 2000000U);
    return levels();
}
void Delay_us(uint32_t us) { assert(us >= 5U); }
static void RCC_APB2PeriphClockCmd(int clock, int on) { (void)clock; (void)on; }
static void RCC_APB1PeriphClockCmd(int clock, int on) { (void)clock; (void)on; }
static void I2C_DeInit(Peripheral *p) { (void)p; enabled = 0; }
static void I2C_Init(Peripheral *p, I2C_InitTypeDef *cfg) { (void)p; (void)cfg; }
static void I2C_Cmd(Peripheral *p, int on) { (void)p; enabled = on; }
static void I2C_AcknowledgeConfig(Peripheral *p, int on) { (void)p; (void)on; }
static void I2C_NACKPositionConfig(Peripheral *p, int pos) { (void)p; (void)pos; }
static void I2C_SoftwareResetCmd(Peripheral *p, int on)
{
    (void)p;
    assert(mode == GPIO_Mode_AF_OD && !enabled);
    if (on) { resetActive = 1; resetCount++; }
    else {
        assert(resetActive);
        resetActive = 0;
        if (transitions == 15U && !heldLow && !permanentlyBusy) busy = 0;
    }
}
static int I2C_GetFlagStatus(Peripheral *p, uint32_t flag)
{
    (void)p;
    assert(++reads < 2000000U);
    return flag == I2C_FLAG_BUSY ? busy : 0;
}
static void I2C_GenerateSTART(Peripheral *p, int on) { (void)p; (void)on; }
static void I2C_GenerateSTOP(Peripheral *p, int on) { (void)p; (void)on; }
static void I2C_Send7bitAddress(Peripheral *p, uint8_t addr, uint8_t dir)
{ (void)p; (void)addr; (void)dir; }
static void I2C_SendData(Peripheral *p, uint8_t data) { (void)p; (void)data; }
static uint8_t I2C_ReceiveData(Peripheral *p) { (void)p; return 0; }

#include "../User/bsp/bsp_iic.c"

static void resetModel(int stuck, uint16_t low, int permanent)
{
    outputs = IIC_SCL_PIN | IIC_SDA_PIN;
    heldLow = low;
    busy = stuck;
    permanentlyBusy = permanent;
    enabled = 0;
    mode = GPIO_Mode_AF_OD;
    resetActive = resetCount = 0;
    transitions = reads = 0U;
}
static void restored(void)
{
    assert(mode == GPIO_Mode_AF_OD && enabled);
    assert(outputs == (IIC_SCL_PIN | IIC_SDA_PIN));
}
int main(void)
{
    resetModel(0, 0U, 0);
    BSP_IIC_Init();
    assert(g_debugIicRecovery == 0U && resetCount == 0);
    restored();

    resetModel(1, 0U, 0);
    BSP_IIC_Init();
    assert(!busy && g_debugIicRecovery == 1U && g_debugIicLines == 3U);
    assert(transitions == 15U && resetCount == 1);
    restored();

    resetModel(0, 0U, 0);
    BSP_IIC_Init();
    busy = 1;
    assert(IIC_WaitBusFree() == BSP_IIC_OK);
    assert(!busy && resetCount == 1);
    restored();

    resetModel(1, IIC_SDA_PIN, 0);
    BSP_IIC_Init();
    assert(g_debugIicRecovery == 2U && g_debugIicLines == 1U);
    assert(busy && resetCount == 1);
    restored();
    assert(BSP_IIC_WriteAddr(0x68U, 0U, 0U) == BSP_IIC_ERR_BUSY);

    resetModel(1, IIC_SCL_PIN, 0);
    BSP_IIC_Init();
    assert(g_debugIicRecovery == 2U && g_debugIicLines == 2U);
    restored();

    resetModel(1, 0U, 1);
    BSP_IIC_Init();
    assert(g_debugIicRecovery == 3U && resetCount == 1 && busy);
    restored();
    assert(IIC_WaitBusFree() == BSP_IIC_ERR_BUSY);
    restored();
    puts("PASS: healthy bus, sticky BUSY, transaction recovery, held-low SCL/SDA, permanent BUSY, bounded failure and pin restoration");
    return 0;
}
