/*
 * Hardware I2C1 master for the MPU6050 bus. A future I2C sensor may share it.
 *
 * BSP_Board_Init() applies GPIO_Remap_I2C1 before this module configures
 * PB8/PB9 as AF open-drain. Transactions use unshifted 7-bit addresses;
 * this module is the only place that converts them to the SPL address format.
 */

#include "bsp_iic.h"
#include "Delay.h"

#define IIC_TIMEOUT_LOOPS       100000UL

/* Last recovery: 0=not attempted, 1=OK, 2=GPIO level failed, 3=BUSY remained.
 * Lines at recovery entry: bit 0=SCL high, bit 1=SDA high. */
volatile uint32_t g_debugIicRecovery;
volatile uint32_t g_debugIicLines;

static void IIC_ConfigPeripheral(void)
{
    I2C_InitTypeDef I2C_InitStructure;

    I2C_DeInit(IIC_PERIPH);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = IIC_BUS_SPEED;
    I2C_Init(IIC_PERIPH, &I2C_InitStructure);
    I2C_AcknowledgeConfig(IIC_PERIPH, ENABLE);
    I2C_NACKPositionConfig(IIC_PERIPH, I2C_NACKPosition_Current);
    I2C_Cmd(IIC_PERIPH, ENABLE);
}

static uint8_t IIC_WaitPins(uint16_t pins, uint16_t levels)
{
    uint32_t timeout = IIC_TIMEOUT_LOOPS;
    while ((GPIO_ReadInputData(IIC_GPIO_PORT) & pins) != levels)
    {
        if (--timeout == 0U) return BSP_IIC_ERR_BUSY;
    }
    return BSP_IIC_OK;
}

/* STM32F103x8/B ES096 Rev 15, section 2.8.7.
 * Single-master bus: force both filter inputs through verified transitions.
 * Open-drain outputs only release the high level; a held-low line fails. */
static uint8_t IIC_UnlockBus(void)
{
    GPIO_InitTypeDef gpio;
    uint16_t lines;
    uint8_t result = BSP_IIC_ERR_BUSY;
    uint32_t timeout;

    I2C_Cmd(IIC_PERIPH, DISABLE);
    GPIO_SetBits(IIC_GPIO_PORT, IIC_SCL_PIN | IIC_SDA_PIN);
    gpio.GPIO_Pin = IIC_SCL_PIN | IIC_SDA_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IIC_GPIO_PORT, &gpio);
    Delay_us(5U);
    lines = GPIO_ReadInputData(IIC_GPIO_PORT);
    g_debugIicLines = ((lines & IIC_SCL_PIN) ? 1U : 0U) |
                     ((lines & IIC_SDA_PIN) ? 2U : 0U);
    g_debugIicRecovery = 2U;
    if (IIC_WaitPins(gpio.GPIO_Pin, gpio.GPIO_Pin) != BSP_IIC_OK)
        goto restore;

    GPIO_ResetBits(IIC_GPIO_PORT, IIC_SDA_PIN);
    if (IIC_WaitPins(IIC_SDA_PIN, 0U) != BSP_IIC_OK) goto restore;
    Delay_us(5U);
    GPIO_ResetBits(IIC_GPIO_PORT, IIC_SCL_PIN);
    if (IIC_WaitPins(IIC_SCL_PIN, 0U) != BSP_IIC_OK) goto restore;
    Delay_us(5U);
    GPIO_SetBits(IIC_GPIO_PORT, IIC_SCL_PIN);
    if (IIC_WaitPins(IIC_SCL_PIN, IIC_SCL_PIN) != BSP_IIC_OK) goto restore;
    Delay_us(5U);
    GPIO_SetBits(IIC_GPIO_PORT, IIC_SDA_PIN);
    if (IIC_WaitPins(IIC_SDA_PIN, IIC_SDA_PIN) != BSP_IIC_OK) goto restore;
    Delay_us(5U);
    result = BSP_IIC_OK;

restore:
    GPIO_SetBits(IIC_GPIO_PORT, IIC_SCL_PIN | IIC_SDA_PIN);
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(IIC_GPIO_PORT, &gpio);
    I2C_SoftwareResetCmd(IIC_PERIPH, ENABLE);
    I2C_SoftwareResetCmd(IIC_PERIPH, DISABLE);
    IIC_ConfigPeripheral();
    if (result != BSP_IIC_OK) return result;

    timeout = IIC_TIMEOUT_LOOPS;
    while (I2C_GetFlagStatus(IIC_PERIPH, I2C_FLAG_BUSY) != RESET)
    {
        if (--timeout == 0U)
        {
            g_debugIicRecovery = 3U;
            return BSP_IIC_ERR_BUSY;
        }
    }
    g_debugIicRecovery = 1U;
    return BSP_IIC_OK;
}

static uint8_t IIC_GetError(void)
{
    if (I2C_GetFlagStatus(IIC_PERIPH, I2C_FLAG_AF) != RESET)
        return BSP_IIC_ERR_NACK;
    if ((I2C_GetFlagStatus(IIC_PERIPH, I2C_FLAG_BERR) != RESET) ||
        (I2C_GetFlagStatus(IIC_PERIPH, I2C_FLAG_ARLO) != RESET) ||
        (I2C_GetFlagStatus(IIC_PERIPH, I2C_FLAG_OVR) != RESET))
        return BSP_IIC_ERR_BUS;
    return BSP_IIC_OK;
}

static uint8_t IIC_WaitFlagSet(uint32_t flag)
{
    uint32_t timeout = IIC_TIMEOUT_LOOPS;
    uint8_t result;

    while (I2C_GetFlagStatus(IIC_PERIPH, flag) == RESET)
    {
        result = IIC_GetError();
        if (result != BSP_IIC_OK)
            return result;
        if (--timeout == 0U)
            return BSP_IIC_ERR_TIMEOUT;
    }
    return BSP_IIC_OK;
}

static uint8_t IIC_WaitBusFree(void)
{
    uint32_t timeout = IIC_TIMEOUT_LOOPS;
    uint8_t result;

    while (I2C_GetFlagStatus(IIC_PERIPH, I2C_FLAG_BUSY) != RESET)
    {
        result = IIC_GetError();
        if (result != BSP_IIC_OK)
            return result;
        if (--timeout == 0U)
            return IIC_UnlockBus();
    }
    return BSP_IIC_OK;
}

/* ADDR is cleared only by an SR1 read followed by an SR2 read on STM32F1. */
static void IIC_ClearAddrFlag(void)
{
    volatile uint16_t dummy;

    dummy = IIC_PERIPH->SR1;
    dummy = IIC_PERIPH->SR2;
    (void)dummy;
}

static uint8_t IIC_Start(void)
{
    I2C_GenerateSTART(IIC_PERIPH, ENABLE);
    return IIC_WaitFlagSet(I2C_FLAG_SB);
}

static uint8_t IIC_SendAddress(uint8_t addr7, uint8_t direction)
{
    I2C_Send7bitAddress(IIC_PERIPH, (uint8_t)(addr7 << 1), direction);
    return IIC_WaitFlagSet(I2C_FLAG_ADDR);
}

static void IIC_Abort(void)
{
    I2C_GenerateSTOP(IIC_PERIPH, ENABLE);
    I2C_AcknowledgeConfig(IIC_PERIPH, ENABLE);
    I2C_NACKPositionConfig(IIC_PERIPH, I2C_NACKPosition_Current);
    IIC_ConfigPeripheral();
}

void BSP_IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    g_debugIicRecovery = 0U;
    g_debugIicLines = 0U;
    RCC_APB2PeriphClockCmd(IIC_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(IIC_PERIPH_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = IIC_SCL_PIN | IIC_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IIC_GPIO_PORT, &GPIO_InitStructure);

    IIC_ConfigPeripheral();
    if (I2C_GetFlagStatus(IIC_PERIPH, I2C_FLAG_BUSY) != RESET)
        (void)IIC_UnlockBus();
}

void BSP_IIC_Recover(void)
{
    (void)IIC_UnlockBus();
}

uint8_t BSP_IIC_WriteAddr(uint8_t addr7, uint8_t reg, uint8_t data)
{
    uint8_t result;

    if (addr7 > 0x7FU)
        return BSP_IIC_ERR_PARAM;

    result = IIC_WaitBusFree();
    if (result != BSP_IIC_OK)
    {
        IIC_Abort();
        return result;
    }

    result = IIC_Start();
    if (result != BSP_IIC_OK) goto write_error;
    result = IIC_SendAddress(addr7, I2C_Direction_Transmitter);
    if (result != BSP_IIC_OK) goto write_error;
    IIC_ClearAddrFlag();

    result = IIC_WaitFlagSet(I2C_FLAG_TXE);
    if (result != BSP_IIC_OK) goto write_error;
    I2C_SendData(IIC_PERIPH, reg);

    result = IIC_WaitFlagSet(I2C_FLAG_TXE);
    if (result != BSP_IIC_OK) goto write_error;
    I2C_SendData(IIC_PERIPH, data);

    result = IIC_WaitFlagSet(I2C_FLAG_BTF);
    if (result != BSP_IIC_OK) goto write_error;

    I2C_GenerateSTOP(IIC_PERIPH, ENABLE);
    return BSP_IIC_OK;

write_error:
    IIC_Abort();
    return result;
}

uint8_t BSP_IIC_ReadAddr(uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t result;

    if ((addr7 > 0x7FU) || (buf == 0) || (len == 0U))
        return BSP_IIC_ERR_PARAM;

    result = IIC_WaitBusFree();
    if (result != BSP_IIC_OK)
    {
        IIC_Abort();
        return result;
    }

    /* Write the register pointer first. */
    result = IIC_Start();
    if (result != BSP_IIC_OK) goto read_error;
    result = IIC_SendAddress(addr7, I2C_Direction_Transmitter);
    if (result != BSP_IIC_OK) goto read_error;
    IIC_ClearAddrFlag();

    result = IIC_WaitFlagSet(I2C_FLAG_TXE);
    if (result != BSP_IIC_OK) goto read_error;
    I2C_SendData(IIC_PERIPH, reg);
    result = IIC_WaitFlagSet(I2C_FLAG_BTF);
    if (result != BSP_IIC_OK) goto read_error;

    /* Repeated START then receive exactly len bytes. */
    result = IIC_Start();
    if (result != BSP_IIC_OK) goto read_error;

    if (len == 1U)
    {
        /* ACK must be disabled before ADDR is cleared for a one-byte read. */
        I2C_AcknowledgeConfig(IIC_PERIPH, DISABLE);
        result = IIC_SendAddress(addr7, I2C_Direction_Receiver);
        if (result != BSP_IIC_OK) goto read_error;
        IIC_ClearAddrFlag();
        I2C_GenerateSTOP(IIC_PERIPH, ENABLE);

        result = IIC_WaitFlagSet(I2C_FLAG_RXNE);
        if (result != BSP_IIC_OK) goto read_error;
        *buf = I2C_ReceiveData(IIC_PERIPH);
    }
    else if (len == 2U)
    {
        /* STM32F1 two-byte sequence: POS=1, NACK, clear ADDR, wait BTF. */
        I2C_NACKPositionConfig(IIC_PERIPH, I2C_NACKPosition_Next);
        I2C_AcknowledgeConfig(IIC_PERIPH, ENABLE);
        result = IIC_SendAddress(addr7, I2C_Direction_Receiver);
        if (result != BSP_IIC_OK) goto read_error;
        I2C_AcknowledgeConfig(IIC_PERIPH, DISABLE);
        IIC_ClearAddrFlag();

        result = IIC_WaitFlagSet(I2C_FLAG_BTF);
        if (result != BSP_IIC_OK) goto read_error;
        I2C_GenerateSTOP(IIC_PERIPH, ENABLE);
        *buf++ = I2C_ReceiveData(IIC_PERIPH);
        *buf = I2C_ReceiveData(IIC_PERIPH);
    }
    else
    {
        I2C_NACKPositionConfig(IIC_PERIPH, I2C_NACKPosition_Current);
        I2C_AcknowledgeConfig(IIC_PERIPH, ENABLE);
        result = IIC_SendAddress(addr7, I2C_Direction_Receiver);
        if (result != BSP_IIC_OK) goto read_error;
        IIC_ClearAddrFlag();

        while (len > 3U)
        {
            result = IIC_WaitFlagSet(I2C_FLAG_RXNE);
            if (result != BSP_IIC_OK) goto read_error;
            *buf++ = I2C_ReceiveData(IIC_PERIPH);
            len--;
        }

        /* Three bytes remain: use the F1 ACK/STOP sequence for the final 3. */
        result = IIC_WaitFlagSet(I2C_FLAG_BTF);
        if (result != BSP_IIC_OK) goto read_error;
        I2C_AcknowledgeConfig(IIC_PERIPH, DISABLE);
        *buf++ = I2C_ReceiveData(IIC_PERIPH);
        len--;

        result = IIC_WaitFlagSet(I2C_FLAG_BTF);
        if (result != BSP_IIC_OK) goto read_error;
        I2C_GenerateSTOP(IIC_PERIPH, ENABLE);
        *buf++ = I2C_ReceiveData(IIC_PERIPH);
        *buf = I2C_ReceiveData(IIC_PERIPH);
    }

    I2C_AcknowledgeConfig(IIC_PERIPH, ENABLE);
    I2C_NACKPositionConfig(IIC_PERIPH, I2C_NACKPosition_Current);
    return BSP_IIC_OK;

read_error:
    IIC_Abort();
    return result;
}
