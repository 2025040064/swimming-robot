#include "bsp_iic.h"
#include "bsp_systick.h"

static void IIC_Delay(void)
{
    volatile uint8_t i = 10;
    while (i--);
}

void BSP_IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(IIC_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin  = IIC_SCL_PIN | IIC_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IIC_SCL_PORT, &GPIO_InitStructure);
    IIC_SCL_H();
    IIC_SDA_H();
}

void BSP_IIC_Start(void)
{
    IIC_SDA_H();
    IIC_SCL_H();
    IIC_Delay();
    IIC_SDA_L();
    IIC_Delay();
    IIC_SCL_L();
}

void BSP_IIC_Stop(void)
{
    IIC_SDA_L();
    IIC_SCL_H();
    IIC_Delay();
    IIC_SDA_H();
    IIC_Delay();
}

uint8_t BSP_IIC_WaitAck(void)
{
    uint8_t ack;
    IIC_SDA_H();
    IIC_Delay();
    IIC_SCL_H();
    IIC_Delay();
    ack = IIC_SDA_READ();
    IIC_SCL_L();
    return ack;
}

void BSP_IIC_SendAck(void)
{
    IIC_SDA_L();
    IIC_Delay();
    IIC_SCL_H();
    IIC_Delay();
    IIC_SCL_L();
}

void BSP_IIC_SendNAck(void)
{
    IIC_SDA_H();
    IIC_Delay();
    IIC_SCL_H();
    IIC_Delay();
    IIC_SCL_L();
}

void BSP_IIC_SendByte(uint8_t data)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)
            IIC_SDA_H();
        else
            IIC_SDA_L();
        data <<= 1;
        IIC_Delay();
        IIC_SCL_H();
        IIC_Delay();
        IIC_SCL_L();
    }
}

uint8_t BSP_IIC_ReadByte(uint8_t ack)
{
    uint8_t i, data = 0;
    IIC_SDA_H();
    for (i = 0; i < 8; i++)
    {
        data <<= 1;
        IIC_SCL_H();
        IIC_Delay();
        if (IIC_SDA_READ())
            data |= 0x01;
        IIC_SCL_L();
        IIC_Delay();
    }
    if (ack)
        BSP_IIC_SendAck();
    else
        BSP_IIC_SendNAck();
    return data;
}

uint8_t BSP_IIC_WriteAddr(uint8_t addr, uint8_t reg, uint8_t data)
{
    BSP_IIC_Start();
    BSP_IIC_SendByte((addr << 1) | 0x00);
    if (BSP_IIC_WaitAck()) { BSP_IIC_Stop(); return 1; }
    BSP_IIC_SendByte(reg);
    if (BSP_IIC_WaitAck()) { BSP_IIC_Stop(); return 2; }
    BSP_IIC_SendByte(data);
    if (BSP_IIC_WaitAck()) { BSP_IIC_Stop(); return 3; }
    BSP_IIC_Stop();
    return 0;
}

uint8_t BSP_IIC_ReadAddr(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    BSP_IIC_Start();
    BSP_IIC_SendByte((addr << 1) | 0x00);
    if (BSP_IIC_WaitAck()) { BSP_IIC_Stop(); return 1; }
    BSP_IIC_SendByte(reg);
    if (BSP_IIC_WaitAck()) { BSP_IIC_Stop(); return 2; }
    BSP_IIC_Start();
    BSP_IIC_SendByte((addr << 1) | 0x01);
    if (BSP_IIC_WaitAck()) { BSP_IIC_Stop(); return 3; }
    for (i = 0; i < len; i++)
    {
        buf[i] = BSP_IIC_ReadByte(i < (len - 1));
    }
    BSP_IIC_Stop();
    return 0;
}
