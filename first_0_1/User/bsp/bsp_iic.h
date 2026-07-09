#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#include "stm32f10x.h"

/* Software I2C pins: PB10=SCL, PB11=SDA */
#define IIC_SCL_PORT        GPIOB
#define IIC_SCL_PIN         GPIO_Pin_10
#define IIC_SDA_PORT        GPIOB
#define IIC_SDA_PIN         GPIO_Pin_11
#define IIC_CLK             RCC_APB2Periph_GPIOB

/* Timeout iterations for WaitAck (prevents permanent hang if MPU6050 unresponsive) */
#define IIC_TIMEOUT         2000

#define IIC_SCL_H()         GPIO_SetBits(IIC_SCL_PORT, IIC_SCL_PIN)
#define IIC_SCL_L()         GPIO_ResetBits(IIC_SCL_PORT, IIC_SCL_PIN)
#define IIC_SDA_H()         GPIO_SetBits(IIC_SDA_PORT, IIC_SDA_PIN)
#define IIC_SDA_L()         GPIO_ResetBits(IIC_SDA_PORT, IIC_SDA_PIN)
#define IIC_SDA_READ()      GPIO_ReadInputDataBit(IIC_SDA_PORT, IIC_SDA_PIN)

void    BSP_IIC_Init(void);
void    BSP_IIC_Start(void);
void    BSP_IIC_Stop(void);
void    BSP_IIC_SendAck(void);
void    BSP_IIC_SendNAck(void);
void    BSP_IIC_SendByte(uint8_t data);
uint8_t BSP_IIC_ReadByte(uint8_t ack);
uint8_t BSP_IIC_WriteAddr(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t BSP_IIC_ReadAddr(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);

#endif
