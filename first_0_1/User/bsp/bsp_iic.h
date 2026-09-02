#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#include "stm32f10x.h"

/* Hardware I2C1 is remapped by BSP_Board_Init(): SCL=PB8, SDA=PB9. */
#define IIC_PERIPH              I2C1
#define IIC_PERIPH_CLK          RCC_APB1Periph_I2C1
#define IIC_GPIO_PORT           GPIOB
#define IIC_GPIO_CLK            RCC_APB2Periph_GPIOB
#define IIC_SCL_PIN             GPIO_Pin_8
#define IIC_SDA_PIN             GPIO_Pin_9
#define IIC_BUS_SPEED           100000UL

/* All device addresses passed to this BSP are unshifted 7-bit I2C addresses. */
#define BSP_IIC_OK              0U
#define BSP_IIC_ERR_PARAM       1U
#define BSP_IIC_ERR_TIMEOUT     2U
#define BSP_IIC_ERR_NACK        3U
#define BSP_IIC_ERR_BUS         4U
#define BSP_IIC_ERR_BUSY        5U

void    BSP_IIC_Init(void);
void    BSP_IIC_Recover(void);
uint8_t BSP_IIC_WriteAddr(uint8_t addr7, uint8_t reg, uint8_t data);
uint8_t BSP_IIC_ReadAddr(uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t len);

#endif
