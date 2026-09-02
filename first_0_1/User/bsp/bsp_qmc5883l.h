#ifndef __BSP_QMC5883L_H
#define __BSP_QMC5883L_H

#include "stm32f10x.h"

#define QMC5883L_ADDR              0x0DU

#define QMC5883L_OK                0U
#define QMC5883L_ERR_BUS           1U
#define QMC5883L_ERR_ID            2U
#define QMC5883L_ERR_NOT_READY     3U
#define QMC5883L_ERR_OVERFLOW      4U
#define QMC5883L_ERR_PARAM         5U

/* BSP_IIC_Init() must be called once before this shared-bus device is used. */
uint8_t BSP_QMC5883L_Init(void);
uint8_t BSP_QMC5883L_Test(void);
uint8_t BSP_QMC5883L_ReadRaw(int16_t *mag);

#endif
