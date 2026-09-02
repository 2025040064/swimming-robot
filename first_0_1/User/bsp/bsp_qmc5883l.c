/* QMC5883L driver for the shared I2C1 bus (7-bit address 0x0D). */

#include "bsp_qmc5883l.h"
#include "bsp_iic.h"

#define QMC_REG_X_LSB              0x00U
#define QMC_REG_STATUS             0x06U
#define QMC_REG_CONTROL_1          0x09U
#define QMC_REG_CONTROL_2          0x0AU
#define QMC_REG_SET_RESET_PERIOD   0x0BU
#define QMC_REG_CHIP_ID            0x0DU

#define QMC_STATUS_DRDY            0x01U
#define QMC_STATUS_OVL             0x02U
#define QMC_CHIP_ID_VALUE          0xFFU

/* OSR=512, range=+/-8G, ODR=200Hz, continuous-measurement mode. */
#define QMC_CONTROL_1_VALUE        0x1DU
#define QMC_SOFT_RESET             0x80U

uint8_t BSP_QMC5883L_Test(void)
{
    uint8_t id;

    if (BSP_IIC_ReadAddr(QMC5883L_ADDR, QMC_REG_CHIP_ID, &id, 1) != BSP_IIC_OK)
        return QMC5883L_ERR_BUS;
    if (id != QMC_CHIP_ID_VALUE)
        return QMC5883L_ERR_ID;
    return QMC5883L_OK;
}

uint8_t BSP_QMC5883L_Init(void)
{
    uint8_t result;

    result = BSP_QMC5883L_Test();
    if (result != QMC5883L_OK)
        return result;

    if (BSP_IIC_WriteAddr(QMC5883L_ADDR, QMC_REG_CONTROL_2, QMC_SOFT_RESET) != BSP_IIC_OK)
        return QMC5883L_ERR_BUS;
    if (BSP_IIC_WriteAddr(QMC5883L_ADDR, QMC_REG_SET_RESET_PERIOD, 0x01) != BSP_IIC_OK)
        return QMC5883L_ERR_BUS;
    if (BSP_IIC_WriteAddr(QMC5883L_ADDR, QMC_REG_CONTROL_1, QMC_CONTROL_1_VALUE) != BSP_IIC_OK)
        return QMC5883L_ERR_BUS;

    return QMC5883L_OK;
}

uint8_t BSP_QMC5883L_ReadRaw(int16_t *mag)
{
    uint8_t status;
    uint8_t data[6];

    if (mag == 0)
        return QMC5883L_ERR_PARAM;

    if (BSP_IIC_ReadAddr(QMC5883L_ADDR, QMC_REG_STATUS, &status, 1) != BSP_IIC_OK)
        return QMC5883L_ERR_BUS;
    if ((status & QMC_STATUS_OVL) != 0U)
        return QMC5883L_ERR_OVERFLOW;
    if ((status & QMC_STATUS_DRDY) == 0U)
        return QMC5883L_ERR_NOT_READY;
    if (BSP_IIC_ReadAddr(QMC5883L_ADDR, QMC_REG_X_LSB, data, 6) != BSP_IIC_OK)
        return QMC5883L_ERR_BUS;

    mag[0] = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    mag[1] = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    mag[2] = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
    return QMC5883L_OK;
}
