#ifndef __BSP_BOARD_H
#define __BSP_BOARD_H

#include "stm32f10x.h"

/*
 * Board-wide alternate-function contract (STM32F103C8T6):
 *   TIM2 partial remap 2: CH1=PA0, CH2=PA1, CH3=PB10, CH4=PB11
 *   I2C1 remap:           SCL=PB8, SDA=PB9
 *   SWJ:                   JTAG disabled, SWD on PA13/PA14 retained
 *
 * Call this once before any GPIO, TIM, I2C, USART or EXTI peripheral is
 * initialized.  PB3/PB4 are released from JTAG here for the right ultrasonic
 * sensor.
 */
void BSP_Board_Init(void);

#endif
