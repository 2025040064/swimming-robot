#ifndef __BSP_BOARD_H
#define __BSP_BOARD_H

#include "stm32f10x.h"

/*
 * Board-wide alternate-function contract (STM32F103C8T6):
 *   TIM2 no remap:        CH1=PA0, CH2=PA1, CH3=PA2, CH4=PA3
 *   I2C1 remap:           SCL=PB8, SDA=PB9
 *   SWJ:                   JTAG disabled, SWD on PA13/PA14 retained
 *
 * Call this once before any GPIO, TIM, I2C, USART or EXTI peripheral is
 * initialized.  PB3/PB4 are released from JTAG here for the right ultrasonic
 * sensor; PA15 remains available as a spare GPIO.
 */
void BSP_Board_Init(void);

#endif
