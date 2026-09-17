#ifndef __DRV_TB6612_H
#define __DRV_TB6612_H

#include "stm32f10x.h"

/* PWM Timer: TIM2 default mapping, four channels on PA0/PA1/PA2/PA3. */
#define TB_PWM_TIM              TIM2
#define TB_PWM_CLK              RCC_APB1Periph_TIM2
#define TB_PWM_PERIOD           7199
#define TB_PWM_MAX_DUTY         TB_PWM_PERIOD
#define TB_PWM_GPIO_PORT        GPIOA
#define TB_PWM_GPIO_CLK         RCC_APB2Periph_GPIOA
#define TB_PWM_GPIO_PINS        (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3)

/*
 * TB6612 #1 (propulsion):
 * Channel A: AO1/AO2 -> left motor;  PWMA=PA0, AIN1/2=PB12/PB13.
 * Channel B: BO1/BO2 -> right motor; PWMB=PA1, BIN1/2=PB14/PB15.
 * STBY=PA4.
 */
#define TB1_AIN1_PORT           GPIOB
#define TB1_AIN1_PIN            GPIO_Pin_12
#define TB1_AIN2_PORT           GPIOB
#define TB1_AIN2_PIN            GPIO_Pin_13
#define TB1_BIN1_PORT           GPIOB
#define TB1_BIN1_PIN            GPIO_Pin_14
#define TB1_BIN2_PORT           GPIOB
#define TB1_BIN2_PIN            GPIO_Pin_15
#define TB1_STBY_PORT           GPIOA
#define TB1_STBY_PIN            GPIO_Pin_4

#define TB1_CLK                 RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO

/*
 * TB6612 #2 (K230 gimbal):
 * Channel A: AO1/AO2 -> pan motor;  PWMA=PA2, AIN1/2=PA11/PA12.
 * Channel B: BO1/BO2 -> tilt motor; PWMB=PA3, BIN1/2=PB0/PB1.
 * STBY=PA5.
 */
#define TB2_AIN1_PORT           GPIOA
#define TB2_AIN1_PIN            GPIO_Pin_11
#define TB2_AIN2_PORT           GPIOA
#define TB2_AIN2_PIN            GPIO_Pin_12
#define TB2_BIN1_PORT           GPIOB
#define TB2_BIN1_PIN            GPIO_Pin_0
#define TB2_BIN2_PORT           GPIOB
#define TB2_BIN2_PIN            GPIO_Pin_1
#define TB2_STBY_PORT           GPIOA
#define TB2_STBY_PIN            GPIO_Pin_5

#define TB2_CLK                 RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB

/* Motor IDs */
#define MOTOR_LEFT              0
#define MOTOR_RIGHT             1
#define MOTOR_GIMBAL_PAN        2
#define MOTOR_GIMBAL_TILT       3

void DRV_TB6612_Init(void);
void DRV_TB6612_SetSpeed(uint8_t motor, int16_t speed);
void DRV_TB6612_SetStandby(uint8_t tbNum, uint8_t state);
void DRV_TB6612_StopAll(void);
void DRV_TB6612_RunDirectionDiagnostic(int16_t speed);
uint8_t DRV_TB6612_GetDirectionOutput(void);
uint8_t DRV_TB6612_GetDirectionInput(void);

#endif
