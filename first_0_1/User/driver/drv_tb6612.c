#include "drv_tb6612.h"

void DRV_TB6612_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(TB1_CLK | TB2_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(TB_PWM_CLK, ENABLE);

    /* TB6612#1 direction pins: PB12-15 */
    GPIO_InitStructure.GPIO_Pin  = TB1_AIN1_PIN | TB1_AIN2_PIN | TB1_BIN1_PIN | TB1_BIN2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* TB6612#1 STBY: PA4 */
    GPIO_InitStructure.GPIO_Pin = TB1_STBY_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* TB6612#2 direction pins */
    GPIO_InitStructure.GPIO_Pin  = TB2_AIN1_PIN | TB2_AIN2_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin  = TB2_BIN1_PIN | TB2_BIN2_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* TB6612#2 STBY: PA5 */
    GPIO_InitStructure.GPIO_Pin = TB2_STBY_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PWM GPIOs: PA0-3 for TIM2 CH1-4 */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* TIM2: 10kHz PWM, period=7200 */
    TIM_TimeBaseStructure.TIM_Prescaler         = 71;
    TIM_TimeBaseStructure.TIM_Period            = TB_PWM_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TB_PWM_TIM, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse       = 0;

    TIM_OC1Init(TB_PWM_TIM, &TIM_OCInitStructure);
    TIM_OC2Init(TB_PWM_TIM, &TIM_OCInitStructure);
    TIM_OC3Init(TB_PWM_TIM, &TIM_OCInitStructure);
    TIM_OC4Init(TB_PWM_TIM, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TB_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TB_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TB_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TB_PWM_TIM, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TB_PWM_TIM, ENABLE);
    TIM_Cmd(TB_PWM_TIM, ENABLE);

    DRV_TB6612_SetStandby(1, 1);
    DRV_TB6612_SetStandby(2, 1);
    DRV_TB6612_StopAll();
}

void DRV_TB6612_SetStandby(uint8_t tbNum, uint8_t state)
{
    if (tbNum == 1)
    {
        if (state)
            GPIO_SetBits(TB1_STBY_PORT, TB1_STBY_PIN);
        else
            GPIO_ResetBits(TB1_STBY_PORT, TB1_STBY_PIN);
    }
    else
    {
        if (state)
            GPIO_SetBits(TB2_STBY_PORT, TB2_STBY_PIN);
        else
            GPIO_ResetBits(TB2_STBY_PORT, TB2_STBY_PIN);
    }
}

void DRV_TB6612_SetSpeed(uint8_t motor, int16_t speed)
{
    uint8_t dir;

    if (speed > 7200)  speed = 7200;
    if (speed < -7200) speed = -7200;

    dir = (speed >= 0) ? 1 : 0;
    if (speed < 0) speed = -speed;

    switch (motor)
    {
    case MOTOR_LEFT:
        GPIO_WriteBit(TB1_AIN1_PORT, TB1_AIN1_PIN, dir ? Bit_SET : Bit_RESET);
        GPIO_WriteBit(TB1_AIN2_PORT, TB1_AIN2_PIN, dir ? Bit_RESET : Bit_SET);
        TIM_SetCompare1(TB_PWM_TIM, (uint16_t)speed);
        break;
    case MOTOR_RIGHT:
        GPIO_WriteBit(TB1_BIN1_PORT, TB1_BIN1_PIN, dir ? Bit_SET : Bit_RESET);
        GPIO_WriteBit(TB1_BIN2_PORT, TB1_BIN2_PIN, dir ? Bit_RESET : Bit_SET);
        TIM_SetCompare2(TB_PWM_TIM, (uint16_t)speed);
        break;
    case MOTOR_ROLLER:
        GPIO_WriteBit(TB2_AIN1_PORT, TB2_AIN1_PIN, dir ? Bit_SET : Bit_RESET);
        GPIO_WriteBit(TB2_AIN2_PORT, TB2_AIN2_PIN, dir ? Bit_RESET : Bit_SET);
        TIM_SetCompare3(TB_PWM_TIM, (uint16_t)speed);
        break;
    case MOTOR_CONVEYOR:
        GPIO_WriteBit(TB2_BIN1_PORT, TB2_BIN1_PIN, dir ? Bit_SET : Bit_RESET);
        GPIO_WriteBit(TB2_BIN2_PORT, TB2_BIN2_PIN, dir ? Bit_RESET : Bit_SET);
        TIM_SetCompare4(TB_PWM_TIM, (uint16_t)speed);
        break;
    }
}

void DRV_TB6612_StopAll(void)
{
    TIM_SetCompare1(TB_PWM_TIM, 0);
    TIM_SetCompare2(TB_PWM_TIM, 0);
    TIM_SetCompare3(TB_PWM_TIM, 0);
    TIM_SetCompare4(TB_PWM_TIM, 0);
}
