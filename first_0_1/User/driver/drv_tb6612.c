#include "drv_tb6612.h"
#include "robot_config.h"
#include "Delay.h"

void DRV_TB6612_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(TB1_CLK | TB2_CLK | TB_PWM_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(TB_PWM_CLK, ENABLE);
    TIM_DeInit(TB_PWM_TIM);

    /* TB6612 #1: A channel (left) and B channel (right) direction pins. */
#if ROBOT_IMU_MOTOR_TEST_ENABLE && ROBOT_IMU_TEST_ONE_WIRE_DIRECTION
    /* Bench workaround: PB12/PB14 drive high; PB13/PB15 provide pull-down. */
    GPIO_ResetBits(GPIOB, TB1_AIN1_PIN | TB1_AIN2_PIN | TB1_BIN1_PIN | TB1_BIN2_PIN);
    GPIO_InitStructure.GPIO_Pin  = TB1_AIN1_PIN | TB1_BIN1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin  = TB1_AIN2_PIN | TB1_BIN2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
#else
    GPIO_InitStructure.GPIO_Pin  = TB1_AIN1_PIN | TB1_AIN2_PIN | TB1_BIN1_PIN | TB1_BIN2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif

    /* TB6612 #1 STBY: PA4 */
    GPIO_InitStructure.GPIO_Pin = TB1_STBY_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* TB6612 #2: A channel (pan) and B channel (tilt) direction pins. */
    GPIO_InitStructure.GPIO_Pin = TB2_AIN1_PIN | TB2_AIN2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TB2_AIN1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = TB2_BIN1_PIN | TB2_BIN2_PIN;
    GPIO_Init(TB2_BIN1_PORT, &GPIO_InitStructure);

    /* TB6612 #2 STBY: PA5 */
    GPIO_InitStructure.GPIO_Pin = TB2_STBY_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* TIM2 default pins: CH1=PA0, CH2=PA1, CH3=PA2, CH4=PA3. */
    GPIO_InitStructure.GPIO_Pin  = TB_PWM_GPIO_PINS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TB_PWM_GPIO_PORT, &GPIO_InitStructure);

    /* Keep both bridges disabled until every direction/PWM output is safe. */
    GPIO_ResetBits(TB1_STBY_PORT, TB1_STBY_PIN);
    GPIO_ResetBits(TB2_STBY_PORT, TB2_STBY_PIN);
    GPIO_ResetBits(GPIOB, TB1_AIN1_PIN | TB1_AIN2_PIN | TB1_BIN1_PIN | TB1_BIN2_PIN);
    GPIO_ResetBits(TB2_AIN1_PORT, TB2_AIN1_PIN | TB2_AIN2_PIN);
    GPIO_ResetBits(TB2_BIN1_PORT, TB2_BIN1_PIN | TB2_BIN2_PIN);

    /* TIM2: 10kHz PWM = 72MHz / (0+1) / (7199+1) = 10kHz */
    TIM_TimeBaseStructure.TIM_Prescaler         = 0;
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

    DRV_TB6612_StopAll();
    DRV_TB6612_SetStandby(1, 1);
    /* No gimbal command protocol is defined yet; keep its bridge disabled. */
    DRV_TB6612_SetStandby(2, 0);
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
    else if (tbNum == 2)
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

    if (speed > TB_PWM_MAX_DUTY)  speed = TB_PWM_MAX_DUTY;
    if (speed < -TB_PWM_MAX_DUTY) speed = -TB_PWM_MAX_DUTY;

    dir = (speed >= 0) ? 1 : 0;
    if (speed < 0) speed = -speed;

    switch (motor)
    {
    case MOTOR_LEFT:
        /* TB6612 #1 channel A, motor connector AO1/AO2. */
#if ROBOT_IMU_MOTOR_TEST_ENABLE && ROBOT_IMU_TEST_ONE_WIRE_DIRECTION
        if (!dir) speed = 0;
        GPIO_WriteBit(TB1_AIN1_PORT, TB1_AIN1_PIN, speed ? Bit_SET : Bit_RESET);
#else
        GPIO_WriteBit(TB1_AIN1_PORT, TB1_AIN1_PIN, dir ? Bit_SET : Bit_RESET);
        GPIO_WriteBit(TB1_AIN2_PORT, TB1_AIN2_PIN, dir ? Bit_RESET : Bit_SET);
#endif
        TIM_SetCompare1(TB_PWM_TIM, (uint16_t)speed);
        break;
    case MOTOR_RIGHT:
        /* TB6612 #1 channel B, motor connector BO1/BO2. */
#if ROBOT_IMU_MOTOR_TEST_ENABLE && ROBOT_IMU_TEST_ONE_WIRE_DIRECTION
        if (!dir) speed = 0;
        GPIO_WriteBit(TB1_BIN1_PORT, TB1_BIN1_PIN, speed ? Bit_SET : Bit_RESET);
#else
        GPIO_WriteBit(TB1_BIN1_PORT, TB1_BIN1_PIN, dir ? Bit_SET : Bit_RESET);
        GPIO_WriteBit(TB1_BIN2_PORT, TB1_BIN2_PIN, dir ? Bit_RESET : Bit_SET);
#endif
        TIM_SetCompare2(TB_PWM_TIM, (uint16_t)speed);
        break;
    case MOTOR_GIMBAL_PAN:
        /* TB6612 #2 channel A, pan motor connector AO1/AO2. */
        GPIO_WriteBit(TB2_AIN1_PORT, TB2_AIN1_PIN, dir ? Bit_SET : Bit_RESET);
        GPIO_WriteBit(TB2_AIN2_PORT, TB2_AIN2_PIN, dir ? Bit_RESET : Bit_SET);
        TIM_SetCompare3(TB_PWM_TIM, (uint16_t)speed);
        break;
    case MOTOR_GIMBAL_TILT:
        /* TB6612 #2 channel B, tilt motor connector BO1/BO2. */
        GPIO_WriteBit(TB2_BIN1_PORT, TB2_BIN1_PIN, dir ? Bit_SET : Bit_RESET);
        GPIO_WriteBit(TB2_BIN2_PORT, TB2_BIN2_PIN, dir ? Bit_RESET : Bit_SET);
        TIM_SetCompare4(TB_PWM_TIM, (uint16_t)speed);
        break;
    default:
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

void DRV_TB6612_RunDirectionDiagnostic(int16_t speed)
{
#if ROBOT_IMU_MOTOR_TEST_ENABLE && ROBOT_MOTOR_DIRECTION_DIAG_ENABLE
    GPIO_InitTypeDef gpio;
    if (speed < 0) speed = -speed;
    if (speed > TB_PWM_MAX_DUTY) speed = TB_PWM_MAX_DUTY;
    gpio.GPIO_Pin = TB1_AIN1_PIN | TB1_AIN2_PIN | TB1_BIN1_PIN | TB1_BIN2_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    GPIO_ResetBits(GPIOB, TB1_AIN1_PIN | TB1_AIN2_PIN | TB1_BIN1_PIN | TB1_BIN2_PIN);
    GPIO_SetBits(GPIOB, TB1_AIN1_PIN | TB1_BIN1_PIN);
    TIM_SetCompare1(TB_PWM_TIM, (uint16_t)speed);
    TIM_SetCompare2(TB_PWM_TIM, (uint16_t)speed);
    DRV_TB6612_SetStandby(1U, 1U);
    Delay_ms(1500U);

    DRV_TB6612_StopAll();
    Delay_ms(750U);
    GPIO_ResetBits(GPIOB, TB1_AIN1_PIN | TB1_AIN2_PIN | TB1_BIN1_PIN | TB1_BIN2_PIN);
    GPIO_SetBits(GPIOB, TB1_AIN2_PIN | TB1_BIN2_PIN);
    TIM_SetCompare1(TB_PWM_TIM, (uint16_t)speed);
    TIM_SetCompare2(TB_PWM_TIM, (uint16_t)speed);
    Delay_ms(1500U);

    DRV_TB6612_StopAll();
    DRV_TB6612_SetStandby(1U, 0U);
    Delay_ms(750U);
    DRV_TB6612_Init();
#else
    (void)speed;
#endif
}

uint8_t DRV_TB6612_GetDirectionOutput(void)
{
    return (uint8_t)((GPIOB->ODR >> 12) & 0x0FU);
}

uint8_t DRV_TB6612_GetDirectionInput(void)
{
    return (uint8_t)((GPIOB->IDR >> 12) & 0x0FU);
}
