#include "bsp_board.h"

void BSP_Board_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* TIM2: CH1=PA0, CH2=PA1, CH3=PB10, CH4=PB11. */
    GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);

    /* I2C1: SCL=PB8, SDA=PB9. */
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);

    /* Free PB3/PB4 while preserving the PA13/PA14 SWD debug interface. */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /* Two pre-emption bits: echo EXTI=0, K230 UART=1; PA2/PA3 stay free. */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}
