#include "bsp_board.h"

void BSP_Board_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* TIM2 stays on its default pins: CH1=PA0, CH2=PA1, CH3=PA2, CH4=PA3. */

    /* I2C1: SCL=PB8, SDA=PB9. */
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);

    /* Free PB3/PB4 while preserving the PA13/PA14 SWD debug interface. */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /* Two pre-emption bits: echo EXTI=0, K230 UART=1; debug USART3 is polled. */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}
