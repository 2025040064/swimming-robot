/* Polling USART3 debug output keeps USART1 isolated for the K230 protocol. */

#include "bsp_debug.h"

#define DEBUG_TX_TIMEOUT_LOOPS  100000UL

void BSP_Debug_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    if (baudrate == 0U) baudrate = DEBUG_DEFAULT_BAUDRATE;
    RCC_APB2PeriphClockCmd(DEBUG_USART_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(DEBUG_USART_CLK, ENABLE);

    gpio.GPIO_Pin = DEBUG_USART_TX_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DEBUG_USART_GPIO, &gpio);

    gpio.GPIO_Pin = DEBUG_USART_RX_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(DEBUG_USART_GPIO, &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = baudrate;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(DEBUG_USART, &usart);
    USART_Cmd(DEBUG_USART, ENABLE);
}

void BSP_Debug_SendByte(uint8_t ch)
{
    uint32_t timeout = DEBUG_TX_TIMEOUT_LOOPS;
    while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET)
    {
        if (--timeout == 0U) return;
    }
    USART_SendData(DEBUG_USART, ch);
}

void BSP_Debug_SendString(const char *text)
{
    if (text == 0) return;
    while (*text != '\0')
        BSP_Debug_SendByte((uint8_t)*text++);
}

void BSP_Debug_SendU32(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do
    {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    while (count > 0U)
        BSP_Debug_SendByte((uint8_t)digits[--count]);
}
