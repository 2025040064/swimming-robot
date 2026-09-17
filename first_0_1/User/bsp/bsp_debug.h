#ifndef __BSP_DEBUG_H
#define __BSP_DEBUG_H

#include "stm32f10x.h"

/*
 * USART1 remains exclusive to the K230 protocol.
 * The dedicated 3.3V debug UART is USART3: PB10=TX, PB11=RX.
 */
#define DEBUG_USART                 USART3
#define DEBUG_USART_CLK             RCC_APB1Periph_USART3
#define DEBUG_USART_GPIO            GPIOB
#define DEBUG_USART_GPIO_CLK        RCC_APB2Periph_GPIOB
#define DEBUG_USART_TX_PIN          GPIO_Pin_10
#define DEBUG_USART_RX_PIN          GPIO_Pin_11
#define DEBUG_DEFAULT_BAUDRATE      115200UL

void BSP_Debug_Init(uint32_t baudrate);
void BSP_Debug_SendByte(uint8_t ch);
void BSP_Debug_SendString(const char *text);
void BSP_Debug_SendU32(uint32_t value);

#ifdef DEBUG_ENABLE
#define DBG_INIT()                  BSP_Debug_Init(DEBUG_DEFAULT_BAUDRATE)
#define DBG_PRINT(text)             BSP_Debug_SendString(text)
#define DBG_U32(value)              BSP_Debug_SendU32(value)
#else
#define DBG_INIT()                  ((void)0)
#define DBG_PRINT(text)             ((void)0)
#define DBG_U32(value)              ((void)0)
#endif
#define DBG_STATE(old, new)          ((void)0)
#define DBG_SENSOR(name, val)        ((void)0)
#define DBG_ASSERT(cond)             ((void)0)

#endif
