#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "stm32f10x.h"

#define K230_USART               USART1
#define K230_USART_CLK           RCC_APB2Periph_USART1
#define K230_USART_GPIO_CLK      RCC_APB2Periph_GPIOA
#define K230_USART_TX_PIN        GPIO_Pin_9
#define K230_USART_RX_PIN        GPIO_Pin_10
#define K230_USART_GPIO          GPIOA
#define K230_USART_IRQn          USART1_IRQn

#define USART_RX_BUF_SIZE        256   /* 22ms at 115200 — safe margin over 10ms consumer */

void BSP_USART_Init(uint32_t baudrate);
void BSP_USART_SendByte(uint8_t ch);
void BSP_USART_SendString(char *str);
void BSP_USART_SendBuf(uint8_t *buf, uint16_t len);
uint8_t BSP_USART_GetRxByte(void);
uint8_t BSP_USART_RxAvailable(void);
void BSP_USART_ClearRx(void);

/* Overrun error counter — non-zero indicates serial data loss */
uint32_t BSP_USART_GetOreCount(void);

#endif
