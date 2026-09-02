#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "stm32f10x.h"

/* K230 link: STM32 PA9(TX) -> K230 RX, PA10(RX) <- K230 TX. */
#define K230_USART                  USART1
#define K230_USART_CLK              RCC_APB2Periph_USART1
#define K230_USART_GPIO_CLK         RCC_APB2Periph_GPIOA
#define K230_USART_TX_PIN           GPIO_Pin_9
#define K230_USART_RX_PIN           GPIO_Pin_10
#define K230_USART_GPIO             GPIOA
#define K230_USART_IRQn             USART1_IRQn
#define K230_DEFAULT_BAUDRATE       115200UL

/* PA2/PA3 are intentionally unassigned: this hardware has no GPS module. */

#define K230_RX_BUF_SIZE            256U
#define K230_TX_BUF_SIZE            256U

#define BSP_USART_OK                0U
#define BSP_USART_ERR_TX_FULL       1U

void     BSP_K230_Init(uint32_t baudrate);
uint8_t  BSP_K230_SendByte(uint8_t ch);
uint8_t  BSP_K230_SendString(const char *str);
uint8_t  BSP_K230_SendBuf(const uint8_t *buf, uint16_t len);
uint8_t  BSP_K230_GetRxByte(void);
uint8_t  BSP_K230_RxAvailable(void);
void     BSP_K230_ClearRx(void);
uint32_t BSP_K230_GetOreCount(void);
uint32_t BSP_K230_GetRxDropCount(void);
uint32_t BSP_K230_GetTxDropCount(void);

#endif
