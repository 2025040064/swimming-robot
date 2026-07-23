#include "bsp_usart.h"

static volatile uint8_t g_rxBuf[USART_RX_BUF_SIZE];
static volatile uint16_t g_rxHead = 0;
static volatile uint16_t g_rxTail = 0;
static volatile uint32_t g_oreCount = 0;

void BSP_USART_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(K230_USART_CLK | K230_USART_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = K230_USART_TX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(K230_USART_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = K230_USART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(K230_USART_GPIO, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(K230_USART, &USART_InitStructure);

    USART_ITConfig(K230_USART, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = K230_USART_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(K230_USART, ENABLE);
}

#define USART_TX_TIMEOUT    100000U   /* ~1.4ms at 72MHz — fail-safe against TX pin stuck low */

void BSP_USART_SendByte(uint8_t ch)
{
    uint32_t timeout = USART_TX_TIMEOUT;
    USART_SendData(K230_USART, ch);
    while (USART_GetFlagStatus(K230_USART, USART_FLAG_TXE) == RESET && --timeout);
}

void BSP_USART_SendString(char *str)
{
    while (*str)
    {
        BSP_USART_SendByte(*str++);
    }
}

void BSP_USART_SendBuf(uint8_t *buf, uint16_t len)
{
    while (len--)
    {
        BSP_USART_SendByte(*buf++);
    }
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(K230_USART, USART_IT_RXNE) != RESET)
    {
        uint8_t ch = USART_ReceiveData(K230_USART);
        uint16_t nextHead = (g_rxHead + 1) % USART_RX_BUF_SIZE;
        if (nextHead != g_rxTail)
        {
            g_rxBuf[g_rxHead] = ch;
            g_rxHead = nextHead;
        }
        /* else: buffer full, byte dropped */
    }
    if (USART_GetITStatus(K230_USART, USART_IT_ORE) != RESET)
    {
        g_oreCount++;
        USART_ReceiveData(K230_USART);  /* Clear ORE flag */
    }
}

uint8_t BSP_USART_GetRxByte(void)
{
    uint8_t ch = 0;
    if (g_rxTail != g_rxHead)
    {
        ch = g_rxBuf[g_rxTail];
        g_rxTail = (g_rxTail + 1) % USART_RX_BUF_SIZE;
    }
    return ch;
}

uint8_t BSP_USART_RxAvailable(void)
{
    return (g_rxHead != g_rxTail) ? 1 : 0;
}

void BSP_USART_ClearRx(void)
{
    __disable_irq();
    g_rxHead = 0;
    g_rxTail = 0;
    __enable_irq();
}

uint32_t BSP_USART_GetOreCount(void)
{
    return g_oreCount;
}
