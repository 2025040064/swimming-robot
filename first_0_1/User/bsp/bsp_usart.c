/* Dedicated UART drivers: USART1 for K230 and USART2 for GPS. */

#include "bsp_usart.h"

#define USART_TX_TIMEOUT_LOOPS     100000UL

static volatile uint8_t g_k230RxBuf[K230_RX_BUF_SIZE];
static volatile uint16_t g_k230RxHead = 0;
static volatile uint16_t g_k230RxTail = 0;
static volatile uint32_t g_k230OreCount = 0;
static volatile uint32_t g_k230RxDropCount = 0;

/* K230 telemetry is queued so a status frame does not block the scheduler. */
static volatile uint8_t g_k230TxBuf[K230_TX_BUF_SIZE];
static volatile uint16_t g_k230TxHead = 0;
static volatile uint16_t g_k230TxTail = 0;
static volatile uint32_t g_k230TxDropCount = 0;

static volatile uint8_t g_gpsRxBuf[GPS_RX_BUF_SIZE];
static volatile uint16_t g_gpsRxHead = 0;
static volatile uint16_t g_gpsRxTail = 0;
static volatile uint32_t g_gpsOreCount = 0;
static volatile uint32_t g_gpsRxDropCount = 0;

static void USART_CommonInit(USART_TypeDef *USARTx, uint32_t baudrate)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USARTx, &USART_InitStructure);
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
    USART_Cmd(USARTx, ENABLE);
}

static void K230_RxPush(uint8_t ch)
{
    uint16_t nextHead = (uint16_t)((g_k230RxHead + 1U) % K230_RX_BUF_SIZE);

    if (nextHead == g_k230RxTail)
        g_k230RxDropCount++;
    else
    {
        g_k230RxBuf[g_k230RxHead] = ch;
        g_k230RxHead = nextHead;
    }
}

static void GPS_RxPush(uint8_t ch)
{
    uint16_t nextHead = (uint16_t)((g_gpsRxHead + 1U) % GPS_RX_BUF_SIZE);

    if (nextHead == g_gpsRxTail)
        g_gpsRxDropCount++;
    else
    {
        g_gpsRxBuf[g_gpsRxHead] = ch;
        g_gpsRxHead = nextHead;
    }
}

static uint16_t K230_TxFree(void)
{
    if (g_k230TxHead >= g_k230TxTail)
        return (uint16_t)(K230_TX_BUF_SIZE - (g_k230TxHead - g_k230TxTail) - 1U);
    return (uint16_t)(g_k230TxTail - g_k230TxHead - 1U);
}

void BSP_K230_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(K230_USART_CLK | K230_USART_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = K230_USART_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(K230_USART_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = K230_USART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(K230_USART_GPIO, &GPIO_InitStructure);

    g_k230RxHead = 0;
    g_k230RxTail = 0;
    g_k230TxHead = 0;
    g_k230TxTail = 0;
    g_k230OreCount = 0;
    g_k230RxDropCount = 0;
    g_k230TxDropCount = 0;

    USART_CommonInit(K230_USART, baudrate);

    NVIC_InitStructure.NVIC_IRQChannel = K230_USART_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

uint8_t BSP_K230_SendBuf(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if ((buf == 0) || (len == 0U))
        return BSP_USART_OK;

    __disable_irq();
    if (len > K230_TxFree())
    {
        g_k230TxDropCount++;
        __enable_irq();
        return BSP_USART_ERR_TX_FULL;
    }

    for (i = 0; i < len; i++)
    {
        g_k230TxBuf[g_k230TxHead] = buf[i];
        g_k230TxHead = (uint16_t)((g_k230TxHead + 1U) % K230_TX_BUF_SIZE);
    }
    USART_ITConfig(K230_USART, USART_IT_TXE, ENABLE);
    __enable_irq();
    return BSP_USART_OK;
}

uint8_t BSP_K230_SendByte(uint8_t ch)
{
    return BSP_K230_SendBuf(&ch, 1);
}

uint8_t BSP_K230_SendString(const char *str)
{
    uint16_t len = 0;

    if (str == 0)
        return BSP_USART_OK;
    while (str[len] != '\0')
        len++;
    return BSP_K230_SendBuf((const uint8_t *)str, len);
}

uint8_t BSP_K230_GetRxByte(void)
{
    uint8_t ch = 0;

    if (g_k230RxTail != g_k230RxHead)
    {
        ch = g_k230RxBuf[g_k230RxTail];
        g_k230RxTail = (uint16_t)((g_k230RxTail + 1U) % K230_RX_BUF_SIZE);
    }
    return ch;
}

uint8_t BSP_K230_RxAvailable(void)
{
    return (g_k230RxHead != g_k230RxTail) ? 1U : 0U;
}

void BSP_K230_ClearRx(void)
{
    __disable_irq();
    g_k230RxHead = 0;
    g_k230RxTail = 0;
    __enable_irq();
}

uint32_t BSP_K230_GetOreCount(void) { return g_k230OreCount; }
uint32_t BSP_K230_GetRxDropCount(void) { return g_k230RxDropCount; }
uint32_t BSP_K230_GetTxDropCount(void) { return g_k230TxDropCount; }

void BSP_GPS_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(GPS_USART_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(GPS_USART_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPS_USART_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPS_USART_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPS_USART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPS_USART_GPIO, &GPIO_InitStructure);

    g_gpsRxHead = 0;
    g_gpsRxTail = 0;
    g_gpsOreCount = 0;
    g_gpsRxDropCount = 0;

    USART_CommonInit(GPS_USART, baudrate);

    NVIC_InitStructure.NVIC_IRQChannel = GPS_USART_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

uint8_t BSP_GPS_SendByte(uint8_t ch)
{
    uint32_t timeout = USART_TX_TIMEOUT_LOOPS;

    USART_SendData(GPS_USART, ch);
    while ((USART_GetFlagStatus(GPS_USART, USART_FLAG_TXE) == RESET) && (--timeout != 0U))
    {
    }
    return (timeout == 0U) ? BSP_USART_ERR_TX_TIMEOUT : BSP_USART_OK;
}

uint8_t BSP_GPS_SendBuf(const uint8_t *buf, uint16_t len)
{
    uint8_t result;

    if ((buf == 0) || (len == 0U))
        return BSP_USART_OK;
    while (len--)
    {
        result = BSP_GPS_SendByte(*buf++);
        if (result != BSP_USART_OK)
            return result;
    }
    return BSP_USART_OK;
}

uint8_t BSP_GPS_SendString(const char *str)
{
    uint16_t len = 0;

    if (str == 0)
        return BSP_USART_OK;
    while (str[len] != '\0')
        len++;
    return BSP_GPS_SendBuf((const uint8_t *)str, len);
}

uint8_t BSP_GPS_GetRxByte(void)
{
    uint8_t ch = 0;

    if (g_gpsRxTail != g_gpsRxHead)
    {
        ch = g_gpsRxBuf[g_gpsRxTail];
        g_gpsRxTail = (uint16_t)((g_gpsRxTail + 1U) % GPS_RX_BUF_SIZE);
    }
    return ch;
}

uint8_t BSP_GPS_RxAvailable(void)
{
    return (g_gpsRxHead != g_gpsRxTail) ? 1U : 0U;
}

void BSP_GPS_ClearRx(void)
{
    __disable_irq();
    g_gpsRxHead = 0;
    g_gpsRxTail = 0;
    __enable_irq();
}

uint32_t BSP_GPS_GetOreCount(void) { return g_gpsOreCount; }
uint32_t BSP_GPS_GetRxDropCount(void) { return g_gpsRxDropCount; }

void USART1_IRQHandler(void)
{
    uint16_t status = K230_USART->SR;
    uint8_t ch;

    if ((status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE)) != 0U)
    {
        ch = (uint8_t)K230_USART->DR;  /* SR then DR clears RX/error flags. */
        if ((status & USART_SR_RXNE) != 0U)
            K230_RxPush(ch);
        if ((status & USART_SR_ORE) != 0U)
            g_k230OreCount++;
    }

    if (((status & USART_SR_TXE) != 0U) && ((K230_USART->CR1 & USART_CR1_TXEIE) != 0U))
    {
        if (g_k230TxTail != g_k230TxHead)
        {
            K230_USART->DR = g_k230TxBuf[g_k230TxTail];
            g_k230TxTail = (uint16_t)((g_k230TxTail + 1U) % K230_TX_BUF_SIZE);
        }
        else
        {
            USART_ITConfig(K230_USART, USART_IT_TXE, DISABLE);
        }
    }
}

void USART2_IRQHandler(void)
{
    uint16_t status = GPS_USART->SR;
    uint8_t ch;

    if ((status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE)) != 0U)
    {
        ch = (uint8_t)GPS_USART->DR;   /* SR then DR clears RX/error flags. */
        if ((status & USART_SR_RXNE) != 0U)
            GPS_RxPush(ch);
        if ((status & USART_SR_ORE) != 0U)
            g_gpsOreCount++;
    }
}
