/**
 * Debug logging implementation — only compiled when DEBUG_ENABLE is defined.
 * Uses a minimal sprintf + USART1 TX for output.
 */

#include "bsp_debug.h"

#ifdef DEBUG_ENABLE

#include "bsp_usart.h"
#include <stdio.h>
#include <stdarg.h>

static char g_dbgBuf[128];

void DBG_Init(void)
{
    /* USART1 already initialized by BSP_USART_Init in main() */
}

void DBG_Print(const char *fmt, ...)
{
    va_list args;
    int len;

    va_start(args, fmt);
    len = vsnprintf(g_dbgBuf, sizeof(g_dbgBuf), fmt, args);
    va_end(args);

    if (len > 0)
    {
        BSP_USART_SendBuf((uint8_t *)g_dbgBuf, (uint16_t)len);
    }
}

#endif /* DEBUG_ENABLE */
