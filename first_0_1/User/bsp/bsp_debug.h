/**
 * Debug logging system — compile-time zero-cost when disabled.
 *
 * Usage:
 *   In stm32f10x_conf.h, define DEBUG_ENABLE to activate logging.
 *   Production builds: leave DEBUG_ENABLE undefined — all macros expand to nothing.
 *
 * Macros:
 *   DBG_PRINT(fmt, ...)    — printf-style to USART1
 *   DBG_STATE(old, new)    — log state machine transitions
 *   DBG_SENSOR(name, val)  — log sensor reading
 *   DBG_ASSERT(cond)       — halt if condition false (debug only)
 */

#ifndef __BSP_DEBUG_H
#define __BSP_DEBUG_H

#include "stm32f10x.h"

/* ---- Set to 1 to enable debug output, 0 for production ---- */
#ifdef DEBUG_ENABLE

#include <stdio.h>

void DBG_Init(void);
void DBG_Print(const char *fmt, ...);

#define DBG_PRINT(fmt, ...)         DBG_Print(fmt, ##__VA_ARGS__)
#define DBG_STATE(old, new)         DBG_Print("[SM] %s -> %s\n", #old, #new)
#define DBG_SENSOR(name, val)       DBG_Print("[SENSOR] %s=%.1f\n", name, (double)(val))
#define DBG_ASSERT(cond)            do { if (!(cond)) { DBG_Print("[ASSERT] %s:%d %s\n", __FILE__, __LINE__, #cond); while(1); } } while(0)

#else

#define DBG_INIT()                  ((void)0)
#define DBG_PRINT(fmt, ...)         ((void)0)
#define DBG_STATE(old, new)         ((void)0)
#define DBG_SENSOR(name, val)       ((void)0)
#define DBG_ASSERT(cond)            ((void)0)

#endif /* DEBUG_ENABLE */

#endif /* __BSP_DEBUG_H */
