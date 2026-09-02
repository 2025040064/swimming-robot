#ifndef __BSP_DEBUG_H
#define __BSP_DEBUG_H

#include "stm32f10x.h"

/*
 * USART1 is exclusively the K230 protocol and must not carry printf text.
 * PA2/PA3 are free because this build has no GPS. Use SWD for debugging.
 */
#ifdef DEBUG_ENABLE
#error "DEBUG_ENABLE has no dedicated UART on this board; use SWD debugging instead."
#endif

#define DBG_INIT()                  ((void)0)
#define DBG_PRINT(...)               ((void)0)
#define DBG_STATE(old, new)          ((void)0)
#define DBG_SENSOR(name, val)        ((void)0)
#define DBG_ASSERT(cond)             ((void)0)

#endif
