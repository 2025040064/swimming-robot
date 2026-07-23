#include "Delay.h"
#include "stm32f10x.h"

/**
  * @brief  DWT (Data Watchpoint and Trace) 初始化
  * @note   使能 DWT 周期计数器，用于微秒级精确定时。
  *         该计数器独立于 SysTick，不产生中断、不修改任何
  *         系统寄存器，因此不会破坏 1ms 系统时基。
  * @retval 无
  */
static void DWT_Init(void)
{
    /* 使能 TRC (Trace) 调试接口 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* 清零周期计数器 */
    DWT->CYCCNT = 0;

    /* 使能周期计数器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
  * @brief  获取 CPU 周期计数器当前值
  * @retval 自启动以来的 CPU 周期数（@72MHz，约 59.6 秒回绕一次）
  */
static uint32_t DWT_GetCycles(void)
{
    return DWT->CYCCNT;
}

/**
  * @brief  微秒级阻塞延时（不破坏 SysTick 时基）
  * @param  xus: 延时时长（微秒），范围取决于调用场景
  *         - 72MHz 下每微秒 72 个 CPU 周期
  *         - 单次调用最长约 59.6 秒（32 位计数器回绕限制）
  * @note   本函数不操作 SysTick 寄存器。
  *         内部使用 DWT 周期计数器，与项目中的 1ms 协程调度
  *         器完全独立。但注意这是阻塞延时——延时期间 CPU 不
  *         执行其他任务。
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
    static uint8_t initialized = 0;
    uint32_t start, target;

    if (!initialized)
    {
        DWT_Init();
        initialized = 1;
    }

    start  = DWT_GetCycles();
    target = xus * (SystemCoreClock / 1000000U);  /* 72 cycles/us @72MHz */

    while ((DWT_GetCycles() - start) < target);
}

/**
  * @brief  毫秒级阻塞延时
  * @param  xms: 延时时长（毫秒）
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
    while (xms--)
    {
        Delay_us(1000);
    }
}

/**
  * @brief  秒级阻塞延时
  * @param  xs: 延时时长（秒）
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
    while (xs--)
    {
        Delay_ms(1000);
    }
}
