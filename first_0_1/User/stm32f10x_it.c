/**
 * Fault handlers with crash-dump to BKP registers and auto-reset.
 *
 * On HardFault / MemManage / BusFault / UsageFault:
 *   1. Save SP, LR, PC to BKP_DR1~DR3 (survive reset)
 *   2. Blink LED at 5Hz (fast) to signal fault
 *   3. Wait 3 seconds for debugger attach
 *   4. Trigger system reset via NVIC
 *
 * After reset, check BKP_DR4 for the magic crash marker to know
 * the reset was fault-induced.
 */

#include "stm32f10x_it.h"
#include "bsp/bsp_led.h"
#include "bsp/bsp_systick.h"

#define CRASH_MARKER    0xDEADBEEF

/* Save crash context to backup registers (powered by VBAT, survive reset) */
static void Crash_SaveContext(uint32_t *stack)
{
    /* Wait for BKP/TAMPER clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP | RCC_APB1Periph_PWR, ENABLE);
    PWR_BackupAccessCmd(ENABLE);

    BKP_WriteBackupRegister(BKP_DR1, (uint32_t)stack);       /* SP at fault */
    BKP_WriteBackupRegister(BKP_DR2, stack[5]);               /* LR (EXC_RETURN) */
    BKP_WriteBackupRegister(BKP_DR3, stack[6]);               /* PC (stacked) */
    BKP_WriteBackupRegister(BKP_DR4, CRASH_MARKER);           /* Crash signature */
}

/* Fast LED blink: 5Hz = 100ms period, 50ms on/off */
static void Crash_BlinkLED(void)
{
    uint32_t tick = BSP_GetTick();
    uint32_t phase = tick % 200;
    if (phase < 100)
        BSP_LED_On();
    else
        BSP_LED_Off();
}

static void Crash_Reset(void)
{
    uint32_t start = BSP_GetTick();

    while ((BSP_GetTick() - start) < 3000)
    {
        Crash_BlinkLED();
    }

    /* System reset */
    NVIC_SystemReset();
}

/* ---- Cortex-M3 fault handlers ---- */

void NMI_Handler(void)
{
    /* Non-maskable interrupt — may indicate critical hardware issue */
    while (1);
}

__attribute__((naked))
void HardFault_Handler(void)
{
    __asm volatile (
        "MRS R0, MSP\n"
        "B  HardFault_Handler_C\n"
    );
}

void HardFault_Handler_C(uint32_t *stack)
{
    Crash_SaveContext(stack);
    Crash_Reset();
}

__attribute__((naked))
void MemManage_Handler(void)
{
    __asm volatile (
        "MRS R0, MSP\n"
        "B  MemManage_Handler_C\n"
    );
}

void MemManage_Handler_C(uint32_t *stack)
{
    Crash_SaveContext(stack);
    Crash_Reset();
}

__attribute__((naked))
void BusFault_Handler(void)
{
    __asm volatile (
        "MRS R0, MSP\n"
        "B  BusFault_Handler_C\n"
    );
}

void BusFault_Handler_C(uint32_t *stack)
{
    Crash_SaveContext(stack);
    Crash_Reset();
}

__attribute__((naked))
void UsageFault_Handler(void)
{
    __asm volatile (
        "MRS R0, MSP\n"
        "B  UsageFault_Handler_C\n"
    );
}

void UsageFault_Handler_C(uint32_t *stack)
{
    Crash_SaveContext(stack);
    Crash_Reset();
}

void SVC_Handler(void)            {}
void DebugMon_Handler(void)       {}
void PendSV_Handler(void)         {}
