/**
 * Fault handlers with crash-dump to BKP registers and auto-reset.
 *
 * On HardFault / MemManage / BusFault / UsageFault:
 *   1. Save SP, LR, PC to BKP_DR1~DR6 (each 32-bit value split across two
 *      16-bit backup registers, so it survives reset via VBAT).
 *   2. Write a 16-bit crash signature to BKP_DR7.
 *   3. Trigger system reset via NVIC immediately.
 *
 * After reset, Crash_ReportAndClear() clears the fault signature without using
 * a UART. USART1 is reserved for the K230 protocol and USART2 for GPS, so
 * unsolicited crash text must not be injected into either device link.
 *
 * NOTE: the reset must be immediate — a fault handler runs at a higher
 * exception priority than SysTick, so any SysTick-based delay would hang.
 */

#include "stm32f10x_it.h"

#define CRASH_MARKER    0xDEAD   /* 16-bit: BKP registers on F103 are 16-bit wide */

/* --- 32-bit value <-> pair of 16-bit BKP registers --- */
static void BKP_Write32(uint16_t regHi, uint16_t regLo, uint32_t val)
{
    BKP_WriteBackupRegister(regHi, (uint16_t)(val >> 16));
    BKP_WriteBackupRegister(regLo, (uint16_t)(val & 0xFFFF));
}

/* Save crash context to backup registers (powered by VBAT, survive reset) */
static void Crash_SaveContext(uint32_t *stack)
{
    /* Enable BKP/TAMPER clock and backup write access */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP | RCC_APB1Periph_PWR, ENABLE);
    PWR_BackupAccessCmd(ENABLE);

    BKP_Write32(BKP_DR1, BKP_DR2, (uint32_t)stack);   /* SP at fault */
    BKP_Write32(BKP_DR3, BKP_DR4, stack[5]);           /* LR (EXC_RETURN) */
    BKP_Write32(BKP_DR5, BKP_DR6, stack[6]);           /* PC (stacked) */
    BKP_WriteBackupRegister(BKP_DR7, CRASH_MARKER);    /* Crash signature */
}

static void Crash_Reset(void)
{
    /* Do NOT delay via SysTick here: a fault handler runs at higher priority
     * than SysTick, so SysTick never fires and a BSP_GetTick()-based wait would
     * hang forever. Reset immediately after saving context. */
    NVIC_SystemReset();
}

/* Called once at boot. Keep dedicated K230/GPS links protocol-clean. */
void Crash_ReportAndClear(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP | RCC_APB1Periph_PWR, ENABLE);
    PWR_BackupAccessCmd(ENABLE);

    if (BKP_ReadBackupRegister(BKP_DR7) != CRASH_MARKER)
        return;

    BKP_WriteBackupRegister(BKP_DR7, 0);   /* clear signature */
}

/* ---- Cortex-M3 fault handlers ---- */

void NMI_Handler(void)
{
    /* Non-maskable interrupt — may indicate critical hardware issue */
    while (1);
}

/* ---- Fault handler trampolines: read MSP then call C handler ---- */

#if defined(__CC_ARM)
/* ARM Compiler (Keil MDK): embedded assembler functions are implicitly naked */

__asm void HardFault_Handler(void)
{
    IMPORT HardFault_Handler_C
    MRS R0, MSP
    B  HardFault_Handler_C
    ALIGN
}

void HardFault_Handler_C(uint32_t *stack)
{
    Crash_SaveContext(stack);
    Crash_Reset();
}

__asm void MemManage_Handler(void)
{
    IMPORT MemManage_Handler_C
    MRS R0, MSP
    B  MemManage_Handler_C
    ALIGN
}

void MemManage_Handler_C(uint32_t *stack)
{
    Crash_SaveContext(stack);
    Crash_Reset();
}

__asm void BusFault_Handler(void)
{
    IMPORT BusFault_Handler_C
    MRS R0, MSP
    B  BusFault_Handler_C
    ALIGN
}

void BusFault_Handler_C(uint32_t *stack)
{
    Crash_SaveContext(stack);
    Crash_Reset();
}

__asm void UsageFault_Handler(void)
{
    IMPORT UsageFault_Handler_C
    MRS R0, MSP
    B  UsageFault_Handler_C
    ALIGN
}

void UsageFault_Handler_C(uint32_t *stack)
{
    Crash_SaveContext(stack);
    Crash_Reset();
}

#elif defined(__GNUC__)
/* GCC: use naked attribute + inline assembly */

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

#else
/* Fallback: assume the compiler can't do naked — just call directly */
/* (stack pointer will be slightly off due to prologue, but still useful) */

void HardFault_Handler(void)
{
    uint32_t stack;
    __ASM volatile ("MRS %0, MSP" : "=r" (stack));
    HardFault_Handler_C((uint32_t *)stack);
}

void HardFault_Handler_C(uint32_t *stack)
{
    Crash_SaveContext(stack);
    Crash_Reset();
}

void MemManage_Handler(void)
{
    uint32_t stack;
    __ASM volatile ("MRS %0, MSP" : "=r" (stack));
    MemManage_Handler_C((uint32_t *)stack);
}

void MemManage_Handler_C(uint32_t *stack)
{
    Crash_SaveContext(stack);
    Crash_Reset();
}

void BusFault_Handler(void)
{
    uint32_t stack;
    __ASM volatile ("MRS %0, MSP" : "=r" (stack));
    BusFault_Handler_C((uint32_t *)stack);
}

void BusFault_Handler_C(uint32_t *stack)
{
    Crash_SaveContext(stack);
    Crash_Reset();
}

void UsageFault_Handler(void)
{
    uint32_t stack;
    __ASM volatile ("MRS %0, MSP" : "=r" (stack));
    UsageFault_Handler_C((uint32_t *)stack);
}

void UsageFault_Handler_C(uint32_t *stack)
{
    Crash_SaveContext(stack);
    Crash_Reset();
}

#endif

void SVC_Handler(void)            {}
void DebugMon_Handler(void)       {}
void PendSV_Handler(void)         {}
