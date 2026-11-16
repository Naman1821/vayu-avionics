/*
 * startup.c - what runs before main(): the vector table and the reset handler.
 */
#include <stdint.h>

#include "stm32f405.h"

extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
extern int main(void);

void Reset_Handler(void);
void Default_Handler(void);
void HardFault_Handler(void);
void SysTick_Handler(void);
void PVD_IRQHandler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);

void Reset_Handler(void)
{
    /* copy initialised variables from flash to RAM, zero the rest */
    uint32_t *src = &_sidata;
    for (uint32_t *dst = &_sdata; dst < &_edata;)
        *dst++ = *src++;
    for (uint32_t *dst = &_sbss; dst < &_ebss;)
        *dst++ = 0;

    /* turn on the FPU before any float maths (altitude uses logf/powf) */
    SCB_CPACR |= (0xFu << 20);
    __asm volatile("dsb\n isb");

    main();
    for (;;) {
    }
}

void Default_Handler(void)
{
    for (;;) { /* unexpected interrupt: the watchdog will reset us */
    }
}

void HardFault_Handler(void)
{
    for (;;) { /* e.g. a corrupted pointer (radiation at altitude): the watchdog resets us */
    }
}

/* Cortex-M4 core exceptions (16) followed by the STM32F405's 82 interrupts. */
__attribute__((section(".isr_vector"), used))
void (*const vector_table[16 + 82])(void) = {
    [0]  = (void (*)(void))(&_estack),
    [1]  = Reset_Handler,
    [2]  = Default_Handler,   /* NMI        */
    [3]  = HardFault_Handler,
    [4]  = Default_Handler,   /* MemManage  */
    [5]  = Default_Handler,   /* BusFault   */
    [6]  = Default_Handler,   /* UsageFault */
    [11] = Default_Handler,   /* SVCall     */
    [12] = Default_Handler,   /* DebugMon   */
    [14] = Default_Handler,   /* PendSV     */
    [15] = SysTick_Handler,
    [16 + PVD_IRQn]    = PVD_IRQHandler,
    [16 + USART1_IRQn] = USART1_IRQHandler,
    [16 + USART2_IRQn] = USART2_IRQHandler,
};
