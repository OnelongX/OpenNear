/*
 * Minimal Standalone ARM Cortex-M4 Startup & Vector Table for Nordic nRF52840
 * Links directly with nrf52840.ld to produce flashable opennear_nrf52.elf / .hex.
 */
#include <stdint.h>

extern uint32_t _estack;
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
extern int main(void);

void Default_Handler(void) {
    for (;;) { }
}

void Reset_Handler(void) {
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }
    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0u;
    }
    (void)main();
    for (;;) { }
}

void NMI_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void RADIO_IRQHandler(void)   __attribute__((weak, alias("Default_Handler")));
void TIMER0_IRQHandler(void)  __attribute__((weak, alias("Default_Handler")));
void RTC0_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));

typedef void (*irq_fn_t)(void);

__attribute__((section(".isr_vector"), used))
const irq_fn_t g_nrf52840_vectors[32] = {
    (irq_fn_t)(uintptr_t)&_estack, /* 0: Initial SP */
    Reset_Handler,                 /* 1: Reset */
    NMI_Handler,                   /* 2: NMI */
    HardFault_Handler,             /* 3: HardFault */
    Default_Handler,               /* 4..15: Core exceptions */
    Default_Handler,
    Default_Handler,
    0, 0, 0, 0,
    Default_Handler,
    Default_Handler,
    0,
    Default_Handler,
    Default_Handler,
    /* External IRQs (nRF52840 IRQ 0..15) */
    Default_Handler,               /* IRQ0:  POWER_CLOCK */
    RADIO_IRQHandler,              /* IRQ1:  RADIO (OpenNear PPI RX END) */
    Default_Handler,               /* IRQ2:  UARTE0 */
    Default_Handler,               /* IRQ3:  SPIM0 */
    Default_Handler,               /* IRQ4:  SPIM1 */
    Default_Handler,               /* IRQ5:  NFCT */
    Default_Handler,               /* IRQ6:  GPIOTE */
    Default_Handler,               /* IRQ7:  SAADC */
    TIMER0_IRQHandler,             /* IRQ8:  TIMER0 */
    Default_Handler,               /* IRQ9:  TIMER1 */
    Default_Handler,               /* IRQ10: TIMER2 */
    RTC0_IRQHandler,               /* IRQ11: RTC0 (32.768 kHz sleep wakeup) */
    Default_Handler,               /* IRQ12: TEMP */
    Default_Handler,               /* IRQ13: RNG */
    Default_Handler,               /* IRQ14: ECB */
    Default_Handler                /* IRQ15: CCM_AAR */
};
