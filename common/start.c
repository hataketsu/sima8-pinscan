/* Startup shared by both ends: vector table, .data copy, .bss zero. */
#include <stdint.h>

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
int  main(void);
void Reset_Handler(void);
static void Default_Handler(void) { for (;;) { } }

void Reset_Handler(void)
{
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss; ) *dst++ = 0;
    main();
    for (;;) { }
}

__attribute__((section(".isr_vector"), used))
void (*const g_vectors[])(void) = {
    (void (*)(void))&_estack,
    Reset_Handler,
    Default_Handler,    /* NMI       */
    Default_Handler,    /* HardFault */
};

/* GCC emits calls to these for aggregate initialisers and struct copies even at
 * -nostdlib, so the freestanding builds have to supply them. */
void *memset(void *d, int c, unsigned long n)
{
    unsigned char *p = d;
    while (n--) *p++ = (unsigned char)c;
    return d;
}

void *memcpy(void *d, const void *s, unsigned long n)
{
    unsigned char *a = d;
    const unsigned char *b = s;
    while (n--) *a++ = *b++;
    return d;
}
