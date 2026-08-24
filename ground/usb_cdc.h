/* Minimal polled USB CDC-ACM for STM32F103. */
#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdint.h>

void usb_cdc_init(void);
void usb_cdc_poll(void);          /* call often from the main loop */
uint32_t usb_cdc_ready(void);     /* 1 once the host has configured us */
void usb_cdc_putc(char c);
void usb_cdc_write(const char *s);
int  usb_cdc_getc(void);          /* -1 when nothing is buffered */

/* Set by a 1200-baud open, the convention hosts use to ask for the bootloader. */
extern volatile uint32_t usb_cdc_reboot_request;

#endif
