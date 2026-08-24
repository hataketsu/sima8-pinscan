/* Minimal polled USB CDC-ACM for STM32F103.
 *
 * No interrupts: the main loop already polls everything else, and USB on this
 * part is slow enough that ISTR can be serviced the same way. That keeps the
 * vector table irrelevant, which matters because the application runs behind a
 * bootloader and inherits VTOR.
 *
 * Endpoints
 *   0  control, 64 bytes each way
 *   1  interrupt IN, the notification endpoint CDC requires but nothing reads
 *   2  bulk OUT, host to device
 *   3  bulk IN,  device to host
 *
 * Packet memory is 512 bytes seen by the peripheral, but the CPU sees each
 * 16-bit word in a 32-bit slot, so a PMA byte offset maps to PMA32[off / 2].
 */
#include "usb_cdc.h"

#define USB_BASE   0x40005C00u
#define EPR(n)     (*(volatile uint32_t *)(USB_BASE + (n) * 4u))
#define USB_CNTR   (*(volatile uint32_t *)(USB_BASE + 0x40u))
#define USB_ISTR   (*(volatile uint32_t *)(USB_BASE + 0x44u))
#define USB_DADDR  (*(volatile uint32_t *)(USB_BASE + 0x4Cu))
#define USB_BTABLE (*(volatile uint32_t *)(USB_BASE + 0x50u))
#define PMA32      ((volatile uint32_t *)0x40006000u)

#define RCC_APB1ENR (*(volatile uint32_t *)0x4002101Cu)
#define RCC_APB2ENR (*(volatile uint32_t *)0x40021018u)
#define USBEN       (1u << 23)
#define IOPAEN      (1u << 2)
#define GPIOA_CRH   (*(volatile uint32_t *)0x40010804u)
#define GPIOA_BRR   (*(volatile uint32_t *)0x40010814u)

/* EPnR is a minefield: CTR_RX/CTR_TX are cleared by writing 0 and kept by
 * writing 1, while DTOG/STAT are toggled by writing 1 and kept by writing 0.
 * Every write goes through these helpers so the rules live in one place. */
#define EP_RW    0x0F0Fu     /* EA, EP_KIND, EP_TYPE, SETUP */
#define EP_KEEP  0x8080u     /* write 1 to leave CTR_RX / CTR_TX alone */

#define STAT_DISABLED 0u
#define STAT_STALL    1u
#define STAT_NAK      2u
#define STAT_VALID    3u

static void ep_stat_rx(uint32_t ep, uint32_t stat)
{
    uint32_t cur = EPR(ep);
    uint32_t tog = (((cur >> 12) & 3u) ^ stat) << 12;
    EPR(ep) = (cur & EP_RW) | EP_KEEP | tog;
}
static void ep_stat_tx(uint32_t ep, uint32_t stat)
{
    uint32_t cur = EPR(ep);
    uint32_t tog = (((cur >> 4) & 3u) ^ stat) << 4;
    EPR(ep) = (cur & EP_RW) | EP_KEEP | tog;
}
static void ep_clear_ctr_rx(uint32_t ep) { EPR(ep) = (EPR(ep) & EP_RW) | 0x0080u; }
static void ep_clear_ctr_tx(uint32_t ep) { EPR(ep) = (EPR(ep) & EP_RW) | 0x8000u; }

/* PMA layout, byte offsets */
#define BT_BASE   0x000u
#define EP0_TX    0x040u
#define EP0_RX    0x080u
#define EP1_TX    0x0C0u
#define EP2_RX    0x0D0u
#define EP3_TX    0x110u
#define EP_MAX    64u

static void pma_write(uint32_t off, const uint8_t *src, uint32_t n)
{
    volatile uint32_t *p = &PMA32[off / 2u];
    for (uint32_t i = 0; i < n; i += 2u) {
        uint32_t lo = src[i];
        uint32_t hi = (i + 1u < n) ? src[i + 1u] : 0u;
        *p++ = lo | (hi << 8);
    }
}
static void pma_read(uint32_t off, uint8_t *dst, uint32_t n)
{
    volatile uint32_t *p = &PMA32[off / 2u];
    for (uint32_t i = 0; i < n; i += 2u) {
        uint32_t v = *p++;
        dst[i] = (uint8_t)v;
        if (i + 1u < n) dst[i + 1u] = (uint8_t)(v >> 8);
    }
}
static void bt_set(uint32_t ep, uint32_t addr_tx, uint32_t count_tx,
                   uint32_t addr_rx, uint32_t count_rx_reg)
{
    PMA32[BT_BASE / 2u + ep * 4u + 0u] = addr_tx;
    PMA32[BT_BASE / 2u + ep * 4u + 1u] = count_tx;
    PMA32[BT_BASE / 2u + ep * 4u + 2u] = addr_rx;
    PMA32[BT_BASE / 2u + ep * 4u + 3u] = count_rx_reg;
}
static void bt_tx_count(uint32_t ep, uint32_t n) { PMA32[BT_BASE / 2u + ep * 4u + 1u] = n; }
static uint32_t bt_rx_count(uint32_t ep) { return PMA32[BT_BASE / 2u + ep * 4u + 3u] & 0x3FFu; }

/* 64-byte RX buffer: 32-byte blocks, two of them */
#define RXCOUNT_64 (0x8000u | (1u << 10))

/* ---- descriptors ---- */
static const uint8_t DEV_DESC[18] = {
    18, 1, 0x00, 0x02, 0x02, 0x00, 0x00, EP_MAX,
    0x83, 0x04,          /* 0x0483 STMicroelectronics */
    0x40, 0x57,          /* 0x5740 virtual COM, so hosts bind their stock driver */
    0x00, 0x02, 1, 2, 3, 1
};

#define CONF_LEN 67
static const uint8_t CONF_DESC[CONF_LEN] = {
    9, 2, CONF_LEN, 0, 2, 1, 0, 0xC0, 50,
    /* comm interface */
    9, 4, 0, 0, 1, 0x02, 0x02, 0x01, 0,
    5, 0x24, 0x00, 0x10, 0x01,          /* header */
    5, 0x24, 0x01, 0x00, 1,             /* call management */
    4, 0x24, 0x02, 0x02,                /* ACM, supports line coding */
    5, 0x24, 0x06, 0, 1,                /* union */
    7, 5, 0x81, 0x03, 8, 0, 16,         /* EP1 IN interrupt */
    /* data interface */
    9, 4, 1, 0, 2, 0x0A, 0x00, 0x00, 0,
    7, 5, 0x02, 0x02, EP_MAX, 0, 0,     /* EP2 OUT bulk */
    7, 5, 0x83, 0x02, EP_MAX, 0, 0      /* EP3 IN bulk  */
};

static const uint8_t STR0[4]  = { 4, 3, 0x09, 0x04 };
static const uint8_t STR1[16] = { 16, 3, 's',0,'i',0,'m',0,'a',0,'8',0,' ',0,' ',0 };
static const uint8_t STR2[22] = { 22, 3, 'g',0,'r',0,'o',0,'u',0,'n',0,'d',0,' ',0,'t',0,'x',0,' ',0 };
static const uint8_t STR3[10] = { 10, 3, '0',0,'0',0,'0',0,'1',0 };

/* ---- state ---- */
volatile uint32_t usb_cdc_reboot_request;
static uint32_t configured;
static uint8_t  pending_addr;
static const uint8_t *ctl_src;
static uint32_t ctl_left;
static uint8_t  line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0, 0, 8 };  /* 115200 8N1 */

#define RB_SIZE 256
static volatile uint8_t rxb[RB_SIZE]; static volatile uint32_t rx_head, rx_tail;
static volatile uint8_t txb[RB_SIZE]; static volatile uint32_t tx_head, tx_tail;
static uint32_t tx_busy;

static void ctl_send_chunk(void)
{
    uint32_t n = ctl_left > EP_MAX ? EP_MAX : ctl_left;
    pma_write(EP0_TX, ctl_src, n);
    bt_tx_count(0, n);
    ctl_src  += n;
    ctl_left -= n;
    ep_stat_tx(0, STAT_VALID);
}
static void ctl_send(const uint8_t *d, uint32_t n, uint32_t wanted)
{
    ctl_src = d;
    ctl_left = (n < wanted) ? n : wanted;
    ctl_send_chunk();
}
static void ctl_ack(void) { bt_tx_count(0, 0); ep_stat_tx(0, STAT_VALID); }

static void handle_setup(void)
{
    uint8_t s[8];
    pma_read(EP0_RX, s, 8);
    uint32_t type = s[0], req = s[1];
    uint32_t val = (uint32_t)s[2] | ((uint32_t)s[3] << 8);
    uint32_t len = (uint32_t)s[6] | ((uint32_t)s[7] << 8);

    if ((type & 0x60) == 0) {                      /* standard */
        switch (req) {
        case 6:                                    /* GET_DESCRIPTOR */
            switch (val >> 8) {
            case 1: ctl_send(DEV_DESC, sizeof DEV_DESC, len); break;
            case 2: ctl_send(CONF_DESC, CONF_LEN, len); break;
            case 3:
                switch (val & 0xFF) {
                case 0: ctl_send(STR0, sizeof STR0, len); break;
                case 1: ctl_send(STR1, sizeof STR1, len); break;
                case 2: ctl_send(STR2, sizeof STR2, len); break;
                default: ctl_send(STR3, sizeof STR3, len); break;
                }
                break;
            default: ep_stat_tx(0, STAT_STALL); break;
            }
            break;
        case 5:                                    /* SET_ADDRESS, applied after the status stage */
            pending_addr = (uint8_t)(val & 0x7F);
            ctl_ack();
            break;
        case 9:                                    /* SET_CONFIGURATION */
            configured = (val != 0);
            ctl_ack();
            break;
        case 0: { static const uint8_t z[2] = { 0, 0 }; ctl_send(z, 2, len); break; }
        default: ctl_ack(); break;
        }
    } else {                                       /* class: CDC */
        switch (req) {
        case 0x20:                                 /* SET_LINE_CODING, data follows */
            ctl_ack();
            break;
        case 0x21:                                 /* GET_LINE_CODING */
            ctl_send(line_coding, 7, len);
            break;
        case 0x22:                                 /* SET_CONTROL_LINE_STATE */
            ctl_ack();
            break;
        default:
            ctl_ack();
            break;
        }
    }
    ep_stat_rx(0, STAT_VALID);
}

static void handle_ep0_out(void)
{
    uint32_t n = bt_rx_count(0);
    if (n == 7) {
        /* the data stage of SET_LINE_CODING */
        pma_read(EP0_RX, line_coding, 7);
        uint32_t baud = (uint32_t)line_coding[0] | ((uint32_t)line_coding[1] << 8)
                      | ((uint32_t)line_coding[2] << 16) | ((uint32_t)line_coding[3] << 24);
        /* 1200 baud is the convention for "reopen in the bootloader" */
        if (baud == 1200u) usb_cdc_reboot_request = 1;
        ctl_ack();
    }
    ep_stat_rx(0, STAT_VALID);
}

static void tx_pump(void)
{
    if (!configured || tx_busy) return;
    if (tx_head == tx_tail) return;
    uint8_t tmp[EP_MAX];
    uint32_t n = 0;
    while (n < EP_MAX && tx_head != tx_tail) {
        tmp[n++] = txb[tx_tail];
        tx_tail = (tx_tail + 1u) % RB_SIZE;
    }
    pma_write(EP3_TX, tmp, n);
    bt_tx_count(3, n);
    tx_busy = 1;
    ep_stat_tx(3, STAT_VALID);
}

void usb_cdc_init(void)
{
    RCC_APB2ENR |= IOPAEN;

    /* Force the host to re-enumerate. Coming out of the bootloader the device
     * is already known to the host, and without a visible disconnect it never
     * asks for our descriptors. Driving D+ low for a while is the disconnect. */
    GPIOA_CRH = (GPIOA_CRH & ~(0xFu << 16)) | (0x3u << 16);   /* PA12 output PP */
    GPIOA_BRR = (1u << 12);
    for (volatile uint32_t i = 0; i < 700000u; i++) { }
    GPIOA_CRH = (GPIOA_CRH & ~(0xFu << 16)) | (0x4u << 16);   /* back to floating */

    RCC_APB1ENR |= USBEN;
    USB_CNTR = (1u << 1);                 /* PDWN clear, FRES set */
    for (volatile uint32_t i = 0; i < 1000u; i++) { }
    USB_CNTR = 0;                         /* release reset */
    USB_ISTR = 0;
    USB_BTABLE = BT_BASE;
    USB_DADDR = (1u << 7);                /* enable, address 0 */
    configured = 0;
    tx_busy = 0;
    rx_head = rx_tail = tx_head = tx_tail = 0;
}

static void on_reset(void)
{
    USB_BTABLE = BT_BASE;

    bt_set(0, EP0_TX, 0, EP0_RX, RXCOUNT_64);
    EPR(0) = (1u << 9) | 0u;              /* control, address 0 */
    ep_stat_rx(0, STAT_VALID);
    ep_stat_tx(0, STAT_NAK);

    bt_set(1, EP1_TX, 0, 0, 0);
    EPR(1) = (3u << 9) | 1u;              /* interrupt */
    ep_stat_tx(1, STAT_NAK);
    ep_stat_rx(1, STAT_DISABLED);

    bt_set(2, 0, 0, EP2_RX, RXCOUNT_64);
    EPR(2) = (0u << 9) | 2u;              /* bulk */
    ep_stat_rx(2, STAT_VALID);
    ep_stat_tx(2, STAT_DISABLED);

    bt_set(3, EP3_TX, 0, 0, 0);
    EPR(3) = (0u << 9) | 3u;              /* bulk */
    ep_stat_tx(3, STAT_NAK);
    ep_stat_rx(3, STAT_DISABLED);

    USB_DADDR = (1u << 7);
    configured = 0;
    tx_busy = 0;
}

void usb_cdc_poll(void)
{
    uint32_t istr = USB_ISTR;

    if (istr & (1u << 10)) {              /* RESET */
        USB_ISTR = ~(1u << 10);
        on_reset();
        return;
    }

    if (istr & (1u << 15)) {              /* CTR: a transfer completed */
        uint32_t ep = istr & 0xFu;
        uint32_t epr = EPR(ep);

        if (ep == 0) {
            if (epr & (1u << 15)) {       /* CTR_RX */
                uint32_t setup = epr & (1u << 11);
                ep_clear_ctr_rx(0);
                if (setup) handle_setup(); else handle_ep0_out();
            }
            if (epr & (1u << 7)) {        /* CTR_TX */
                ep_clear_ctr_tx(0);
                if (pending_addr) {
                    USB_DADDR = (1u << 7) | pending_addr;
                    pending_addr = 0;
                }
                if (ctl_left) ctl_send_chunk();
                else ep_stat_rx(0, STAT_VALID);
            }
        } else if (ep == 2) {
            if (epr & (1u << 15)) {
                uint32_t n = bt_rx_count(2);
                uint8_t tmp[EP_MAX];
                pma_read(EP2_RX, tmp, n);
                ep_clear_ctr_rx(2);
                for (uint32_t i = 0; i < n; i++) {
                    uint32_t nh = (rx_head + 1u) % RB_SIZE;
                    if (nh != rx_tail) { rxb[rx_head] = tmp[i]; rx_head = nh; }
                }
                ep_stat_rx(2, STAT_VALID);
            }
        } else if (ep == 3) {
            if (epr & (1u << 7)) {
                ep_clear_ctr_tx(3);
                tx_busy = 0;
            }
        } else {
            ep_clear_ctr_rx(ep);
            ep_clear_ctr_tx(ep);
        }
    }

    tx_pump();
}

uint32_t usb_cdc_ready(void) { return configured; }

void usb_cdc_putc(char c)
{
    uint32_t nh = (tx_head + 1u) % RB_SIZE;
    if (nh == tx_tail) return;            /* drop rather than block the flight loop */
    txb[tx_head] = (uint8_t)c;
    tx_head = nh;
}
void usb_cdc_write(const char *s) { while (*s) usb_cdc_putc(*s++); }

int usb_cdc_getc(void)
{
    if (rx_head == rx_tail) return -1;
    int c = rxb[rx_tail];
    rx_tail = (rx_tail + 1u) % RB_SIZE;
    return c;
}
