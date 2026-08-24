/* Ground unit — transmitter end of the link.
 *
 * STM32F103C8T6 "Blue Pill" at the 8 MHz HSI, nRF24L01+ on hardware SPI1:
 *   PA4 CSN   PA5 SCK   PA6 MISO   PA7 MOSI   PB0 CE   PA1 IRQ (unused)
 *   PA9 USART1_TX at 9600 for debug
 *
 * Unlike the drone, CE here is a real pin, so transmission is the usual
 * sequence: load the FIFO, pulse CE, wait for TX_DS.
 *
 * There are no sticks wired yet, so this sends a fixed neutral frame with the
 * arm bit clear and throttle at zero. It exists to prove the link, not to fly.
 */
#include <stdint.h>
#include "../common/protocol.h"
#include "usb_cdc.h"

#define F_CPU   72000000u          /* HSE 8 MHz through PLL x9 */
#define F_PCLK2 72000000u          /* APB2 undivided, feeds USART1 */

#define RCC_APB1ENR (*(volatile uint32_t *)0x4002101Cu)
#define PWR_CR      (*(volatile uint32_t *)0x40007000u)
#define BKP_DR4     (*(volatile uint32_t *)0x40006C10u)
#define SCB_AIRCR   (*(volatile uint32_t *)0xE000ED0Cu)
#define RCC_CR      (*(volatile uint32_t *)0x40021000u)
#define RCC_CFGR    (*(volatile uint32_t *)0x40021004u)
#define FLASH_ACR   (*(volatile uint32_t *)0x40022000u)
#define RCC_APB2ENR (*(volatile uint32_t *)0x40021018u)
#define IOPAEN      (1u << 2)
#define IOPBEN      (1u << 3)
#define AFIOEN      (1u << 0)
#define SPI1EN      (1u << 12)
#define USART1EN    (1u << 14)

#define STK_CTRL  (*(volatile uint32_t *)0xE000E010u)
#define STK_LOAD  (*(volatile uint32_t *)0xE000E014u)
#define STK_VAL   (*(volatile uint32_t *)0xE000E018u)

#define SPI1_CR1  (*(volatile uint32_t *)0x40013000u)
#define SPI1_SR   (*(volatile uint32_t *)0x40013008u)
#define SPI1_DR   (*(volatile uint32_t *)0x4001300Cu)

#define USART1_SR  (*(volatile uint32_t *)0x40013800u)
#define USART1_DR  (*(volatile uint32_t *)0x40013804u)
#define USART1_BRR (*(volatile uint32_t *)0x40013808u)
#define USART1_CR1 (*(volatile uint32_t *)0x4001380Cu)

/* F1 GPIO is configured through CRL/CRH nibbles, not MODER */
typedef struct {
    volatile uint32_t CRL, CRH, IDR, ODR, BSRR, BRR, LCKR;
} gpio_t;
#define GPIOA ((gpio_t *)0x40010800u)
#define GPIOB ((gpio_t *)0x40010C00u)

#define CNF_OUT_PP   0x3u    /* output 50 MHz, push-pull       */
#define CNF_AF_PP    0xBu    /* output 50 MHz, alternate func  */
#define CNF_IN_FLOAT 0x4u    /* input, floating                */

#define PIN_CSN 4
#define PIN_CE  0            /* on port B */
#define PIN_LED 13           /* on port C, active low on this board */

#define GPIOC ((gpio_t *)0x40011000u)
#define IOPCEN (1u << 4)

static void cfg_pin(gpio_t *g, uint32_t pin, uint32_t cnf)
{
    volatile uint32_t *r = (pin < 8) ? &g->CRL : &g->CRH;
    uint32_t sh = (pin & 7) * 4;
    *r = (*r & ~(0xFu << sh)) | (cnf << sh);
}
static inline void gp_hi(gpio_t *g, uint32_t p) { g->BSRR = 1u << p; }
static inline void gp_lo(gpio_t *g, uint32_t p) { g->BRR  = 1u << p; }

/* ---- time ---- */
static uint32_t cyc_acc, ms_count, stk_last;

static uint32_t millis(void)
{
    uint32_t now = STK_VAL;
    cyc_acc += (stk_last - now) & 0xFFFFFFu;
    stk_last = now;
    while (cyc_acc >= (F_CPU / 1000u)) { cyc_acc -= (F_CPU / 1000u); ms_count++; }
    return ms_count;
}
static void delay_ms(uint32_t n)
{
    uint32_t t0 = millis();
    while (millis() - t0 < n) { }
}
static void delay_us(uint32_t n)          /* short waits, CE pulse width */
{
    uint32_t start = STK_VAL;
    uint32_t want = n * (F_CPU / 1000000u);
    while (((start - STK_VAL) & 0xFFFFFFu) < want) { }
}

/* Reboot into the HID bootloader.
 *
 * The bootloader decides whether to stay resident by reading backup register
 * DR4: non-zero keeps it, zero makes it jump straight to the application. So
 * the only way back without touching BOOT0 is to set that register and reset. */
static void reboot_to_bootloader(void)
{
    RCC_APB1ENR |= (1u << 28) | (1u << 27);   /* PWREN, BKPEN */
    PWR_CR      |= (1u << 8);                 /* DBP: allow backup writes */
    BKP_DR4      = 0x424C;                    /* any non-zero value will do */
    PWR_CR      &= ~(1u << 8);
    SCB_AIRCR    = 0x05FA0004u;               /* system reset request */
    for (;;) { }
}

/* ---- UART ---- */
static void uart_init(void)
{
    RCC_APB2ENR |= USART1EN;
    cfg_pin(GPIOA, 9,  CNF_AF_PP);       /* TX */
    cfg_pin(GPIOA, 10, CNF_IN_FLOAT);    /* RX */
    USART1_CR1 = 0;
    USART1_BRR = F_PCLK2 / 9600u;          /* 72 MHz / 9600 = 7500 */
    USART1_CR1 = (1u << 13) | (1u << 3) | (1u << 2);   /* UE, TE, RE */
}
static int uart_getc(void)               /* -1 when nothing is waiting */
{
    if (!(USART1_SR & (1u << 5))) return -1;   /* RXNE */
    return (int)(USART1_DR & 0xFF);
}
static int cmd_getc(void)                /* UART first, then USB */
{
    int c = uart_getc();
    return (c >= 0) ? c : usb_cdc_getc();
}
static void putc_(char c)
{
    while (!(USART1_SR & (1u << 7))) { }   /* TXE */
    USART1_DR = (uint8_t)c;
    usb_cdc_putc(c);                       /* same text out both links */
}
static void puts_(const char *s) { while (*s) putc_(*s++); }
static void puthex(uint8_t v)
{
    static const char H[] = "0123456789ABCDEF";
    putc_(H[v >> 4]); putc_(H[v & 15]);
}
static void putdec(uint32_t v)
{
    char b[12]; int i = 0;
    if (!v) { putc_('0'); return; }
    while (v) { uint32_t q = 0; while (v >= 10) { v -= 10; q++; } b[i++] = (char)('0' + v); v = q; }
    while (i--) putc_(b[i]);
}
static void putsig(int16_t v)
{
    if (v < 0) { putc_('-'); putdec((uint32_t)(-(int32_t)v)); }
    else putdec((uint32_t)v);
}

/* ---- SPI1 ---- */
static void spi_init(void)
{
    RCC_APB2ENR |= SPI1EN | AFIOEN;
    cfg_pin(GPIOA, PIN_CSN, CNF_OUT_PP);
    cfg_pin(GPIOA, 5, CNF_AF_PP);        /* SCK  */
    cfg_pin(GPIOA, 6, CNF_IN_FLOAT);     /* MISO */
    cfg_pin(GPIOA, 7, CNF_AF_PP);        /* MOSI */
    cfg_pin(GPIOB, PIN_CE, CNF_OUT_PP);
    gp_hi(GPIOA, PIN_CSN);
    gp_lo(GPIOB, PIN_CE);

    SPI1_CR1 = (1u << 2)      /* MSTR             */
             | (5u << 3)      /* BR = /64 -> 1.1 MHz at 72 MHz PCLK2 */
             | (1u << 9)      /* SSM              */
             | (1u << 8)      /* SSI              */
             | (1u << 6);     /* SPE              */
}
static uint8_t spi_xfer(uint8_t v)
{
    while (!(SPI1_SR & (1u << 1))) { }    /* TXE  */
    SPI1_DR = v;
    while (!(SPI1_SR & (1u << 0))) { }    /* RXNE */
    return (uint8_t)SPI1_DR;
}

/* ---- radio ---- */
static uint8_t rf_cmd(uint8_t cmd)
{
    gp_lo(GPIOA, PIN_CSN);
    uint8_t st = spi_xfer(cmd);
    gp_hi(GPIOA, PIN_CSN);
    return st;
}
static uint8_t rf_rd(uint8_t reg)
{
    gp_lo(GPIOA, PIN_CSN);
    spi_xfer(CMD_R_REG | reg);
    uint8_t v = spi_xfer(CMD_NOP);
    gp_hi(GPIOA, PIN_CSN);
    return v;
}
static void rf_wr(uint8_t reg, uint8_t val)
{
    gp_lo(GPIOA, PIN_CSN);
    spi_xfer(CMD_W_REG | reg);
    spi_xfer(val);
    gp_hi(GPIOA, PIN_CSN);
}
static void rf_wr_buf(uint8_t reg, const uint8_t *b, uint32_t n)
{
    gp_lo(GPIOA, PIN_CSN);
    spi_xfer(CMD_W_REG | reg);
    for (uint32_t i = 0; i < n; i++) spi_xfer(b[i]);
    gp_hi(GPIOA, PIN_CSN);
}

/* Direction test: transmitting is what draws peak current, receiving does not.
 * If the link comes alive with the roles swapped, the fault is specifically
 * this board's transmit path, which is what a sagging supply would produce.
 * Set to 0 for the normal control direction. */
#define REVERSE_TEST 0

static void rf_init_tx(void)
{
    static const uint8_t addr[RF_ADDR_LEN] = RF_ADDR;

#if REVERSE_TEST
    rf_wr(REG_CONFIG, CFG_EN_CRC | CFG_CRCO | CFG_PWR_UP | CFG_PRIM_RX);
#else
    rf_wr(REG_CONFIG, CFG_EN_CRC | CFG_CRCO | CFG_PWR_UP);   /* PRIM_RX clear */
#endif
    delay_ms(5);
    rf_wr(REG_EN_AA,      0x00);
    rf_wr(REG_EN_RXADDR,  0x01);
    rf_wr(REG_SETUP_AW,   0x03);
    rf_wr(REG_SETUP_RETR, 0x00);
    rf_wr(REG_RF_CH,      RF_CHANNEL);
    /* Minimum power, 1 Mbps. Two modules sitting a few centimetres apart at
     * full power can desensitise the receiver, which looks exactly like a dead
     * link: the transmitter reports success and the receiver hears nothing. */
    rf_wr(REG_RF_SETUP,   0x06);
    rf_wr_buf(REG_TX_ADDR,     addr, RF_ADDR_LEN);
    rf_wr_buf(REG_RX_ADDR_P0,  addr, RF_ADDR_LEN);
    rf_wr(REG_RX_PW_P0, TLM_LEN);   /* the only thing received here is telemetry */
    rf_cmd(CMD_FLUSH_RX);
    rf_cmd(CMD_FLUSH_TX);
    rf_wr(REG_STATUS, ST_RX_DR | ST_TX_DS | ST_MAX_RT);
}

/* Returns 1 if the radio reported the packet went out. */
/* Between control packets the ground unit turns around and listens for the
 * drone's telemetry burst. CE is a real pin here, so this is the documented
 * dance: CE low into Standby-I, flip PRIM_RX, CE back high. */
static void rf_listen(void)
{
    gp_lo(GPIOB, PIN_CE);
    /* This module is a clone too (it refused EN_ACK_PAY), and like the
     * drone's BK2425 it only samples PRIM_RX on the way out of power-down:
     * with a plain CONFIG flip it kept transmitting fine but never heard a
     * single telemetry burst. Bounce PWR_UP so the mode change takes. */
    rf_wr(REG_CONFIG, CFG_EN_CRC | CFG_CRCO);
    rf_wr(REG_CONFIG, CFG_EN_CRC | CFG_CRCO | CFG_PWR_UP | CFG_PRIM_RX);
    gp_hi(GPIOB, PIN_CE);
}
static void rf_hangup(void)
{
    gp_lo(GPIOB, PIN_CE);
    rf_wr(REG_CONFIG, CFG_EN_CRC | CFG_CRCO);
    rf_wr(REG_CONFIG, CFG_EN_CRC | CFG_CRCO | CFG_PWR_UP);
    delay_ms(3);        /* crystal restart before rf_send pulses CE */
}

static uint32_t rf_send(const pkt_t *p)
{
    gp_lo(GPIOA, PIN_CSN);
    spi_xfer(CMD_W_TX_PAYLOAD);
    const uint8_t *b = (const uint8_t *)p;
    for (uint32_t i = 0; i < RF_PAYLOAD; i++) spi_xfer(b[i]);
    gp_hi(GPIOA, PIN_CSN);

    gp_hi(GPIOB, PIN_CE);
    delay_us(15);                 /* well past the 10 us minimum */
    gp_lo(GPIOB, PIN_CE);

    for (uint32_t i = 0; i < 1000; i++) {
        uint8_t st = rf_rd(REG_STATUS);
        if (st & ST_TX_DS)  { rf_wr(REG_STATUS, ST_TX_DS); return 1; }
        if (st & ST_MAX_RT) { rf_wr(REG_STATUS, ST_MAX_RT); rf_cmd(CMD_FLUSH_TX); return 0; }
        delay_us(10);
    }
    rf_cmd(CMD_FLUSH_TX);
    return 0;
}

/* Clock is set here rather than inherited. The HID bootloader leaves the core at
 * 72 MHz and hands over without restoring it, so an app that assumed the 8 MHz
 * HSI had every timing constant wrong by 9x — UART output came out near 86400
 * baud instead of 9600.
 *
 * 72 MHz is also what USB needs: the peripheral wants exactly 48 MHz, which the
 * internal HSI cannot produce accurately enough, so this runs off the board's
 * 8 MHz crystal through PLL x9 with the USB prescaler dividing by 1.5. */
static void clock_init_hse72(void)
{
    /* The bootloader hands over with the PLL already running AND selected as
     * the system clock. Disabling the PLL first would pull the clock out from
     * under the core mid-instruction, so park on the HSI before touching it. */
    RCC_CR |= (1u << 0);                        /* HSION */
    while (!(RCC_CR & (1u << 1))) { }           /* HSIRDY */
    RCC_CFGR &= ~3u;                            /* SW = HSI */
    while ((RCC_CFGR & (3u << 2)) != 0) { }     /* SWS confirms HSI */

    RCC_CR |= (1u << 16);                       /* HSEON */
    while (!(RCC_CR & (1u << 17))) { }          /* HSERDY */

    FLASH_ACR = (FLASH_ACR & ~7u) | 2u | (1u << 4);   /* 2 wait states + prefetch */

    RCC_CFGR &= ~(0xFu << 4);                   /* AHB  /1  -> 72 MHz */
    RCC_CFGR = (RCC_CFGR & ~(7u << 8))  | (4u << 8);  /* APB1 /2 -> 36 MHz, its max */
    RCC_CFGR &= ~(7u << 11);                    /* APB2 /1  -> 72 MHz */

    RCC_CR &= ~(1u << 24);                      /* PLL off while reconfiguring */
    while (RCC_CR & (1u << 25)) { }
    RCC_CFGR &= ~((1u << 22) | (0xFu << 18) | (1u << 17));
    RCC_CFGR |= (1u << 16)                      /* PLL source = HSE */
             |  (7u << 18);                     /* PLL x9: 8 -> 72 MHz */
    RCC_CFGR &= ~(1u << 22);                    /* USB prescaler /1.5 -> 48 MHz */
    RCC_CR |= (1u << 24);                       /* PLL on */
    while (!(RCC_CR & (1u << 25))) { }

    RCC_CFGR = (RCC_CFGR & ~3u) | 2u;           /* SW = PLL */
    while ((RCC_CFGR & (3u << 2)) != (2u << 2)) { }
}

#define SCB_VTOR (*(volatile uint32_t *)0xE000ED08u)
#define APP_BASE 0x08000800u   /* HID bootloader reserves the first 2 KB */

/* Twenty seconds at 8 MHz before the normal 72 MHz startup.
 *
 * The link was seen working from this board when it ran at 8 MHz with no USB.
 * Raising the clock and adding USB is the only change on the transmit path
 * since. This reproduces the old conditions briefly and then restores the
 * normal ones, so the board is never stranded without its USB console. The
 * result is read on the drone's UART, not here. */
static void hsi8_burst(void)
{
    static const uint8_t addr[RF_ADDR_LEN] = RF_ADDR;

    RCC_CR |= (1u << 0);
    while (!(RCC_CR & (1u << 1))) { }
    RCC_CFGR &= ~3u;                       /* SW = HSI, 8 MHz */
    while ((RCC_CFGR & (3u << 2)) != 0) { }
    RCC_CR &= ~(1u << 24);                 /* PLL off, safe now that HSI drives the core */

    RCC_APB2ENR |= IOPAEN | IOPBEN | AFIOEN | SPI1EN;
    cfg_pin(GPIOA, PIN_CSN, CNF_OUT_PP);
    cfg_pin(GPIOA, 5, CNF_AF_PP);
    cfg_pin(GPIOA, 6, CNF_IN_FLOAT);
    cfg_pin(GPIOA, 7, CNF_AF_PP);
    cfg_pin(GPIOB, PIN_CE, CNF_OUT_PP);
    gp_hi(GPIOA, PIN_CSN);
    gp_lo(GPIOB, PIN_CE);
    SPI1_CR1 = (1u << 2) | (2u << 3) | (1u << 9) | (1u << 8) | (1u << 6);  /* /8 -> 1 MHz */

    rf_wr(REG_CONFIG, CFG_EN_CRC | CFG_CRCO | CFG_PWR_UP);
    for (volatile uint32_t i = 0; i < 40000u; i++) { }
    rf_wr(REG_EN_AA,      0x00);
    rf_wr(REG_EN_RXADDR,  0x01);
    rf_wr(REG_SETUP_AW,   0x03);
    rf_wr(REG_SETUP_RETR, 0x00);
    rf_wr(REG_RF_CH,      RF_CHANNEL);
    rf_wr(REG_RF_SETUP,   0x06);
    rf_wr_buf(REG_TX_ADDR,    addr, RF_ADDR_LEN);
    rf_wr_buf(REG_RX_ADDR_P0, addr, RF_ADDR_LEN);
    rf_cmd(CMD_FLUSH_TX);
    rf_wr(REG_STATUS, ST_RX_DR | ST_TX_DS | ST_MAX_RT);

    pkt_t p = { PKT_MAGIC, 0, 0, 128, 128, 128, 0, 0 };
    for (uint32_t n = 0; n < 1000u; n++) {          /* roughly 20 s */
        p.seq++;
        p.sum = pkt_sum(&p);
        gp_lo(GPIOA, PIN_CSN);
        spi_xfer(CMD_W_TX_PAYLOAD);
        for (uint32_t i = 0; i < RF_PAYLOAD; i++) spi_xfer(((const uint8_t *)&p)[i]);
        gp_hi(GPIOA, PIN_CSN);

        gp_hi(GPIOB, PIN_CE);
        for (volatile uint32_t i = 0; i < 40u; i++) { }
        gp_lo(GPIOB, PIN_CE);

        for (volatile uint32_t i = 0; i < 12000u; i++) { }
        rf_wr(REG_STATUS, ST_TX_DS | ST_MAX_RT);
    }
}

int main(void)
{
    hsi8_burst();

    /* The bootloader left its own vector table active. Point the core at ours;
     * this build polls rather than using interrupts, but leaving VTOR stale is
     * a trap for the first interrupt anyone adds later. */
    SCB_VTOR = APP_BASE;
    clock_init_hse72();

    RCC_APB2ENR |= IOPAEN | IOPBEN | AFIOEN;
    STK_LOAD = 0x00FFFFFFu; STK_VAL = 0; STK_CTRL = 5;
    stk_last = STK_VAL;

    uart_init();
    usb_cdc_init();
    spi_init();

    puts_("\r\nsima8 ground tx\r\n");
    /* A radio that is absent or miswired reads back 0x00 or 0xFF here */
    puts_("RX_ADDR_P0 byte0=0x"); puthex(rf_rd(REG_RX_ADDR_P0));
    rf_init_tx();
    puts_(" CONFIG=0x"); puthex(rf_rd(REG_CONFIG));
    puts_(" RF_CH=");    putdec(rf_rd(REG_RF_CH));
    puts_("\r\n");

    /* PC13 heartbeat, so the board reports liveness without a cable. The LED is
     * active low here: driving the pin low lights it. */
    RCC_APB2ENR |= IOPCEN;
    cfg_pin(GPIOC, PIN_LED, CNF_OUT_PP);
    uint32_t led_on = 0, last_led = millis();

    puts_("lenh: t<n> ga, r/p/y<n> truc, a arm, d disarm, s status, b bootloader\r\n");

    char line[24];
    uint32_t li = 0;

    pkt_t pkt = { PKT_MAGIC, 0, 0, 128, 128, 128, 0, 0 };
    uint32_t sent = 0, failed = 0, rpd_hits = 0;
    tlm_t tlm = {0,0,0,0,0,0,0,0,0,0};
    uint32_t tlm_good = 0, tlm_bad = 0, last_tlm_ms = 0;
    uint32_t last_tx = millis(), last_report = last_tx;

    for (;;) {
        uint32_t now = millis();

        usb_cdc_poll();

        /* A 1200-baud open is how hosts ask for the bootloader; honour it the
         * same way the 'b' command does. */
        if (usb_cdc_reboot_request) {
            usb_cdc_reboot_request = 0;
            for (volatile int k = 0; k < 400000; k++) { usb_cdc_poll(); }
            reboot_to_bootloader();
        }

        /* command input, one line at a time */
        int ch = cmd_getc();
        if (ch >= 0) {
            if (ch == '\r' || ch == '\n') {
                line[li] = 0;
                if (li) {
                    uint32_t v = 0;
                    for (uint32_t i = 1; i < li; i++)
                        if (line[i] >= '0' && line[i] <= '9') v = v * 10 + (uint32_t)(line[i] - '0');
                    if (v > 255) v = 255;
                    switch (line[0]) {
                    case 't': pkt.throttle = (uint8_t)v; puts_("ok throttle\r\n"); break;
                    case 'r': pkt.roll     = (uint8_t)v; puts_("ok roll\r\n");     break;
                    case 'p': pkt.pitch    = (uint8_t)v; puts_("ok pitch\r\n");    break;
                    case 'y': pkt.yaw      = (uint8_t)v; puts_("ok yaw\r\n");      break;
                    /* arming is refused unless the throttle is already at zero,
                     * so a stale stick value cannot become thrust */
                    case 'a':
                        if (pkt.throttle == 0) { pkt.flags |= PKT_ARM; puts_("ARMED\r\n"); }
                        else puts_("tu choi: ha throttle ve 0 truoc\r\n");
                        break;
                    case 'd': pkt.flags &= (uint8_t)~PKT_ARM; pkt.throttle = 0; puts_("DISARMED\r\n"); break;
                    case 's': puts_("status theo dong bao cao moi giay\r\n"); break;
                    /* Continuous carrier. Decides whether this radio puts any
                     * energy on air at all, independent of address, CRC or
                     * payload — things TX_DS cannot tell us. */
                    case 'w':
                        rf_wr(REG_RF_SETUP, 0x96);   /* CONT_WAVE | PLL_LOCK | 1 Mbps */
                        rf_wr(REG_CONFIG, CFG_EN_CRC | CFG_CRCO | CFG_PWR_UP);
                        gp_hi(GPIOB, PIN_CE);
                        puts_("song mang LIEN TUC bat\r\n");
                        break;
                    case 'n':
                        gp_lo(GPIOB, PIN_CE);
                        rf_wr(REG_RF_SETUP, 0x06);
                        puts_("song mang tat\r\n");
                        break;
                    case 'b': puts_("vao bootloader...\r\n"); for (volatile int k = 0; k < 200000; k++) { } reboot_to_bootloader(); break;
                    default:  puts_("lenh la\r\n"); break;
                    }
                }
                li = 0;
            } else if (li < sizeof(line) - 1) {
                line[li++] = (char)ch;
            }
        }

        if (now - last_led >= 250) {        /* 2 Hz: alive and in the main loop */
            last_led = now;
            led_on ^= 1;
            if (led_on) gp_lo(GPIOC, PIN_LED); else gp_hi(GPIOC, PIN_LED);
        }

#if REVERSE_TEST
        /* CE is held high for RX and the drone does the transmitting */
        gp_hi(GPIOB, PIN_CE);
        if (rf_rd(0x09) & 1u) rpd_hits++;
        if (rf_rd(REG_STATUS) & ST_RX_DR) {
            pkt_t in;
            gp_lo(GPIOA, PIN_CSN);
            spi_xfer(CMD_R_RX_PAYLOAD);
            for (uint32_t i = 0; i < RF_PAYLOAD; i++) ((uint8_t *)&in)[i] = spi_xfer(CMD_NOP);
            gp_hi(GPIOA, PIN_CSN);
            rf_wr(REG_STATUS, ST_RX_DR);
            if (pkt_valid(&in)) sent++; else failed++;
        }
        (void)last_tx;
#else
        if (now - last_tx >= TX_PERIOD_MS) {
            last_tx = now;
            rf_hangup();
            pkt.seq++;
            pkt.sum = pkt_sum(&pkt);
            if (rf_send(&pkt)) sent++; else failed++;
            rf_listen();
        }

        /* Raw carrier sniff while listening: RPD (reg 0x09 bit 0) sets on any
         * >-64 dBm energy on the channel, address and CRC be damned. Separates
         * "the burst never arrives" from "it arrives and is rejected". */
        if (rf_rd(0x09) & 1u) rpd_hits++;

        /* Telemetry arriving in the listening window between control packets */
        if (rf_rd(REG_STATUS) & ST_RX_DR) {
            tlm_t in;
            gp_lo(GPIOA, PIN_CSN);
            spi_xfer(CMD_R_RX_PAYLOAD);
            for (uint32_t i = 0; i < TLM_LEN; i++) ((uint8_t *)&in)[i] = spi_xfer(CMD_NOP);
            gp_hi(GPIOA, PIN_CSN);
            rf_wr(REG_STATUS, ST_RX_DR);
            if (tlm_valid(&in)) {
                tlm = in;
                tlm_good++;
                last_tlm_ms = now;
                puts_("TLM ");
                puts_((tlm.flags & TLM_MPU_OK) ? "ok " : "mat ");
                putsig(tlm.ax); putc_(' ');
                putsig(tlm.ay); putc_(' ');
                putsig(tlm.az); putc_(' ');
                putsig(tlm.gx); putc_(' ');
                putsig(tlm.gy); putc_(' ');
                putsig(tlm.gz);
                puts_("\r\n");
            } else {
                tlm_bad++;
            }
        }
#endif

        if (now - last_report >= 1000) {
            last_report = now;
            /* Register readbacks separate "no radio on the bus" from "radio
             * present but the transmit is failing": a missing or unpowered
             * module reads 0x00 or 0xFF everywhere, while a live one echoes
             * the address and channel that were written into it. */
            uint8_t a0 = rf_rd(REG_RX_ADDR_P0);
            uint8_t cf = rf_rd(REG_CONFIG);
            uint8_t st = rf_rd(REG_STATUS);
            uint8_t fs = rf_rd(REG_FIFO_STATUS);
            puts_("rpd=");     putdec(rpd_hits); rpd_hits = 0;
            puts_(" ch=");     putdec(rf_rd(REG_RF_CH));
            puts_(" a0=0x");   puthex(rf_rd(REG_RX_ADDR_P0));
            puts_(" tx0=0x");  puthex(rf_rd(REG_TX_ADDR));
            puts_(" aw=0x");   puthex(rf_rd(REG_SETUP_AW));
            puts_(" rf=0x");   puthex(rf_rd(REG_RF_SETUP));
            puts_(" retr=0x"); puthex(rf_rd(REG_SETUP_RETR));
            puts_(" aa=0x");   puthex(rf_rd(REG_EN_AA));
            puts_("\r\n");
            puts_("rx good="); putdec(sent);
            puts_(" failed=");  putdec(failed);
            puts_(" ADDR0=0x"); puthex(a0);
            puts_(" CONFIG=0x");puthex(cf);
            puts_(" STATUS=0x");puthex(st);
            puts_(" FIFO=0x");  puthex(fs);
            puts_(" tlm=");     putdec(tlm_good);
            puts_("/");         putdec(tlm_bad);
            puts_(now - last_tlm_ms < 1000 ? " rf-tlm=UP" : " rf-tlm=DOWN");
            puts_(a0 == 0x00 || a0 == 0xFF ? "  <- khong thay radio\r\n" : "\r\n");
        }
    }
}
