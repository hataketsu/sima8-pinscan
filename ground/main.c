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
    while (cyc_acc >= 8000u) { cyc_acc -= 8000u; ms_count++; }
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
    uint32_t want = n * 8u;
    while (((start - STK_VAL) & 0xFFFFFFu) < want) { }
}

/* ---- UART ---- */
static void uart_init(void)
{
    RCC_APB2ENR |= USART1EN;
    cfg_pin(GPIOA, 9, CNF_AF_PP);
    USART1_CR1 = 0;
    USART1_BRR = 833;                      /* 8 MHz / 9600 */
    USART1_CR1 = (1u << 13) | (1u << 3);   /* UE, TE */
}
static void putc_(char c)
{
    while (!(USART1_SR & (1u << 7))) { }   /* TXE */
    USART1_DR = (uint8_t)c;
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
             | (2u << 3)      /* BR = /8 -> 1 MHz */
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

static void rf_init_tx(void)
{
    static const uint8_t addr[RF_ADDR_LEN] = RF_ADDR;

    rf_wr(REG_CONFIG, CFG_EN_CRC | CFG_CRCO | CFG_PWR_UP);   /* PRIM_RX clear */
    delay_ms(5);
    rf_wr(REG_EN_AA,      0x00);
    rf_wr(REG_EN_RXADDR,  0x01);
    rf_wr(REG_SETUP_AW,   0x03);
    rf_wr(REG_SETUP_RETR, 0x00);
    rf_wr(REG_RF_CH,      RF_CHANNEL);
    rf_wr(REG_RF_SETUP,   0x06);
    rf_wr_buf(REG_TX_ADDR,     addr, RF_ADDR_LEN);
    rf_wr_buf(REG_RX_ADDR_P0,  addr, RF_ADDR_LEN);
    rf_cmd(CMD_FLUSH_TX);
    rf_wr(REG_STATUS, ST_RX_DR | ST_TX_DS | ST_MAX_RT);
}

/* Returns 1 if the radio reported the packet went out. */
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

int main(void)
{
    RCC_APB2ENR |= IOPAEN | IOPBEN | AFIOEN;
    STK_LOAD = 0x00FFFFFFu; STK_VAL = 0; STK_CTRL = 5;
    stk_last = STK_VAL;

    uart_init();
    spi_init();

    puts_("\r\nsima8 ground tx\r\n");
    /* A radio that is absent or miswired reads back 0x00 or 0xFF here */
    puts_("RX_ADDR_P0 byte0=0x"); puthex(rf_rd(REG_RX_ADDR_P0));
    rf_init_tx();
    puts_(" CONFIG=0x"); puthex(rf_rd(REG_CONFIG));
    puts_(" RF_CH=");    putdec(rf_rd(REG_RF_CH));
    puts_("\r\n");

    pkt_t pkt = { PKT_MAGIC, 0, 0, 128, 128, 128, 0, 0 };
    uint32_t sent = 0, failed = 0;
    uint32_t last_tx = millis(), last_report = last_tx;

    for (;;) {
        uint32_t now = millis();

        if (now - last_tx >= TX_PERIOD_MS) {
            last_tx = now;
            pkt.seq++;
            pkt.sum = pkt_sum(&pkt);
            if (rf_send(&pkt)) sent++; else failed++;
        }

        if (now - last_report >= 1000) {
            last_report = now;
            puts_("tx sent="); putdec(sent);
            puts_(" failed="); putdec(failed);
            puts_("\r\n");
        }
    }
}
