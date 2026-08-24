/* Sima X8 flight board — receiver end of the link.
 *
 * STM32F031K6 at the 8 MHz HSI. Pin map came from the scanner in this repo:
 *   PA4 CSN   PA5 SCK   PA6 MISO   PA7 MOSI    BK2425 on hardware SPI1
 *   PA9 USART1_TX at 9600                      telemetry
 *   PA2 PA3 PA8 PA11                           motor gates, ACTIVE LOW
 *   PA1                                        LED
 *
 * The radio's CE is strapped high on this board, so it is always enabled and
 * RX/TX is selected purely through the PRIM_RX bit. There is no IRQ line
 * either, so STATUS is polled.
 *
 * Motors are forced off before anything else runs. The gates are active low and
 * float at reset, so leaving them alone is not the same as leaving them off.
 */
#include <stdint.h>
#include "../common/protocol.h"

#define RCC_AHBENR   (*(volatile uint32_t *)0x40021014u)
#define RCC_APB2ENR  (*(volatile uint32_t *)0x40021018u)
#define IOPAEN       (1u << 17)
#define IOPBEN       (1u << 18)
#define SPI1EN       (1u << 12)
#define USART1EN     (1u << 14)

#define STK_CTRL  (*(volatile uint32_t *)0xE000E010u)
#define STK_LOAD  (*(volatile uint32_t *)0xE000E014u)
#define STK_VAL   (*(volatile uint32_t *)0xE000E018u)

#define SPI1_CR1  (*(volatile uint32_t *)0x40013000u)
#define SPI1_CR2  (*(volatile uint32_t *)0x40013004u)
#define SPI1_SR   (*(volatile uint32_t *)0x40013008u)
#define SPI1_DR8  (*(volatile uint8_t  *)0x4001300Cu)

#define RCC_APB1ENR (*(volatile uint32_t *)0x4002101Cu)
#define TIM1EN      (1u << 11)   /* APB2 */
#define TIM2EN      (1u << 0)    /* APB1 */

typedef struct {
    volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER,
                      CNT, PSC, ARR, RCR, CCR1, CCR2, CCR3, CCR4, BDTR;
} tim_t;
#define TIM1 ((tim_t *)0x40012C00u)
#define TIM2 ((tim_t *)0x40000000u)

#define USART1_CR1 (*(volatile uint32_t *)0x40013800u)
#define USART1_BRR (*(volatile uint32_t *)0x4001380Cu)
#define USART1_ISR (*(volatile uint32_t *)0x4001381Cu)
#define USART1_TDR (*(volatile uint32_t *)0x40013828u)

typedef struct {
    volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2], BRR;
} gpio_t;
#define GPIOA ((gpio_t *)0x48000000u)

#define PIN_CSN   4
#define PIN_LED   1
static const uint8_t MOTOR[4] = { 2, 3, 8, 11 };   /* M1 M2 M3 M4 */

/* Non-zero only while the pins belong to the timers. */
static uint32_t motors_live;

/* ---- pin helpers, port A only ---- */
static void pa_out(uint32_t p)
{
    GPIOA->OTYPER &= ~(1u << p);
    GPIOA->MODER   = (GPIOA->MODER & ~(3u << (p * 2))) | (1u << (p * 2));
}
static void pa_af(uint32_t p, uint32_t af)
{
    GPIOA->AFR[p >> 3] = (GPIOA->AFR[p >> 3] & ~(0xFu << ((p & 7) * 4)))
                       | (af << ((p & 7) * 4));
    GPIOA->MODER = (GPIOA->MODER & ~(3u << (p * 2))) | (2u << (p * 2));
}
static inline void pa_hi(uint32_t p) { GPIOA->BSRR = 1u << p; }
static inline void pa_lo(uint32_t p) { GPIOA->BRR  = 1u << p; }

/* Gates are active low: high is off. Called before anything else. */
static void motors_off(void)
{
    RCC_AHBENR |= IOPAEN;
    for (uint32_t i = 0; i < 4; i++) { pa_hi(MOTOR[i]); pa_out(MOTOR[i]); }
    motors_live = 0;
}

/* ---- motor PWM ----
 *
 * M1 PA2 = TIM2_CH3   M2 PA3 = TIM2_CH4   M3 PA8 = TIM1_CH1   M4 PA11 = TIM1_CH4
 *
 * The gates are active low, so the channels run PWM mode 1 with CCxP set, which
 * inverts the output: CCR = 0 leaves the pin high and the motor off, CCR = ARR
 * holds it low for full drive. Getting this backwards would spin every motor at
 * full throttle the moment the timer starts.
 *
 * The pins are only handed to the timer while armed. Disarming, a failsafe, or
 * a reset takes them back as plain GPIO driven high, so every path out of
 * flight ends with the gates off regardless of what the timer registers hold.
 */
#define PWM_ARR 499u             /* 8 MHz / 500 = 16 kHz, above audible whine */

static void pwm_init(void)
{
    RCC_APB2ENR |= TIM1EN;
    RCC_APB1ENR |= TIM2EN;

    TIM2->PSC = 0; TIM2->ARR = PWM_ARR;
    TIM2->CCR3 = 0; TIM2->CCR4 = 0;
    TIM2->CCMR2 = (6u << 4) | (1u << 3)        /* CH3: PWM mode 1, preload */
                | (6u << 12) | (1u << 11);     /* CH4: PWM mode 1, preload */
    TIM2->CCER  = (1u << 8) | (1u << 9)        /* CH3 enable, active low */
                | (1u << 12) | (1u << 13);     /* CH4 enable, active low */
    TIM2->CR1  |= (1u << 7) | (1u << 0);       /* ARPE, CEN */
    TIM2->EGR   = 1;                           /* load the preloads */

    TIM1->PSC = 0; TIM1->ARR = PWM_ARR;
    TIM1->CCR1 = 0; TIM1->CCR4 = 0;
    TIM1->CCMR1 = (6u << 4) | (1u << 3);       /* CH1: PWM mode 1, preload */
    TIM1->CCMR2 = (6u << 12) | (1u << 11);     /* CH4: PWM mode 1, preload */
    TIM1->CCER  = (1u << 0) | (1u << 1)        /* CH1 enable, active low */
                | (1u << 12) | (1u << 13);     /* CH4 enable, active low */
    TIM1->BDTR |= (1u << 15);                  /* MOE: advanced timer outputs */
    TIM1->CR1  |= (1u << 7) | (1u << 0);
    TIM1->EGR   = 1;
}

/* Hand the four pins to the timers. Only ever called with a live link. */
static void motors_engage(void)
{
    TIM2->CCR3 = 0; TIM2->CCR4 = 0;
    TIM1->CCR1 = 0; TIM1->CCR4 = 0;
    pa_af(2, 2); pa_af(3, 2); pa_af(8, 2); pa_af(11, 2);
    motors_live = 1;
}

static void motors_set(uint8_t throttle)
{
    /* 0..255 -> 0..PWM_ARR without a divide, which Cortex-M0 lacks */
    uint32_t duty = ((uint32_t)throttle * (PWM_ARR + 1)) >> 8;
    TIM2->CCR3 = duty; TIM2->CCR4 = duty;
    TIM1->CCR1 = duty; TIM1->CCR4 = duty;
}

/* ---- time ---- */
static uint32_t cyc_acc, ms_count, stk_last;

/* No divide instruction on Cortex-M0 and no libgcc in this build, so the
 * millisecond count is carried by subtraction instead of division. */
static uint32_t millis(void)
{
    uint32_t now = STK_VAL;
    cyc_acc += (stk_last - now) & 0xFFFFFFu;
    stk_last = now;
    while (cyc_acc >= 8000u) { cyc_acc -= 8000u; ms_count++; }   /* 8 MHz core */
    return ms_count;
}

static void delay_ms(uint32_t n)
{
    uint32_t t0 = millis();
    while (millis() - t0 < n) { }
}

/* ---- UART ---- */
static void uart_init(void)
{
    RCC_APB2ENR |= USART1EN;
    pa_af(9, 1);
    USART1_CR1 = 0;
    USART1_BRR = 833;                   /* 8 MHz / 9600 */
    USART1_CR1 = (1u << 3) | (1u << 0); /* TE, UE */
}
/* Transmit through a ring buffer so printing never stalls the main loop:
 * a blocked putc_ used to hold the loop for the length of a whole line,
 * overflowing the radio's 3-deep RX FIFO and silently dropping packets.
 * uart_pump() moves one queued byte to the shifter whenever TXE is set;
 * putc_ only spins if the queue itself is full, which a 512-byte queue
 * makes rare (the longest burst of lines is ~200 bytes). */
#define TXQ_SIZE 512u
static uint8_t txq[TXQ_SIZE];
static volatile uint32_t txq_head, txq_tail;

static void uart_pump(void)
{
    if (txq_head != txq_tail && (USART1_ISR & (1u << 7))) {
        USART1_TDR = txq[txq_tail % TXQ_SIZE];
        txq_tail++;
    }
}
static void putc_(char c)
{
    while (txq_head - txq_tail >= TXQ_SIZE) uart_pump();
    txq[txq_head % TXQ_SIZE] = (uint8_t)c;
    txq_head++;
}
static void puts_(const char *s) { while (*s) putc_(*s++); }
static void puthex(uint8_t v)
{
    static const char H[] = "0123456789ABCDEF";
    putc_(H[v >> 4]); putc_(H[v & 15]);
}
static void putsig(int16_t v);
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
    RCC_APB2ENR |= SPI1EN;
    pa_hi(PIN_CSN); pa_out(PIN_CSN);
    pa_af(5, 0); pa_af(6, 0); pa_af(7, 0);     /* SCK MISO MOSI are AF0 here */

    SPI1_CR1 = 0;
    /* 8-bit frames, RX threshold at 8 bits or RXNE never fires for bytes */
    SPI1_CR2 = (7u << 8) | (1u << 12);
    SPI1_CR1 = (1u << 2)      /* MSTR                      */
             | (2u << 3)      /* BR = /8 -> 1 MHz          */
             | (1u << 9)      /* SSM                       */
             | (1u << 8)      /* SSI                       */
             | (1u << 6);     /* SPE                       */
}

static uint8_t spi_xfer(uint8_t v)
{
    while (!(SPI1_SR & (1u << 1))) { }      /* TXE  */
    SPI1_DR8 = v;
    while (!(SPI1_SR & (1u << 0))) { }      /* RXNE */
    return SPI1_DR8;
}

/* ---- MPU6050 on PB6/PB7, bit-banged ----
 * I2C1 sits on exactly these pins, but bit-banging keeps this independent of
 * the alternate-function mapping, which is the one part of the pin map that was
 * read off a datasheet rather than measured. */
#define GPIOB ((gpio_t *)0x48000400u)
#define SCL 6
#define SDA 7
#define MPU_ADDR 0xD0
#define I2C_HALF 40

static void pb_in(uint32_t p)  { GPIOB->MODER &= ~(3u << (p * 2));
                                 GPIOB->PUPDR = (GPIOB->PUPDR & ~(3u << (p * 2))) | (1u << (p * 2)); }
static void pb_low(uint32_t p) { GPIOB->BRR = 1u << p;
                                 GPIOB->OTYPER &= ~(1u << p);
                                 GPIOB->MODER = (GPIOB->MODER & ~(3u << (p * 2))) | (1u << (p * 2)); }
static uint32_t pb_rd(uint32_t p) { return (GPIOB->IDR >> p) & 1u; }
static void ib_dly(volatile uint32_t n) { while (n--) __asm__ volatile("nop"); }

static void i2c_start(void)
{
    pb_in(SDA); pb_in(SCL); ib_dly(I2C_HALF);
    pb_low(SDA); ib_dly(I2C_HALF);
    pb_low(SCL); ib_dly(I2C_HALF);
}
static void i2c_stop(void)
{
    pb_low(SDA); ib_dly(I2C_HALF);
    pb_in(SCL);  ib_dly(I2C_HALF);
    pb_in(SDA);  ib_dly(I2C_HALF);
}
static uint32_t i2c_wr(uint8_t b)
{
    for (int i = 7; i >= 0; i--) {
        if ((b >> i) & 1) pb_in(SDA); else pb_low(SDA);
        ib_dly(I2C_HALF);
        pb_in(SCL); ib_dly(I2C_HALF);
        pb_low(SCL); ib_dly(I2C_HALF);
    }
    pb_in(SDA); ib_dly(I2C_HALF);
    pb_in(SCL);  ib_dly(I2C_HALF);
    uint32_t ack = !pb_rd(SDA);
    pb_low(SCL); ib_dly(I2C_HALF);
    return ack;
}
static uint8_t i2c_rd(uint32_t ack)
{
    uint8_t v = 0;
    pb_in(SDA);
    for (int i = 7; i >= 0; i--) {
        ib_dly(I2C_HALF);
        pb_in(SCL); ib_dly(I2C_HALF);
        v |= (uint8_t)(pb_rd(SDA) << i);
        pb_low(SCL); ib_dly(I2C_HALF);
    }
    if (ack) pb_low(SDA); else pb_in(SDA);
    ib_dly(I2C_HALF);
    pb_in(SCL); ib_dly(I2C_HALF);
    pb_low(SCL); ib_dly(I2C_HALF);
    pb_in(SDA);
    return v;
}
static uint32_t mpu_wr(uint8_t reg, uint8_t val)
{
    i2c_start();
    if (!i2c_wr(MPU_ADDR) || !i2c_wr(reg) || !i2c_wr(val)) { i2c_stop(); return 0; }
    i2c_stop();
    return 1;
}
static uint32_t mpu_burst(uint8_t reg, uint8_t *b, uint32_t n)
{
    i2c_start();
    if (!i2c_wr(MPU_ADDR) || !i2c_wr(reg)) { i2c_stop(); return 0; }
    i2c_start();
    if (!i2c_wr(MPU_ADDR | 1)) { i2c_stop(); return 0; }
    for (uint32_t i = 0; i < n; i++) b[i] = i2c_rd(i + 1 < n);
    i2c_stop();
    return 1;
}
static uint32_t mpu_begin(void)
{
    RCC_AHBENR |= IOPBEN;
    pb_in(SCL); pb_in(SDA);
    delay_ms(2);
    if (!mpu_wr(0x6B, 0x01)) return 0;   /* leave sleep, clock off the X gyro */
    delay_ms(20);
    mpu_wr(0x1B, 0x00);                  /* gyro  +-250 dps */
    mpu_wr(0x1C, 0x00);                  /* accel +-2 g     */
    mpu_wr(0x19, 0x07);                  /* sample rate divider */
    return 1;
}

/* ---- radio ---- */
static uint8_t rf_cmd(uint8_t cmd)
{
    pa_lo(PIN_CSN);
    uint8_t st = spi_xfer(cmd);
    pa_hi(PIN_CSN);
    return st;
}
static uint8_t rf_rd(uint8_t reg)
{
    pa_lo(PIN_CSN);
    spi_xfer(CMD_R_REG | reg);
    uint8_t v = spi_xfer(CMD_NOP);
    pa_hi(PIN_CSN);
    return v;
}
static void rf_wr(uint8_t reg, uint8_t val)
{
    pa_lo(PIN_CSN);
    spi_xfer(CMD_W_REG | reg);
    spi_xfer(val);
    pa_hi(PIN_CSN);
}
static void rf_wr_buf(uint8_t reg, const uint8_t *b, uint32_t n)
{
    pa_lo(PIN_CSN);
    spi_xfer(CMD_W_REG | reg);
    for (uint32_t i = 0; i < n; i++) spi_xfer(b[i]);
    pa_hi(PIN_CSN);
}
static void rf_rx_payload(uint8_t *b, uint32_t n)
{
    pa_lo(PIN_CSN);
    spi_xfer(CMD_R_RX_PAYLOAD);
    for (uint32_t i = 0; i < n; i++) b[i] = spi_xfer(CMD_NOP);
    pa_hi(PIN_CSN);
}

/* Matches REVERSE_TEST on the ground unit: this end transmits instead. */
#define REVERSE_TEST 0

static void rf_init_rx(void)
{
    static const uint8_t addr[RF_ADDR_LEN] = RF_ADDR;

#if REVERSE_TEST
    rf_wr(REG_CONFIG, CFG_EN_CRC | CFG_CRCO | CFG_PWR_UP);
#else
    rf_wr(REG_CONFIG, CFG_EN_CRC | CFG_CRCO | CFG_PWR_UP | CFG_PRIM_RX);
#endif
    delay_ms(5);                        /* power-up settling */
    rf_wr(REG_EN_AA,      0x00);        /* no auto-ack, matching the ground unit */
    rf_wr(REG_EN_RXADDR,  0x01);        /* pipe 0 only */
    rf_wr(REG_SETUP_AW,   0x03);        /* 5-byte addresses */
    rf_wr(REG_SETUP_RETR, 0x00);
    rf_wr(REG_RF_CH,      RF_CHANNEL);
    rf_wr(REG_RF_SETUP,   0x06);        /* 1 Mbps, 0 dBm */
    rf_wr_buf(REG_RX_ADDR_P0, addr, RF_ADDR_LEN);
    rf_wr_buf(REG_TX_ADDR,    addr, RF_ADDR_LEN);
    rf_wr(REG_RX_PW_P0,   RF_PAYLOAD);

    rf_cmd(CMD_FLUSH_RX);
    rf_wr(REG_STATUS, ST_RX_DR | ST_TX_DS | ST_MAX_RT);
}



int main(void)
{
    motors_off();

    RCC_AHBENR |= IOPAEN | IOPBEN;
    STK_LOAD = 0x00FFFFFFu; STK_VAL = 0; STK_CTRL = 5;
    stk_last = STK_VAL;

    pa_lo(PIN_LED); pa_out(PIN_LED);

    uart_init();
    spi_init();
    pwm_init();
    motors_off();          /* pwm_init only prepared the timers; keep the pins */

    puts_("\r\nsima8 drone rx\r\n");
    uint8_t cfg_check = rf_rd(REG_CONFIG);
    rf_init_rx();
    rf_init_rx();
    puts_("CONFIG before=0x"); puthex(cfg_check);
    puts_(" after=0x"); puthex(rf_rd(REG_CONFIG));
    puts_(" RF_CH="); putdec(rf_rd(REG_RF_CH));
    puts_("\r\n");

    pkt_t pkt;
    uint32_t good = 0, bad = 0;
    tlm_t tlm = { TLM_MAGIC, 0, 0,0,0, 0,0,0, 0, 0 };
    uint32_t last_tlm = 0, last_imu = 0;
    uint32_t mpu_ok = mpu_begin();
    /* Received Power Detector: set while incoming power is above about
     * -64 dBm. Sampling it separates "no RF is arriving at all" from "RF
     * arrives but the packets are being rejected". */
    uint32_t last_pkt = millis(), last_report = last_pkt, last_led = last_pkt;
    uint32_t led = 0;

    for (;;) {
        uint32_t now = millis();
        uart_pump();

#if REVERSE_TEST
        /* CE is strapped high here, so a payload write transmits immediately */
        if (now - last_report_tx >= 20u) {
            last_report_tx = now;
            pkt_t out = { PKT_MAGIC, (uint8_t)good, 0, 128, 128, 128, 0, 0 };
            out.sum = pkt_sum(&out);
            pa_lo(PIN_CSN);
            spi_xfer(CMD_W_TX_PAYLOAD);
            for (uint32_t i = 0; i < RF_PAYLOAD; i++) spi_xfer(((uint8_t *)&out)[i]);
            pa_hi(PIN_CSN);
            good++;
            rf_wr(REG_STATUS, ST_TX_DS | ST_MAX_RT);
        }
#endif
        /* Drain the whole RX FIFO, not just one payload: the UART prints
         * below block for tens of milliseconds, and packets arriving in the
         * meantime stack up in the 3-deep FIFO. Reading a single payload per
         * loop lets it overflow (FIFO_STATUS showed RX_FULL) and silently
         * drops the rest. RX_EMPTY is FIFO_STATUS bit 0. */
        while (!(rf_rd(REG_FIFO_STATUS) & 0x01)) {
            rf_rx_payload((uint8_t *)&pkt, RF_PAYLOAD);
            rf_wr(REG_STATUS, ST_RX_DR);
            if (pkt_valid(&pkt)) { good++; last_pkt = now; }
            else                 { bad++; }
        }

        uint32_t link_up = (now - last_pkt < FAILSAFE_MS);
        uint32_t want = link_up && (pkt.flags & PKT_ARM);

        if (!want) {
            if (motors_live) motors_off();
        } else {
            if (!motors_live) motors_engage();
            motors_set(pkt.throttle);
        }

        /* LED: fast while the link is up, slow when it is not */
        uint32_t period = link_up ? 100 : 500;
        if (now - last_led >= period) {
            last_led = now;
            led ^= 1;
            if (led) pa_hi(PIN_LED); else pa_lo(PIN_LED);
        }

        /* Refresh the queued ack payload. Only one can be pending, so it is
         * flushed first: a stale reading is worse than a missed one. */
        if (now - last_tlm >= 20u) {
            last_tlm = now;
            uint8_t b[14];
            tlm.magic = TLM_MAGIC;
            tlm.seq++;
            if (mpu_ok && mpu_burst(0x3B, b, 14)) {
                tlm.ax = (int16_t)(((uint16_t)b[0]  << 8) | b[1]);
                tlm.ay = (int16_t)(((uint16_t)b[2]  << 8) | b[3]);
                tlm.az = (int16_t)(((uint16_t)b[4]  << 8) | b[5]);
                tlm.gx = (int16_t)(((uint16_t)b[8]  << 8) | b[9]);
                tlm.gy = (int16_t)(((uint16_t)b[10] << 8) | b[11]);
                tlm.gz = (int16_t)(((uint16_t)b[12] << 8) | b[13]);
                tlm.flags = TLM_MPU_OK;
            } else {
                tlm.flags = 0;
            }
            tlm.sum = tlm_sum(&tlm);

            /* No ack payload is queued: this pair will not run auto-ack, and
             * issuing W_ACK_PAYLOAD with the feature disabled only disturbs the
             * receiver. Telemetry goes out over the UART instead. */
        }

        if (now - last_report >= 1000) {
            last_report = now;
            /* Both ends claim to be configured, so print what the radio
             * actually holds and compare the two sides directly. */
            puts_("ch=");     putdec(rf_rd(REG_RF_CH));
            puts_(" a0=0x");  puthex(rf_rd(REG_RX_ADDR_P0));
            puts_(" cfg=0x"); puthex(rf_rd(REG_CONFIG));
            puts_(" st=0x");  puthex(rf_rd(REG_STATUS));
            puts_(" fifo=0x");puthex(rf_rd(REG_FIFO_STATUS));
            puts_(" en_rx=0x"); puthex(rf_rd(REG_EN_RXADDR));
            puts_(" aw=0x");  puthex(rf_rd(REG_SETUP_AW));
            puts_(" pw=");    putdec(rf_rd(REG_RX_PW_P0));
            puts_(" rf=0x");  puthex(rf_rd(REG_RF_SETUP));
            puts_("\r\n");
            puts_("rx good="); putdec(good);
            puts_(" bad=");    putdec(bad);
            puts_(" thr=");    putdec(pkt.throttle);
            puts_(" link=");   puts_(link_up ? "UP" : "DOWN");
            puts_(" motor=");  puts_(motors_live ? "ON" : "off");
            puts_("\r\n");
        }

        /* One line the host can parse: signed raw counts, no trig here.
         * The firmware links without libm and the host has far more
         * precision to spend on the arctangent anyway. 10 Hz keeps the
         * attitude display fluid while filling under half of the 9600-baud
         * line; the radio FIFO is 3 deep, enough to ride out each print. */
        if (now - last_imu >= 100u) {
            last_imu = now;
            puts_("IMU ");
            puts_(mpu_ok ? "ok " : "mat ");
            putsig(tlm.ax); putc_(' ');
            putsig(tlm.ay); putc_(' ');
            putsig(tlm.az); putc_(' ');
            putsig(tlm.gx); putc_(' ');
            putsig(tlm.gy); putc_(' ');
            putsig(tlm.gz);
            puts_("\r\n");
        }
    }
}
