/* STM32F031K6 pin scanner
 *
 * Discovers board wiring by probing GPIO directly:
 *   phase 1  census   - classify every candidate pin as driven-high / driven-low / floating
 *   phase 2  i2c      - bit-bang every (SCL,SDA) pair looking for MPU6050
 *   phase 3  spi      - bit-bang every (CSN,SCK,MOSI) triple looking for BK2425,
 *                       sampling the whole IDR each clock so MISO falls out for free
 *   phase 4  report   - bit-bang the report out of every candidate pin in turn, so the
 *                       one actually wired to the USB-serial adapter is heard without
 *                       knowing in advance which pin it is
 *   phase 9  interact - host drives one pin at a time (high / low / blink) over SWD
 *
 * Results also land in a struct at 0x20000000, readable with openocd.
 * PA13/PA14 are never touched: they are the SWD link this all depends on.
 *
 * Clock is the 8 MHz HSI default, no PLL. Bit timing comes off SysTick, not
 * nop loops, so the UART framing survives compiler changes.
 */
#include <stdint.h>

#define RCC_AHBENR  (*(volatile uint32_t *)0x40021014u)
#define IOPAEN      (1u << 17)
#define IOPBEN      (1u << 18)
#define IOPFEN      (1u << 22)

#define STK_CTRL    (*(volatile uint32_t *)0xE000E010u)
#define STK_LOAD    (*(volatile uint32_t *)0xE000E014u)
#define STK_VAL     (*(volatile uint32_t *)0xE000E018u)

#define F_CPU       8000000u
#define BAUD        9600u
#define BIT_CYCLES  (F_CPU / BAUD)      /* 833 */

typedef struct {
    volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2], BRR;
} gpio_t;

#define GPIOA ((gpio_t *)0x48000000u)
#define GPIOB ((gpio_t *)0x48000400u)
#define GPIOF ((gpio_t *)0x48001400u)

/* pin code = port index << 4 | pin number; port 0=A 1=B 2=F */
#define PC(port, pin) (uint8_t)(((port) << 4) | (pin))
#define PORT_OF(c) ((c) >> 4)
#define PIN_OF(c)  ((c) & 15)

static gpio_t *const PORTS[3] = { GPIOA, GPIOB, GPIOF };
static const char PORTCH[3] = { 'A', 'B', 'F' };

/* every GPIO broken out on LQFP32, minus the two SWD pins */
static const uint8_t CAND[] = {
    PC(0,0),  PC(0,1),  PC(0,2),  PC(0,3),  PC(0,4),  PC(0,5),  PC(0,6),  PC(0,7),
    PC(0,8),  PC(0,9),  PC(0,10), PC(0,11), PC(0,12), PC(0,15),
    PC(1,0),  PC(1,1),  PC(1,3),  PC(1,4),  PC(1,5),  PC(1,6),  PC(1,7),  PC(1,8),
    PC(2,0),  PC(2,1),
};
#define NCAND (sizeof(CAND) / sizeof(CAND[0]))

typedef struct { uint8_t scl, sda, addr, who; } i2c_hit_t;
typedef struct { uint8_t csn, sck, mosi, miso, val, pad[3]; } spi_hit_t;

typedef struct {
    uint32_t  magic;          /* 'PSC1' once the scan has finished          */
    uint32_t  phase;          /* live progress marker                       */
    uint32_t  npins;
    uint8_t   pins[32];
    uint32_t  census_pu[3];   /* per port: level read with internal pull-up */
    uint32_t  census_pd[3];   /* per port: level read with internal pull-dn */
    uint32_t  i2c_n;
    i2c_hit_t i2c[8];
    uint32_t  spi_n;
    spi_hit_t spi[8];
    volatile uint32_t cmd;      /* host: (mode << 16) | pincode             */
    volatile uint32_t cmd_seq;  /* host bumps this to submit cmd            */
    volatile uint32_t ack_seq;  /* firmware echoes it back                  */
    volatile uint32_t tx_pin;   /* host: pin to talk on, 0xFF = try them all */
    volatile uint32_t mask[3];  /* host: per-port pin mask for group drive     */
    volatile uint32_t mpu_seq;  /* bumped after every successful sensor read    */
    volatile uint32_t mpu_ok;   /* 1 once the device answered and left sleep    */
    volatile int16_t  mpu[7];   /* ax ay az temp gx gy gz, raw big-endian regs  */
    volatile int16_t  mpu_pad;
} res_t;

res_t res __attribute__((section(".results")));

/* ---- timing ---- */
static void wait_cycles(uint32_t n)
{
    uint32_t start = STK_VAL;
    while (((start - STK_VAL) & 0xFFFFFFu) < n) { }
}
static inline void dly(volatile uint32_t n) { while (n--) __asm__ volatile("nop"); }

/* ---- low level pin control ---- */
static void pin_in(uint8_t c, uint32_t pull)   /* pull: 0 none, 1 up, 2 down */
{
    gpio_t *g = PORTS[PORT_OF(c)];
    uint32_t p = PIN_OF(c);
    g->MODER &= ~(3u << (p * 2));
    g->PUPDR  = (g->PUPDR & ~(3u << (p * 2))) | (pull << (p * 2));
}

static void pin_out(uint8_t c)
{
    gpio_t *g = PORTS[PORT_OF(c)];
    uint32_t p = PIN_OF(c);
    g->OTYPER &= ~(1u << p);
    g->PUPDR  &= ~(3u << (p * 2));
    g->MODER   = (g->MODER & ~(3u << (p * 2))) | (1u << (p * 2));
}

static inline void pin_hi(uint8_t c) { PORTS[PORT_OF(c)]->BSRR = 1u << PIN_OF(c); }
static inline void pin_lo(uint8_t c) { PORTS[PORT_OF(c)]->BRR  = 1u << PIN_OF(c); }
static inline uint32_t pin_rd(uint8_t c) { return (PORTS[PORT_OF(c)]->IDR >> PIN_OF(c)) & 1u; }

static void all_idle(void)
{
    for (uint32_t i = 0; i < NCAND; i++) pin_in(CAND[i], 2);
}

/* ---- bit-banged UART transmit ---- */
static uint8_t g_tx = 0xFF;

static void tx_setup(uint8_t pin)
{
    g_tx = pin;
    pin_hi(pin);        /* idle high before enabling the driver */
    pin_out(pin);
}

static void tx_char(char ch)
{
    if (g_tx == 0xFF) return;
    pin_lo(g_tx);                       /* start bit */
    wait_cycles(BIT_CYCLES);
    for (int i = 0; i < 8; i++) {
        if ((ch >> i) & 1) pin_hi(g_tx); else pin_lo(g_tx);
        wait_cycles(BIT_CYCLES);
    }
    pin_hi(g_tx);                       /* stop bit */
    wait_cycles(BIT_CYCLES * 2);
}

static void tx_str(const char *s) { while (*s) tx_char(*s++); }

static void tx_hex2(uint8_t v)
{
    static const char H[] = "0123456789ABCDEF";
    tx_char(H[v >> 4]);
    tx_char(H[v & 15]);
}

static void tx_pin_name(uint8_t c)
{
    tx_char('P');
    tx_char(PORTCH[PORT_OF(c)]);
    uint32_t n = PIN_OF(c);
    if (n >= 10) { tx_char('1'); tx_char((char)('0' + n - 10)); }
    else tx_char((char)('0' + n));
}

/* Cortex-M0 has no divide instruction and this build links no libgcc, so
 * decimal conversion is done by repeated subtraction. Counts here are tiny. */
static void tx_dec(uint32_t v)
{
    char b[12];
    int i = 0;
    if (!v) { tx_char('0'); return; }
    while (v) {
        uint32_t q = 0;
        while (v >= 10) { v -= 10; q++; }
        b[i++] = (char)('0' + v);
        v = q;
    }
    while (i--) tx_char(b[i]);
}

/* ---- phase 1: census ---- */
static void census(void)
{
    for (uint32_t i = 0; i < NCAND; i++) pin_in(CAND[i], 1);
    dly(20000);
    for (uint32_t k = 0; k < 3; k++) res.census_pu[k] = PORTS[k]->IDR;

    for (uint32_t i = 0; i < NCAND; i++) pin_in(CAND[i], 2);
    dly(20000);
    for (uint32_t k = 0; k < 3; k++) res.census_pd[k] = PORTS[k]->IDR;
}

/* ---- phase 2: bit-banged I2C, open-drain emulated by switching direction ---- */
#define I2C_HALF 30

static uint8_t g_scl, g_sda;

static void sda_release(void) { pin_in(g_sda, 1); }
static void sda_pull(void)    { pin_lo(g_sda); pin_out(g_sda); }
static void scl_release(void) { pin_in(g_scl, 1); }
static void scl_pull(void)    { pin_lo(g_scl); pin_out(g_scl); }

static void i2c_start(void)
{
    sda_release(); scl_release(); dly(I2C_HALF);
    sda_pull();    dly(I2C_HALF);
    scl_pull();    dly(I2C_HALF);
}

static void i2c_stop(void)
{
    sda_pull();    dly(I2C_HALF);
    scl_release(); dly(I2C_HALF);
    sda_release(); dly(I2C_HALF);
}

static uint32_t i2c_wr(uint8_t b)   /* returns 1 on ACK */
{
    for (int i = 7; i >= 0; i--) {
        if ((b >> i) & 1) sda_release(); else sda_pull();
        dly(I2C_HALF);
        scl_release(); dly(I2C_HALF);
        scl_pull();    dly(I2C_HALF);
    }
    sda_release(); dly(I2C_HALF);
    scl_release(); dly(I2C_HALF);
    uint32_t ack = !pin_rd(g_sda);
    scl_pull();    dly(I2C_HALF);
    return ack;
}

static uint8_t i2c_rd(uint32_t ack)
{
    uint8_t v = 0;
    sda_release();
    for (int i = 7; i >= 0; i--) {
        dly(I2C_HALF);
        scl_release(); dly(I2C_HALF);
        v |= (uint8_t)(pin_rd(g_sda) << i);
        scl_pull();    dly(I2C_HALF);
    }
    if (ack) sda_pull(); else sda_release();
    dly(I2C_HALF);
    scl_release(); dly(I2C_HALF);
    scl_pull();    dly(I2C_HALF);
    sda_release();
    return v;
}

static void i2c_scan(void)
{
    static const uint8_t ADDRS[2] = { 0xD0, 0xD2 };   /* MPU6050, AD0 low / high */

    for (uint32_t a = 0; a < NCAND && res.i2c_n < 8; a++) {
        for (uint32_t b = 0; b < NCAND && res.i2c_n < 8; b++) {
            if (a == b) continue;
            g_scl = CAND[a];
            g_sda = CAND[b];
            all_idle();
            sda_release(); scl_release();
            dly(500);
            /* both lines must idle high, otherwise this pair is not a bus */
            if (!pin_rd(g_scl) || !pin_rd(g_sda)) continue;

            for (uint32_t k = 0; k < 2; k++) {
                i2c_start();
                if (!i2c_wr(ADDRS[k])) { i2c_stop(); continue; }
                if (!i2c_wr(0x75)) { i2c_stop(); continue; }      /* WHO_AM_I */
                i2c_start();
                if (!i2c_wr((uint8_t)(ADDRS[k] | 1))) { i2c_stop(); continue; }
                uint8_t who = i2c_rd(0);
                i2c_stop();

                res.i2c[res.i2c_n].scl  = g_scl;
                res.i2c[res.i2c_n].sda  = g_sda;
                res.i2c[res.i2c_n].addr = ADDRS[k];
                res.i2c[res.i2c_n].who  = who;
                res.i2c_n++;
                break;
            }
        }
    }
    all_idle();
}

/* ---- phase 3: bit-banged SPI, MISO recovered by sampling the whole IDR ---- */
#define SPI_HALF 3

static void spi_scan(void)
{
    uint32_t samp[3][8];

    for (uint32_t a = 0; a < NCAND && res.spi_n < 8; a++) {
      for (uint32_t b = 0; b < NCAND && res.spi_n < 8; b++) {
        if (b == a) continue;
        for (uint32_t c = 0; c < NCAND && res.spi_n < 8; c++) {
            if (c == a || c == b) continue;
            uint8_t csn = CAND[a], sck = CAND[b], mosi = CAND[c];

            all_idle();
            pin_hi(csn);  pin_out(csn);
            pin_lo(sck);  pin_out(sck);
            pin_lo(mosi); pin_out(mosi);
            dly(SPI_HALF);

            pin_lo(csn);
            dly(SPI_HALF);

            /* command 0x0A = R_REGISTER | RX_ADDR_P0 */
            for (int i = 7; i >= 0; i--) {
                if ((0x0A >> i) & 1) pin_hi(mosi); else pin_lo(mosi);
                dly(SPI_HALF);
                pin_hi(sck); dly(SPI_HALF);
                pin_lo(sck); dly(SPI_HALF);
            }
            /* clock out one payload byte, sampling every port on each rising edge */
            for (int i = 7; i >= 0; i--) {
                pin_lo(mosi);
                dly(SPI_HALF);
                pin_hi(sck);
                samp[0][i] = GPIOA->IDR;
                samp[1][i] = GPIOB->IDR;
                samp[2][i] = GPIOF->IDR;
                dly(SPI_HALF);
                pin_lo(sck); dly(SPI_HALF);
            }
            pin_hi(csn);

            for (uint32_t m = 0; m < NCAND; m++) {
                uint8_t mp = CAND[m];
                if (mp == csn || mp == sck || mp == mosi) continue;
                uint8_t v = 0;
                for (int i = 7; i >= 0; i--)
                    v |= (uint8_t)(((samp[PORT_OF(mp)][i] >> PIN_OF(mp)) & 1u) << i);
                if (v != 0xE7) continue;          /* RX_ADDR_P0 power-on default */

                res.spi[res.spi_n].csn  = csn;
                res.spi[res.spi_n].sck  = sck;
                res.spi[res.spi_n].mosi = mosi;
                res.spi[res.spi_n].miso = mp;
                res.spi[res.spi_n].val  = v;
                res.spi_n++;
                break;
            }
        }
      }
    }
    all_idle();
}

/* ---- MPU6050 live read, phase 6 ----
 * The scan already proved the device ACKs on PB6/PB7. This goes further and
 * streams real sample data, so a host watching res.mpu across reads can tell a
 * live sensor from one that merely acknowledges its address. */
#define MPU_ADDR 0xD0

static uint32_t mpu_write(uint8_t reg, uint8_t val)
{
    i2c_start();
    if (!i2c_wr(MPU_ADDR)) { i2c_stop(); return 0; }
    if (!i2c_wr(reg))      { i2c_stop(); return 0; }
    if (!i2c_wr(val))      { i2c_stop(); return 0; }
    i2c_stop();
    return 1;
}

static uint32_t mpu_burst(uint8_t reg, uint8_t *buf, uint32_t n)
{
    i2c_start();
    if (!i2c_wr(MPU_ADDR)) { i2c_stop(); return 0; }
    if (!i2c_wr(reg))      { i2c_stop(); return 0; }
    i2c_start();
    if (!i2c_wr(MPU_ADDR | 1)) { i2c_stop(); return 0; }
    for (uint32_t i = 0; i < n; i++) buf[i] = i2c_rd(i + 1 < n);
    i2c_stop();
    return 1;
}

static void mpu_begin(void)
{
    g_scl = PC(1, 6);          /* PB6, from the scan */
    g_sda = PC(1, 7);          /* PB7, from the scan */
    all_idle();
    sda_release(); scl_release();
    dly(2000);
    /* PWR_MGMT_1: clear SLEEP, run off the X gyro PLL */
    res.mpu_ok = mpu_write(0x6B, 0x01);
    dly(20000);
    mpu_write(0x1B, 0x00);     /* GYRO_CONFIG  +-250 dps  */
    mpu_write(0x1C, 0x00);     /* ACCEL_CONFIG +-2 g      */
    mpu_write(0x19, 0x07);     /* sample rate divider     */
}

static void mpu_poll(void)
{
    uint8_t b[14];
    if (!mpu_burst(0x3B, b, 14)) { res.mpu_ok = 0; return; }
    for (uint32_t i = 0; i < 7; i++)
        res.mpu[i] = (int16_t)(((uint16_t)b[i * 2] << 8) | b[i * 2 + 1]);
    res.mpu_ok = 1;
    res.mpu_seq++;
}

/* ---- report ---- */
static void report(void)
{
    tx_str("\r\n== PINSCAN tx=");
    tx_pin_name(g_tx);
    tx_str(" ==\r\nCENSUS ");
    for (uint32_t i = 0; i < NCAND; i++) {
        uint8_t c = CAND[i];
        uint32_t pu = (res.census_pu[PORT_OF(c)] >> PIN_OF(c)) & 1u;
        uint32_t pd = (res.census_pd[PORT_OF(c)] >> PIN_OF(c)) & 1u;
        tx_pin_name(c);
        tx_char('=');
        tx_str(pu && pd ? "HI " : (!pu && !pd ? "LO " : "-- "));
    }
    tx_str("\r\nI2C hits ");
    tx_dec(res.i2c_n);
    tx_str("\r\n");
    for (uint32_t i = 0; i < res.i2c_n; i++) {
        tx_str("  SCL="); tx_pin_name(res.i2c[i].scl);
        tx_str(" SDA=");  tx_pin_name(res.i2c[i].sda);
        tx_str(" addr=0x"); tx_hex2(res.i2c[i].addr);
        tx_str(" WHO_AM_I=0x"); tx_hex2(res.i2c[i].who);
        tx_str("\r\n");
    }
    tx_str("SPI hits ");
    tx_dec(res.spi_n);
    tx_str("\r\n");
    for (uint32_t i = 0; i < res.spi_n; i++) {
        tx_str("  CSN=");  tx_pin_name(res.spi[i].csn);
        tx_str(" SCK=");   tx_pin_name(res.spi[i].sck);
        tx_str(" MOSI=");  tx_pin_name(res.spi[i].mosi);
        tx_str(" MISO=");  tx_pin_name(res.spi[i].miso);
        tx_str(" val=0x"); tx_hex2(res.spi[i].val);
        tx_str("\r\n");
    }
    tx_str("READY\r\n");
}

/* ---- phase 9: host-driven pin exercise ---- */
#define HB_PIN     PC(0, 1)          /* PA1, the LED found by the scan */
#define HB_CYCLES  2000000u          /* ~0.25 s at 8 MHz -> 2 Hz blink */

static void interact(void)
{
    uint32_t seq = res.ack_seq;
    uint8_t  active = 0xFF;
    uint32_t mode = 0, level = 0;

    /* Heartbeat: the LED blinks whenever the firmware is in its main loop, so a
     * glance at the board says whether it is alive without attaching anything.
     * It stands down if a command targets PA1 itself. */
    uint32_t hb_on = 1, hb_level = 0, hb_acc = 0, hb_last = STK_VAL;
    pin_lo(HB_PIN); pin_out(HB_PIN);

    for (;;) {
        if (res.cmd_seq != seq) {
            seq = res.cmd_seq;
            uint32_t cmd = res.cmd;
            uint8_t  pin = (uint8_t)(cmd & 0xFF);
            uint8_t  keep_tx = (uint8_t)res.tx_pin;

            all_idle();
            if (keep_tx != 0xFF) tx_setup(keep_tx);

            hb_on = !(pin == HB_PIN && mode >= 1 && mode <= 3)
                 && !((res.mask[PORT_OF(HB_PIN)] >> PIN_OF(HB_PIN)) & 1u);
            if (hb_on) { pin_lo(HB_PIN); pin_out(HB_PIN); hb_level = 0; }

            mode   = (cmd >> 16) & 0xFF;
            active = pin;
            level  = 0;

            if (mode == 1)      { pin_hi(active); pin_out(active); }
            else if (mode == 2) { pin_lo(active); pin_out(active); }
            else if (mode == 3) { pin_lo(active); pin_out(active); }
            else if (mode == 4 || mode == 5) {
                /* group drive: every candidate pin selected in res.mask.
                 * CAND already excludes PA13/PA14, so SWD can never be driven. */
                for (uint32_t i = 0; i < NCAND; i++) {
                    uint8_t c = CAND[i];
                    if (!((res.mask[PORT_OF(c)] >> PIN_OF(c)) & 1u)) continue;
                    if (mode == 4) pin_hi(c); else pin_lo(c);
                    pin_out(c);
                }
                active = 0xFF;
            }
            else if (mode == 6) { mpu_begin(); active = 0xFF; }
            else                { active = 0xFF; }

            if (keep_tx != 0xFF && active != 0xFF) {
                tx_str("DRIVE ");
                tx_pin_name(active);
                tx_str(mode == 1 ? " HIGH\r\n" : mode == 2 ? " LOW\r\n" : " BLINK\r\n");
            }
            res.ack_seq = seq;
        }

        if (hb_on) {                     /* cycle-accurate, so loop speed does not matter */
            uint32_t now = STK_VAL;
            hb_acc += (hb_last - now) & 0xFFFFFFu;
            hb_last = now;
            if (hb_acc >= HB_CYCLES) {
                hb_acc = 0;
                hb_level ^= 1;
                if (hb_level) pin_hi(HB_PIN); else pin_lo(HB_PIN);
            }
        }

        if (mode == 6) { mpu_poll(); dly(20000); }

        if (mode == 3 && active != 0xFF) {      /* ~5 Hz: visible on an LED, readable on a meter */
            level ^= 1;
            if (level) pin_hi(active); else pin_lo(active);
            dly(200000);
        }
    }
}

int main(void)
{
    RCC_AHBENR |= IOPAEN | IOPBEN | IOPFEN;

    STK_LOAD = 0x00FFFFFFu;
    STK_VAL  = 0;
    STK_CTRL = 5;                 /* enable, processor clock, no interrupt */

    res.magic  = 0;
    res.npins  = NCAND;
    for (uint32_t i = 0; i < NCAND; i++) res.pins[i] = CAND[i];
    res.i2c_n  = 0;
    res.spi_n  = 0;
    res.cmd    = 0;
    res.cmd_seq = 0;
    res.ack_seq = 0;
    res.tx_pin  = 0xFF;
    res.mask[0] = res.mask[1] = res.mask[2] = 0;
    res.mpu_seq = 0;
    res.mpu_ok = 0;

    res.phase = 1; census();
    res.phase = 2; i2c_scan();
    res.phase = 3; spi_scan();

    /* UART broadcast is parked: the host reads results over SWD instead, and
     * skipping it keeps every reset fast. Re-enable by restoring the loop. */
    res.phase = 4;
    if (res.tx_pin != 0xFF) { tx_setup((uint8_t)res.tx_pin); report(); }

    res.magic = 0x50534331;   /* 'PSC1' */
    res.phase = 9;
    all_idle();
    if (res.tx_pin != 0xFF) tx_setup((uint8_t)res.tx_pin);
    interact();
    return 0;
}
