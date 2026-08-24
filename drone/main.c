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

static void motors_write(int32_t m1, int32_t m2, int32_t m3, int32_t m4)
{
    if (m1 < 0) m1 = 0;
    if (m2 < 0) m2 = 0;
    if (m3 < 0) m3 = 0;
    if (m4 < 0) m4 = 0;
    if (m1 > (int32_t)PWM_ARR) m1 = PWM_ARR;
    if (m2 > (int32_t)PWM_ARR) m2 = PWM_ARR;
    if (m3 > (int32_t)PWM_ARR) m3 = PWM_ARR;
    if (m4 > (int32_t)PWM_ARR) m4 = PWM_ARR;
    TIM2->CCR3 = (uint32_t)m1;          /* M1 PA2  */
    TIM2->CCR4 = (uint32_t)m2;          /* M2 PA3  */
    TIM1->CCR1 = (uint32_t)m3;          /* M3 PA8  */
    TIM1->CCR4 = (uint32_t)m4;          /* M4 PA11 */
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
    /* Bus recovery: an MCU reset mid-transaction can leave the MPU6050
     * holding SDA low. Nine clock pulses and a STOP release it. */
    for (uint32_t i = 0; i < 9; i++) {
        pb_low(SCL); ib_dly(I2C_HALF);
        pb_in(SCL);  ib_dly(I2C_HALF);
    }
    i2c_stop();
    uint32_t up = 0;
    for (uint32_t i = 0; i < 5 && !up; i++) {   /* retry: sensor may still be waking */
        up = mpu_wr(0x6B, 0x01);                /* leave sleep, clock off the X gyro */
        if (!up) delay_ms(20);
    }
    if (!up) return 0;
    delay_ms(20);
    mpu_wr(0x1A, 0x03);                  /* DLPF 44 Hz: keeps prop vibration
                                            out of the gyro the PID feeds on */
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

/* ---- flight control ----
 *
 * Cascade-free toy-quad stabiliser, all integer, all shifts: no divide
 * instruction and no libgcc/libm in this build.
 *
 * Angle bookkeeping: the accumulators add one gyro sample (LSB, 131 per
 * deg/s at +-250 dps) every CTL_MS, so one degree of attitude equals
 * 131 * (1000/CTL_MS) = 26200 units. Accelerometer angles use the
 * small-angle approximation: ay counts * 92 = the same fixed-point degrees
 * (16384 counts/g, 57.3 deg/rad: 57.3/16384 * 26200 = 91.6).
 *
 * The complementary filter drags the gyro integral toward the accel angle
 * by 1/128 per cycle: tau = 128 * 5 ms = 0.64 s.
 *
 * MOTOR LAYOUT (MIX table) IS AN ASSUMPTION until the motor-test mode has
 * been used to map M1..M4 pads to airframe corners. Do not fly before
 * checking it: use 'm' on the ground console (motor test), y 0/64/128/192
 * to pick the motor, small throttle to spin it, and note which corner moves.
 */
#define CTL_MS       5
#define ONE_DEG      26200
#define ACC_TO_FP    92
#define MAX_SP_FP    (30 * ONE_DEG)      /* full stick = 30 degrees      */
#define TILT_CUT_FP  (60 * ONE_DEG)      /* auto-cut past 60 degrees     */
#define KP           10                  /* ~4 duty counts per degree    */
#define KD           2                   /* ~50 counts at 100 deg/s      */
#define KI           1
#define I_CLAMP      (40 << 12)
#define KPY          2                   /* yaw rate P                   */

static int32_t roll_fp, pitch_fp;        /* fused attitude               */
static int16_t gx0, gy0, gz0;            /* gyro bias from calibration   */
static int32_t i_roll, i_pitch;
static uint32_t armed, crash_latch;

/* {roll, pitch, yaw} sign per motor — see the layout warning above. */
static const int8_t MIX[4][3] = {
    { -1, +1, -1 },   /* M1 PA2:  assumed front-right, CCW */
    { +1, -1, -1 },   /* M2 PA3:  assumed rear-left,  CCW */
    { +1, +1, +1 },   /* M3 PA8:  assumed front-left,  CW */
    { -1, -1, +1 },   /* M4 PA11: assumed rear-right,  CW */
};

/* 256 still samples, ~0.9 s. Board must not move during power-up. */
static void gyro_calibrate(void)
{
    int32_t sx = 0, sy = 0, sz = 0;
    uint8_t b[14];
    for (uint32_t i = 0; i < 256; i++) {
        if (mpu_burst(0x3B, b, 14)) {
            sx += (int16_t)(((uint16_t)b[8]  << 8) | b[9]);
            sy += (int16_t)(((uint16_t)b[10] << 8) | b[11]);
            sz += (int16_t)(((uint16_t)b[12] << 8) | b[13]);
        }
        delay_ms(2);
    }
    gx0 = (int16_t)(sx >> 8);
    gy0 = (int16_t)(sy >> 8);
    gz0 = (int16_t)(sz >> 8);
}

/* One 200 Hz step: fuse attitude, run the PIDs, mix, write the motors.
 * Inputs are the latest stick packet and the fresh IMU sample. */
static void flight_step(const pkt_t *cmd, int16_t ax, int16_t ay, int16_t az,
                        int16_t gx, int16_t gy, int16_t gz)
{
    (void)az;
    int32_t rrate = gx - gx0;            /* roll  rate, gyro LSB */
    int32_t prate = gy - gy0;            /* pitch rate           */
    int32_t yrate = gz - gz0;            /* yaw   rate           */

    roll_fp  += rrate;
    pitch_fp += prate;
    roll_fp  += (( (int32_t)ay * ACC_TO_FP) - roll_fp)  >> 7;
    pitch_fp += ((-(int32_t)ax * ACC_TO_FP) - pitch_fp) >> 7;

    if (!armed) return;

    /* Tilt beyond the cut angle while armed: something is upside down or
     * tumbling. Kill the motors and stay dead until a fresh arm. */
    int32_t r_abs = roll_fp  < 0 ? -roll_fp  : roll_fp;
    int32_t p_abs = pitch_fp < 0 ? -pitch_fp : pitch_fp;
    if (r_abs > TILT_CUT_FP || p_abs > TILT_CUT_FP) {
        crash_latch = 1;
        armed = 0;
        motors_off();
        return;
    }

    int32_t base = ((int32_t)cmd->throttle * (PWM_ARR + 1)) >> 8;
    if (base < 20) {
        /* idle: props barely turning, no authority — hold everything level
         * later, do not let the integrators wind up on the ground */
        i_roll = i_pitch = 0;
        motors_write(base, base, base, base);
        return;
    }

    int32_t sp_r = ((int32_t)cmd->roll  - 128) * (MAX_SP_FP >> 7);   /* 6140/LSB */
    int32_t sp_p = ((int32_t)cmd->pitch - 128) * (MAX_SP_FP >> 7);
    int32_t err_r = sp_r - roll_fp;
    int32_t err_p = sp_p - pitch_fp;

    i_roll  += err_r >> 10;
    i_pitch += err_p >> 10;
    if (i_roll  >  I_CLAMP) i_roll  =  I_CLAMP;
    if (i_roll  < -I_CLAMP) i_roll  = -I_CLAMP;
    if (i_pitch >  I_CLAMP) i_pitch =  I_CLAMP;
    if (i_pitch < -I_CLAMP) i_pitch = -I_CLAMP;

    int32_t out_r = ((err_r * KP) >> 16) + ((i_roll  * KI) >> 12) - ((rrate * KD) >> 9);
    int32_t out_p = ((err_p * KP) >> 16) + ((i_pitch * KI) >> 12) - ((prate * KD) >> 9);

    /* Yaw is rate-only: full stick asks for ~180 deg/s (184 gyro LSB per
     * stick LSB), P alone brings it there. */
    int32_t sp_y  = ((int32_t)cmd->yaw - 128) * 184;
    int32_t out_y = ((sp_y - yrate) * KPY) >> 10;

    motors_write(base + MIX[0][0]*out_r + MIX[0][1]*out_p + MIX[0][2]*out_y,
                 base + MIX[1][0]*out_r + MIX[1][1]*out_p + MIX[1][2]*out_y,
                 base + MIX[2][0]*out_r + MIX[2][1]*out_p + MIX[2][2]*out_y,
                 base + MIX[3][0]*out_r + MIX[3][1]*out_p + MIX[3][2]*out_y);
}

static uint8_t dbg_cfg, dbg_st, dbg_fifo;

/* Half-duplex telemetry burst.
 *
 * CE is strapped high, so the mode can never be changed the documented way.
 * Writing PRIM_RX on the live chip updates the register but not the state
 * machine: with CONFIG reading 0x0E (TX) the chip demonstrably kept
 * receiving (RX_DR kept setting) and the queued payload sat in the TX FIFO.
 * The state machine only samples PRIM_RX when it leaves power-down, so the
 * mode is changed by bouncing PWR_UP: power down, wake straight into TX
 * (CE already high and the FIFO survives power-down, so the payload leaves
 * as soon as the radio settles), then bounce again to wake back into RX.
 * Costs a few ms of deafness per burst. Returns 1 on TX_DS. */
static uint32_t rf_send_tlm(const tlm_t *t)
{
    /* Give the ground unit time to finish its own PWR_UP bounce into RX;
     * bursting immediately would land while its radio is still waking. */
    delay_ms(4);

    rf_wr(REG_CONFIG, CFG_EN_CRC | CFG_CRCO);                 /* power down  */
    rf_wr(REG_CONFIG, CFG_EN_CRC | CFG_CRCO | CFG_PWR_UP);    /* wake into TX */
    delay_ms(3);                                              /* PLL settle   */

    /* Payload written only once the chip is awake in TX mode — the exact
     * sequence REVERSE_TEST proved over the air. Writing it before the
     * power-down bounce made TX_DS assert but nothing valid left the
     * antenna, so the FIFO does not reliably survive power-down here. */
    pa_lo(PIN_CSN);
    spi_xfer(CMD_W_TX_PAYLOAD);
    for (uint32_t i = 0; i < TLM_LEN; i++) spi_xfer(((const uint8_t *)t)[i]);
    pa_hi(PIN_CSN);

    uint32_t ok = 0, t0 = millis();
    while (millis() - t0 < 8u) {
        if (rf_rd(REG_STATUS) & ST_TX_DS) { ok = 1; break; }
    }
    if (!ok) { dbg_cfg = rf_rd(REG_CONFIG); dbg_st = rf_rd(REG_STATUS);
               dbg_fifo = rf_rd(REG_FIFO_STATUS); }
    rf_wr(REG_STATUS, ST_TX_DS | ST_MAX_RT);
    if (!ok) rf_cmd(CMD_FLUSH_TX);

    rf_wr(REG_CONFIG, CFG_EN_CRC | CFG_CRCO);                 /* power down  */
    rf_wr(REG_CONFIG, CFG_EN_CRC | CFG_CRCO | CFG_PWR_UP | CFG_PRIM_RX); /* RX */
    return ok;
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
    /* The radio powers up more slowly than the MCU: straight after battery
     * plug-in, writes are silently ignored and every register reads its
     * power-on default (RF_CH=2, RX_ADDR_P0=0xE7..., CONFIG=0x08). That is
     * why the link historically only came up on some power-ups. Configure,
     * read back, and retry until the settings actually stick. */
    delay_ms(50);
    for (uint32_t i = 0; i < 100; i++) {
        rf_init_rx();
        if (rf_rd(REG_RF_CH) == RF_CHANNEL && rf_rd(REG_RX_PW_P0) == RF_PAYLOAD)
            break;
        delay_ms(20);
    }
    puts_("CONFIG before=0x"); puthex(cfg_check);
    puts_(" after=0x"); puthex(rf_rd(REG_CONFIG));
    puts_(" RF_CH="); putdec(rf_rd(REG_RF_CH));
    puts_("\r\n");

    pkt_t pkt;
    uint32_t good = 0, bad = 0;
    tlm_t tlm = { TLM_MAGIC, 0, 0,0,0, 0,0,0, 0, 0 };
    uint32_t last_tlm = 0, last_imu = 0, last_ctl = 0, last_att = 0;
    (void)last_tlm;
    uint32_t tlm_due = 0, tlm_sent = 0, tlm_fail = 0, reinits = 0;
    uint32_t mpu_ok = mpu_begin();
    if (mpu_ok) gyro_calibrate();        /* ~0.9 s; keep the drone still */
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
            if (pkt_valid(&pkt)) { good++; last_pkt = now; tlm_due++; }
            else                 { bad++; }
        }

        /* Answer every 5th control packet with a telemetry burst (~8 Hz).
         * Sending right after a reception lands inside the ground unit's
         * listening window, and only ever transmitting immediately after a
         * good packet guarantees the ground's own transmitter is idle. */
        if (tlm_due >= (armed ? 25u : 5u)) {
            tlm_due = 0;
            tlm.sum = tlm_sum(&tlm);
            if (rf_send_tlm(&tlm)) tlm_sent++; else tlm_fail++;
        }

        uint32_t link_up = (now - last_pkt < FAILSAFE_MS);

        /* Arm only on a rising edge with the throttle low; the crash latch
         * (tilt cut) holds everything dead until a disarm clears it. */
        if (!(pkt.flags & PKT_ARM)) { armed = 0; crash_latch = 0; }
        else if (!armed && !crash_latch && pkt.throttle < 10 && link_up) {
            armed = 1;
            i_roll = i_pitch = 0;
        }
        if (!link_up) armed = 0;

        uint32_t mtest = link_up && !armed && !crash_latch
                         && (pkt.flags & PKT_MTEST);
        uint32_t want = (link_up && armed) || mtest;

        if (!want) {
            if (motors_live) motors_off();
        } else if (!motors_live) {
            motors_engage();
        }

        if (mtest && motors_live) {
            /* Single-motor identification: yaw stick picks the pad, the
             * throttle (capped) spins only that one. Map pads to corners
             * with this BEFORE trusting the MIX table. */
            uint32_t idx  = (pkt.yaw >> 6) & 3;
            uint32_t thr  = pkt.throttle > 64 ? 64 : pkt.throttle;
            int32_t  duty = (int32_t)((thr * (PWM_ARR + 1)) >> 8);
            motors_write(idx == 0 ? duty : 0, idx == 1 ? duty : 0,
                         idx == 2 ? duty : 0, idx == 3 ? duty : 0);
        }

        /* 200 Hz stabilisation: read the IMU and run the controller. The
         * same fresh sample feeds the telemetry record. */
        if (now - last_ctl >= CTL_MS) {
            last_ctl = now;
            uint8_t b[14];
            if (mpu_ok && mpu_burst(0x3B, b, 14)) {
                int16_t ax = (int16_t)(((uint16_t)b[0]  << 8) | b[1]);
                int16_t ay = (int16_t)(((uint16_t)b[2]  << 8) | b[3]);
                int16_t az = (int16_t)(((uint16_t)b[4]  << 8) | b[5]);
                int16_t gx = (int16_t)(((uint16_t)b[8]  << 8) | b[9]);
                int16_t gy = (int16_t)(((uint16_t)b[10] << 8) | b[11]);
                int16_t gz = (int16_t)(((uint16_t)b[12] << 8) | b[13]);
                /* Runs disarmed too: keeps the attitude estimate warm and
                 * returns before touching the motors. */
                flight_step(&pkt, ax, ay, az, gx, gy, gz);
                tlm.magic = TLM_MAGIC;
                tlm.seq++;
                tlm.ax = ax; tlm.ay = ay; tlm.az = az;
                tlm.gx = gx; tlm.gy = gy; tlm.gz = gz;
                tlm.flags = TLM_MPU_OK;
            } else if (!mpu_ok) {
                tlm.flags = 0;
            }
        }

        /* LED: fast while the link is up, slow when it is not */
        uint32_t period = link_up ? 100 : 500;
        if (now - last_led >= period) {
            last_led = now;
            led ^= 1;
            if (led) pa_hi(PIN_LED); else pa_lo(PIN_LED);
        }


        if (now - last_report >= 1000) {
            last_report = now;
            /* Self-heal: if the radio lost or never took its configuration
             * (brown-out, slow power-up), put it back and say so. */
            if (rf_rd(REG_RF_CH) != RF_CHANNEL) { rf_init_rx(); reinits++; }
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
            puts_(" arm=");    putdec(armed);
            puts_(" crash="); putdec(crash_latch);
            puts_(" tlm=");    putdec(tlm_sent);
            puts_("/");        putdec(tlm_fail);
            puts_(" ri=");     putdec(reinits);
            puts_(" dbg=");    puthex(dbg_cfg); puthex(dbg_st); puthex(dbg_fifo);
            puts_("\r\n");
        }

        /* One line the host can parse: signed raw counts, no trig here.
         * The firmware links without libm and the host has far more
         * precision to spend on the arctangent anyway. 10 Hz keeps the
         * attitude display fluid while filling under half of the 9600-baud
         * line; the radio FIFO is 3 deep, enough to ride out each print. */
        if (now - last_att >= 200u) {
            last_att = now;
            /* fused attitude in centi-degrees: fp/262 done as (fp*250)>>16 */
            puts_("ATT ");
            putsig((int16_t)((roll_fp  / 4 * 250) >> 14));
            putc_(' ');
            putsig((int16_t)((pitch_fp / 4 * 250) >> 14));
            puts_("\r\n");
        }

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
