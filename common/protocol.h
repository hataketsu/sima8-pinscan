/* Link between the Sima X8 board (BK2425) and a Blue Pill ground unit (nRF24L01+).
 *
 * The two radios are register-compatible in bank 0, so both ends run the same
 * on-air settings. Auto-ack is off: the drone's CE line is strapped high, which
 * leaves no way to sequence the CE toggling that ack turnaround wants, and a
 * control link is better served by a steady stream of fresh packets than by
 * retransmits of stale ones.
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define RF_CHANNEL   76           /* the channel the link was first seen working on */
#define RF_ADDR_LEN  5
#define RF_PAYLOAD   8
#define RF_ADDR      { 'S', 'I', 'M', 'A', '8' }

#define PKT_MAGIC    0xA5
#define PKT_ARM      0x01         /* flags bit 0 */

#define TX_PERIOD_MS   20         /* 50 Hz */
#define FAILSAFE_MS   300         /* motors cut after this long with no packet */

/* Byte layout on air, 8 bytes:
 *   0 magic   1 seq   2 throttle   3 roll   4 pitch   5 yaw   6 flags   7 sum
 * Sticks are unsigned: throttle 0 = idle, the other three are centred at 128. */
typedef struct {
    uint8_t magic, seq, throttle, roll, pitch, yaw, flags, sum;
} pkt_t;

static inline uint8_t pkt_sum(const pkt_t *p)
{
    const uint8_t *b = (const uint8_t *)p;
    uint8_t s = 0;
    for (uint32_t i = 0; i < RF_PAYLOAD - 1; i++) s ^= b[i];
    return s;
}

/* Telemetry rides back inside the auto-ack.
 *
 * The drone's CE is strapped high, so it can never be put into standby, and
 * changing PRIM_RX while it is live latches the radio. Ack payloads avoid that
 * entirely: the receiver stays in RX forever, the transmitter stays in TX
 * forever, and the turnaround is done by the radio hardware.
 *
 * Raw sensor counts are sent rather than angles: the firmware links without
 * libm, and the host can do the trigonometry with far more precision anyway. */
#define TLM_MAGIC   0x5A
#define TLM_LEN     16
#define TLM_MPU_OK  0x01

typedef struct {
    uint8_t magic, seq;
    int16_t ax, ay, az, gx, gy, gz;
    uint8_t flags, sum;
} tlm_t;

static inline uint8_t tlm_sum(const tlm_t *t)
{
    const uint8_t *b = (const uint8_t *)t;
    uint8_t s = 0;
    for (uint32_t i = 0; i < TLM_LEN - 1; i++) s ^= b[i];
    return s;
}

static inline int tlm_valid(const tlm_t *t)
{
    return t->magic == TLM_MAGIC && t->sum == tlm_sum(t);
}

static inline int pkt_valid(const pkt_t *p)
{
    return p->magic == PKT_MAGIC && p->sum == pkt_sum(p);
}

/* nRF24L01+ / BK2425 command and register set, bank 0 */
#define CMD_R_REG        0x00
#define CMD_W_REG        0x20
#define CMD_R_RX_PAYLOAD 0x61
#define CMD_W_TX_PAYLOAD 0xA0
#define CMD_W_ACK_PAYLOAD 0xA8
#define CMD_R_RX_PL_WID   0x60
#define CMD_ACTIVATE      0x50
#define CMD_FLUSH_TX     0xE1
#define CMD_FLUSH_RX     0xE2
#define CMD_NOP          0xFF

#define REG_CONFIG      0x00
#define REG_EN_AA       0x01
#define REG_EN_RXADDR   0x02
#define REG_SETUP_AW    0x03
#define REG_SETUP_RETR  0x04
#define REG_RF_CH       0x05
#define REG_RF_SETUP    0x06
#define REG_STATUS      0x07
#define REG_RX_ADDR_P0  0x0A
#define REG_TX_ADDR     0x10
#define REG_RX_PW_P0    0x11
#define REG_FIFO_STATUS 0x17
#define REG_DYNPD       0x1C
#define REG_FEATURE     0x1D

#define FEAT_EN_DPL     0x04
#define FEAT_EN_ACK_PAY 0x02

#define CFG_PRIM_RX  0x01
#define CFG_PWR_UP   0x02
#define CFG_CRCO     0x04
#define CFG_EN_CRC   0x08

#define ST_MAX_RT    0x10
#define ST_TX_DS     0x20
#define ST_RX_DR     0x40

#endif
