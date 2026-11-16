#include "ublox.h"

#include <string.h>

#include "config.h"
#include "hal.h"

/* ---- configuration messages (all fields little-endian) ------------------ */
static const uint8_t CFG_PRT_UART1[20] = {
    0x01, 0x00, 0x00, 0x00, /* port 1 = UART1, reserved, txReady off              */
    0xD0, 0x08, 0x00, 0x00, /* 8 data bits, no parity, 1 stop bit                  */
    0x80, 0x25, 0x00, 0x00, /* 9600 baud (unchanged)                               */
    0x03, 0x00,             /* accept UBX + NMEA                                   */
    0x01, 0x00,             /* send UBX only: NMEA would eat ~45 % of 9600 baud    */
    0x00, 0x00, 0x00, 0x00,
};
static const uint8_t CFG_RATE_5HZ[6] = { 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00 }; /* 200 ms */
static const uint8_t CFG_MSG_PVT[3]  = { UBX_CLS_NAV, UBX_NAV_PVT, 1 };       /* every fix */
static const uint8_t CFG_NAV5_AIR[36] = {
    0x01, 0x00,           /* mask: apply only the dynamic model */
    GPS_DYN_AIRBORNE_1G,  /* 6 = airborne < 1 g, valid to 50 km  */
    0x03,                 /* fix mode auto 2D/3D (ignored by mask) */
};

typedef struct {
    uint8_t        id;
    const uint8_t *payload;
    uint8_t        len;
} cfg_step_t;

static const cfg_step_t STEPS[] = {
    { UBX_CFG_PRT,  CFG_PRT_UART1, sizeof CFG_PRT_UART1 },
    { UBX_CFG_RATE, CFG_RATE_5HZ,  sizeof CFG_RATE_5HZ },
    { UBX_CFG_MSG,  CFG_MSG_PVT,   sizeof CFG_MSG_PVT },
    { UBX_CFG_NAV5, CFG_NAV5_AIR,  sizeof CFG_NAV5_AIR },
};
#define N_STEPS (sizeof STEPS / sizeof STEPS[0])

typedef enum { CFG_SEND, CFG_WAIT_ACK, CFG_VERIFY, CFG_WAIT_VERIFY, CFG_DONE } cfg_state_t;

static struct {
    /* byte parser */
    uint8_t  pstate, cls, id, ck_a, ck_b;
    uint16_t len, idx;
    uint8_t  buf[UBX_MAX_PAYLOAD];
    /* newest results */
    ubx_pvt_t pvt;
    bool      pvt_new;
    uint32_t  last_msg_ms;
    bool      ack_new, ack_ok;
    uint8_t   ack_id;
    bool      nav5_new;
    uint8_t   nav5_dyn;
    /* configuration state machine */
    cfg_state_t cfg;
    uint8_t     step, tries;
    uint32_t    sent_ms;
    bool        configured, failed;
} u;

/* ---- frames ------------------------------------------------------------ */
void ubx_checksum(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    uint8_t a = 0, b = 0; /* 8-bit Fletcher over class, id, length, payload */
    for (uint16_t i = 0; i < len; i++) {
        a = (uint8_t)(a + data[i]);
        b = (uint8_t)(b + a);
    }
    *ck_a = a;
    *ck_b = b;
}

uint16_t ubx_frame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len, uint8_t *out)
{
    out[0] = 0xB5;
    out[1] = 0x62;
    out[2] = cls;
    out[3] = id;
    out[4] = (uint8_t)len;
    out[5] = (uint8_t)(len >> 8);
    if (len)
        memcpy(&out[6], payload, len);
    ubx_checksum(&out[2], (uint16_t)(4 + len), &out[6 + len], &out[7 + len]);
    return (uint16_t)(8 + len);
}

static void send(uint8_t id, const uint8_t *payload, uint16_t len)
{
    uint8_t frame[8 + UBX_MAX_PAYLOAD];
    hal_uart_write(UART_GPS, frame, ubx_frame(UBX_CLS_CFG, id, payload, len, frame));
}

/* ---- parser ------------------------------------------------------------- */
static int32_t le32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24);
}

static void handle_frame(uint32_t now)
{
    u.last_msg_ms = now;
    if (u.cls == UBX_CLS_NAV && u.id == UBX_NAV_PVT && u.len >= 92) {
        const uint8_t *p = u.buf;
        u.pvt.fix_type     = p[20];
        u.pvt.flags        = p[21];
        u.pvt.num_sv       = p[23];
        u.pvt.lon_e7       = le32(p + 24);
        u.pvt.lat_e7       = le32(p + 28);
        u.pvt.hmsl_mm      = le32(p + 36);
        u.pvt.h_acc_mm     = (uint32_t)le32(p + 40);
        u.pvt.v_acc_mm     = (uint32_t)le32(p + 44);
        u.pvt.vel_down_mms = le32(p + 56);
        u.pvt_new = true;
    } else if (u.cls == UBX_CLS_ACK && u.len == 2 && u.buf[0] == UBX_CLS_CFG) {
        u.ack_new = true;
        u.ack_ok  = (u.id == UBX_ACK_ACK);
        u.ack_id  = u.buf[1];
    } else if (u.cls == UBX_CLS_CFG && u.id == UBX_CFG_NAV5 && u.len == 36) {
        u.nav5_new = true; /* answer to our read-back poll */
        u.nav5_dyn = u.buf[2];
    }
}

static void parse_byte(uint8_t b, uint32_t now)
{
    switch (u.pstate) {
    case 0: if (b == 0xB5) u.pstate = 1; break;                       /* sync 1   */
    case 1: u.pstate = (b == 0x62) ? 2 : (b == 0xB5 ? 1 : 0); break;  /* sync 2   */
    case 2: u.cls = b; u.ck_a = b; u.ck_b = b; u.pstate = 3; break;   /* class    */
    case 3: u.id = b; u.ck_a += b; u.ck_b += u.ck_a; u.pstate = 4; break;
    case 4: u.len = b; u.ck_a += b; u.ck_b += u.ck_a; u.pstate = 5; break;
    case 5:
        u.len |= (uint16_t)(b << 8);
        u.ck_a += b; u.ck_b += u.ck_a;
        u.idx = 0;
        if (u.len > UBX_MAX_PAYLOAD) u.pstate = 0;   /* a message we don't keep: resync */
        else u.pstate = u.len ? 6 : 7;
        break;
    case 6:                                          /* payload */
        u.buf[u.idx++] = b;
        u.ck_a += b; u.ck_b += u.ck_a;
        if (u.idx == u.len) u.pstate = 7;
        break;
    case 7: u.pstate = (b == u.ck_a) ? 8 : 0; break;
    case 8:
        if (b == u.ck_b) handle_frame(now);          /* bad checksum: silently dropped */
        u.pstate = 0;
        break;
    }
}

/* ---- configuration state machine -------------------------------------- */
static void next_step(void)
{
    u.tries = 0;
    u.step++;
    u.cfg = (u.step < N_STEPS) ? CFG_SEND : CFG_VERIFY;
}

static void retry_or_move_on(void)
{
    if (++u.tries > GPS_CFG_RETRIES)
        next_step(); /* give up on this one: the read-back will show if it mattered */
    else
        u.cfg = CFG_SEND;
}

static void config_step(uint32_t now)
{
    switch (u.cfg) {
    case CFG_SEND:
        u.ack_new = false;
        send(STEPS[u.step].id, STEPS[u.step].payload, STEPS[u.step].len);
        u.sent_ms = now;
        u.cfg = CFG_WAIT_ACK;
        break;
    case CFG_WAIT_ACK:
        if (u.ack_new && u.ack_id == STEPS[u.step].id) {
            u.ack_new = false;
            if (u.ack_ok) next_step();
            else retry_or_move_on();           /* NAK */
        } else if (now - u.sent_ms > GPS_ACK_TIMEOUT_MS) {
            retry_or_move_on();                /* no answer */
        }
        break;
    case CFG_VERIFY:
        u.nav5_new = false;
        send(UBX_CFG_NAV5, 0, 0);              /* empty payload = "what is your setting?" */
        u.sent_ms = now;
        u.cfg = CFG_WAIT_VERIFY;
        break;
    case CFG_WAIT_VERIFY:
        if (u.nav5_new) {
            u.configured = (u.nav5_dyn == GPS_DYN_AIRBORNE_1G);
            u.failed = !u.configured;
            u.sent_ms = now;
            u.cfg = CFG_DONE;
        } else if (now - u.sent_ms > GPS_ACK_TIMEOUT_MS) {
            if (++u.tries > GPS_CFG_RETRIES) {
                u.failed = true;
                u.sent_ms = now;
                u.cfg = CFG_DONE;
            } else {
                u.cfg = CFG_VERIFY;
            }
        }
        break;
    case CFG_DONE:
        if (u.failed && now - u.sent_ms > GPS_CFG_RETRY_MS)
            ublox_restart_config(now);         /* keep trying while flying */
        break;
    }
}

/* ---- public ------------------------------------------------------------- */
void ublox_init(uint32_t now)
{
    memset(&u, 0, sizeof u);
    u.last_msg_ms = now;
    u.sent_ms = now;
    u.cfg = CFG_SEND;
}

void ublox_restart_config(uint32_t now)
{
    u.cfg = CFG_SEND;
    u.step = 0;
    u.tries = 0;
    u.configured = false; /* unknown until read back again; "failed" stays as it was */
    u.sent_ms = now;
}

void ublox_poll(uint32_t now)
{
    int c;
    for (int budget = 512; budget > 0 && (c = hal_uart_read(UART_GPS)) >= 0; budget--)
        parse_byte((uint8_t)c, now);
    config_step(now);
}

bool ublox_take_pvt(ubx_pvt_t *out)
{
    if (!u.pvt_new)
        return false;
    *out = u.pvt;
    u.pvt_new = false;
    return true;
}

uint32_t ublox_last_msg_ms(void)  { return u.last_msg_ms; }
bool     ublox_configured(void)   { return u.configured; }
bool     ublox_config_failed(void){ return u.failed; }
bool     ublox_config_busy(void)  { return u.cfg != CFG_DONE; }
