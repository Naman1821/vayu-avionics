/*
 * u-blox M8 model.
 *
 * Starts like a real receiver out of the box: NMEA text at 9600 baud, 1 Hz,
 * no NAV-PVT, "portable" dynamic model. It accepts UBX CFG commands and
 * answers ACK/NAK, and it applies the dynamic-model altitude limit: in
 * "portable" mode it stops giving fixes above 12 km.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "models.h"
#include "sim.h"
#include "ublox.h"
#include "world.h"

static struct {
    bool     powered;
    uint64_t ready_ms;       /* boots ~0.5 s after power                 */
    uint64_t fix_ready_ms;   /* time-to-first-fix                        */
    bool     out_ubx, out_nmea;
    uint16_t rate_ms;
    uint8_t  pvt_rate;
    uint8_t  dyn_model;      /* 0 = portable                              */
    uint64_t next_epoch_ms;
    bool     was_outage;
    double   alt_err, num_sv;
    /* UBX input parser (independent of the firmware's) */
    uint8_t  st, cls, id;
    uint16_t len, idx;
    uint8_t  buf[128];
} g;

static void defaults(void)
{
    g.out_ubx = true; /* M8 default UART1 output: UBX + NMEA */
    g.out_nmea = true;
    g.rate_ms = 1000;
    g.pvt_rate = 0;
    g.dyn_model = 0;
}

void ublox_model_power(bool on, bool cold_start)
{
    g.powered = on;
    if (!on)
        return;
    defaults(); /* no saved configuration: everything is back to factory settings */
    g.st = 0;
    g.ready_ms = sim_ms + 500;
    g.fix_ready_ms = sim_ms + (cold_start ? 26000 : 2000); /* backup battery keeps ephemeris */
    g.next_epoch_ms = g.ready_ms;
    g.num_sv = 14;
}

int ublox_model_dyn_model(void) { return g.dyn_model; }

static double altitude_limit(void)
{
    switch (g.dyn_model) {
    case 6: case 7: case 8: return 50000.0; /* airborne <1g, <2g, <4g */
    default: return 12000.0;                /* portable, pedestrian, automotive ... */
    }
}

bool ublox_model_has_fix(void)
{
    return g.powered && sim_ms >= g.fix_ready_ms && !F.gps_outage && W.alt_m <= altitude_limit();
}

static void send(uint8_t cls, uint8_t id, const uint8_t *p, uint16_t len)
{
    uint8_t frame[8 + 128];
    sim_uart_push_rx(UART_GPS, frame, ubx_frame(cls, id, p, len, frame));
}

static void ack(bool ok, uint8_t id)
{
    uint8_t p[2] = { UBX_CLS_CFG, id };
    send(UBX_CLS_ACK, ok ? UBX_ACK_ACK : UBX_ACK_NAK, p, 2);
}

static void handle(void)
{
    if (g.cls != UBX_CLS_CFG)
        return;
    const uint8_t *p = g.buf;
    switch (g.id) {
    case UBX_CFG_PRT:
        if (g.len == 20 && p[0] == 1) {
            uint16_t out = (uint16_t)(p[14] | (p[15] << 8));
            g.out_ubx = out & 0x01;
            g.out_nmea = (out & 0x02) != 0;
            ack(true, g.id);
        } else ack(false, g.id);
        break;
    case UBX_CFG_RATE:
        if (g.len == 6) { g.rate_ms = (uint16_t)(p[0] | (p[1] << 8)); ack(true, g.id); }
        else ack(false, g.id);
        break;
    case UBX_CFG_MSG:
        if (g.len == 3) {
            if (p[0] == UBX_CLS_NAV && p[1] == UBX_NAV_PVT) g.pvt_rate = p[2];
            ack(true, g.id);
        } else ack(false, g.id);
        break;
    case UBX_CFG_NAV5:
        if (g.len == 0) { /* poll: report current settings */
            uint8_t r[36] = { 0xFF, 0xFF, g.dyn_model, 3 };
            send(UBX_CLS_CFG, UBX_CFG_NAV5, r, 36);
        } else if (g.len == 36) {
            bool wants_airborne = p[2] >= 6 && p[2] <= 8;
            if (F.gps_rejects_airborne && wants_airborne) { ack(false, g.id); break; }
            if (p[0] & 0x01) g.dyn_model = p[2];
            ack(true, g.id);
        } else ack(false, g.id);
        break;
    default:
        ack(false, g.id);
    }
}

void ublox_model_rx(const uint8_t *d, uint16_t n)
{
    if (!g.powered || sim_ms < g.ready_ms)
        return; /* still booting: the bytes are lost */
    for (uint16_t i = 0; i < n; i++) {
        uint8_t b = d[i];
        switch (g.st) {
        case 0: g.st = (b == 0xB5) ? 1 : 0; break;
        case 1: g.st = (b == 0x62) ? 2 : 0; break;
        case 2: g.cls = b; g.st = 3; break;
        case 3: g.id = b; g.st = 4; break;
        case 4: g.len = b; g.st = 5; break;
        case 5:
            g.len |= (uint16_t)(b << 8);
            g.idx = 0;
            g.st = g.len > sizeof g.buf ? 0 : (g.len ? 6 : 7);
            break;
        case 6: g.buf[g.idx++] = b; if (g.idx == g.len) g.st = 7; break;
        case 7: g.st = 8; break; /* checksum A (trusted: the UART link is short) */
        case 8: handle(); g.st = 0; break;
        }
    }
}

static void put32(uint8_t *p, int32_t v)
{
    for (int i = 0; i < 4; i++) p[i] = (uint8_t)((uint32_t)v >> (8 * i));
}

static void send_pvt(bool fix)
{
    uint8_t p[92];
    memset(p, 0, sizeof p);
    put32(&p[0], (int32_t)(sim_ms % 604800000ULL)); /* iTOW */
    p[20] = fix ? 3 : 0;
    p[21] = fix ? 0x01 : 0x00;
    p[23] = (uint8_t)(fix ? g.num_sv : 2);
    put32(&p[24], (int32_t)lround((W.lon_deg + sim_noise(1.5e-5)) * 1e7));
    put32(&p[28], (int32_t)lround((W.lat_deg + sim_noise(1.5e-5)) * 1e7));
    put32(&p[32], (int32_t)lround((W.alt_m + g.alt_err - 30.0) * 1000)); /* ellipsoid (geoid ~ -30 m) */
    put32(&p[36], (int32_t)lround((W.alt_m + g.alt_err) * 1000));        /* mean sea level */
    put32(&p[40], fix ? 2500 : 999999);
    put32(&p[44], fix ? 4500 : 999999);
    put32(&p[56], (int32_t)lround(-W.vz * 1000));
    send(UBX_CLS_NAV, UBX_NAV_PVT, p, sizeof p);
}

void ublox_model_step(void)
{
    if (!g.powered || sim_ms < g.next_epoch_ms)
        return;
    g.next_epoch_ms += g.rate_ms ? g.rate_ms : 1000;

    if (F.gps_outage) g.was_outage = true;
    else if (g.was_outage) { /* signal back: re-acquire */
        g.was_outage = false;
        if (g.fix_ready_ms < sim_ms + 3000) g.fix_ready_ms = sim_ms + 3000;
    }
    /* slowly wandering vertical error (~4 m) and satellite count */
    g.alt_err = 0.98 * g.alt_err + sim_noise(0.8);
    g.num_sv += sim_noise(0.3);
    if (g.num_sv < 9) g.num_sv = 9;
    if (g.num_sv > 18) g.num_sv = 18;

    bool fix = ublox_model_has_fix();
    if (g.out_nmea) { /* a GGA sentence: the firmware's UBX parser must skip it */
        char s[96];
        int n = snprintf(s, sizeof s, "$GNGGA,%06u.00,3418.000,N,07736.000,E,%d,%02d,0.9,%.1f,M,-30.0,M,,*5A\r\n",
                         (unsigned)(sim_ms / 1000 % 86400), fix ? 1 : 0, fix ? (int)g.num_sv : 0, W.alt_m);
        sim_uart_push_rx(UART_GPS, (const uint8_t *)s, (uint16_t)n);
    }
    if (g.out_ubx && g.pvt_rate)
        send_pvt(fix);
}
