#include "telemetry.h"

#include <stdlib.h>
#include <string.h>

#include "app.h"
#include "config.h"
#include "crc.h"
#include "logger.h"
#include "ublox.h"

static struct {
    uint8_t tick;
    bool    gps_config_reported;
    /* uplink parser */
    uint8_t state, len, type, idx;
    uint8_t buf[TM_MAX_PAYLOAD + 2];
} t;

uint16_t tm_encode(uint8_t type, const void *payload, uint8_t len, uint8_t *out)
{
    out[0] = TM_SYNC1;
    out[1] = TM_SYNC2;
    out[2] = len;
    out[3] = type;
    if (len)
        memcpy(&out[4], payload, len);
    uint16_t crc = crc16_ccitt(&out[2], (uint32_t)len + 2);
    out[4 + len] = (uint8_t)crc;
    out[5 + len] = (uint8_t)(crc >> 8);
    return (uint16_t)(len + 6);
}

static void send(uint8_t type, const void *payload, uint8_t len)
{
    uint8_t frame[TM_MAX_PAYLOAD + 6];
    hal_uart_write(UART_RADIO, frame, tm_encode(type, payload, len, frame));
}

void telemetry_event(uint8_t code, uint16_t arg)
{
    tm_event_t e = { hal_millis(), code, arg };
    send(TM_EVENT, &e, sizeof e);
}

void telemetry_init(void)
{
    memset(&t, 0, sizeof t);
}

void telemetry_task(uint32_t now)
{
    /* one-time note when the GPS airborne mode is confirmed (or re-confirmed) */
    if (ublox_configured() != t.gps_config_reported) {
        t.gps_config_reported = ublox_configured();
        telemetry_event(EVT_GPS_CONFIG, t.gps_config_reported);
    }

    /* low power: send status every 4th slot (0.5 Hz) */
    if (V.power != POWER_NORMAL && (++t.tick % 4) != 0)
        return;

    tm_status_t s;
    s.t_ms = now;
    s.phase = (uint8_t)V.phase;
    s.power = (uint8_t)V.power;
    s.faults = V.faults;
    s.alt_cm = V.alt_cm;
    s.climb_cms = V.climb_cms;
    s.gps_alt_cm = V.gps_alt_cm;
    s.lat_e7 = V.lat_e7;
    s.lon_e7 = V.lon_e7;
    s.gps_num_sv = V.gps_num_sv;
    s.gps_fix = V.gps_fix_type;
    s.board_temp_cdeg = V.board_temp_cdeg;
    s.heater_duty = V.heater_duty_pct;
    s.vbat_mv = V.vbat_mv;
    s.ibat_ma = V.ibat_ma;
    s.used_mah = (uint16_t)(V.used_uah / 1000u);
    s.reset_cause = (uint8_t)V.reset_cause;
    s.alt_src = (uint8_t)V.alt_src;
    send(TM_STATUS, &s, sizeof s);
}

static void handle_command(uint8_t type, const uint8_t *p, uint8_t len)
{
    if (type == TC_PING) {
        telemetry_event(EVT_PONG, 0);
    } else if (type == TC_ERASE_LOG && len == 4) {
        uint32_t magic = (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
        bool on_ground = (V.phase == PHASE_PREFLIGHT || V.phase == PHASE_LANDED) &&
                         labs(V.alt_cm - V.ground_alt_cm) < 5000;
        /* A stray or spoofed command must never wipe the black box in flight. */
        if (magic == TC_ERASE_MAGIC && on_ground && logger_erase())
            telemetry_event(EVT_ERASE_STARTED, 0);
        else
            telemetry_event(EVT_ERASE_REFUSED, V.phase);
    }
}

void telemetry_rx(uint32_t now)
{
    (void)now;
    int c;
    while ((c = hal_uart_read(UART_RADIO)) >= 0) {
        uint8_t b = (uint8_t)c;
        switch (t.state) {
        case 0: if (b == TM_SYNC1) t.state = 1; break;
        case 1: t.state = (b == TM_SYNC2) ? 2 : (b == TM_SYNC1 ? 1 : 0); break;
        case 2:
            t.len = b;
            t.state = (b <= TM_MAX_PAYLOAD) ? 3 : 0;
            break;
        case 3: t.type = b; t.idx = 0; t.state = 4; break;
        case 4:
            t.buf[t.idx++] = b;
            if (t.idx == t.len + 2) { /* payload + 2 CRC bytes */
                uint8_t hdr[TM_MAX_PAYLOAD + 2];
                hdr[0] = t.len;
                hdr[1] = t.type;
                memcpy(&hdr[2], t.buf, t.len);
                uint16_t crc = crc16_ccitt(hdr, (uint32_t)t.len + 2);
                if (crc == (uint16_t)(t.buf[t.len] | (t.buf[t.len + 1] << 8)))
                    handle_command(t.type, t.buf, t.len);
                t.state = 0;
            }
            break;
        }
    }
}
