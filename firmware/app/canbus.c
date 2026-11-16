#include "canbus.h"

#include <string.h>

#include "app.h"
#include "config.h"
#include "telemetry.h"

static struct {
    uint8_t tick;
    uint8_t hb_counter;
    bool    was_bus_off;
} c;

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16)); }

void canbus_init(uint32_t now)
{
    memset(&c, 0, sizeof c);
    V.payload_last_ms = now;
}

void canbus_rx(uint32_t now)
{
    can_frame_t f;
    while (hal_can_receive(&f)) {
        if (f.id == CAN_ID_PAYLOAD_HB)
            V.payload_last_ms = now;
    }

    can_health_t h;
    hal_can_health(&h);
    if (h.bus_off && !c.was_bus_off) {
        V.can_bus_off_count++;
        telemetry_event(EVT_CAN_BUS_OFF, V.can_bus_off_count);
    }
    c.was_bus_off = h.bus_off;
}

void canbus_task(uint32_t now)
{
    can_frame_t f;

    f.id = CAN_ID_ALT_TEMP; /* every call: 5 Hz */
    f.len = 8;
    put32(&f.data[0], (uint32_t)V.alt_cm);
    put16(&f.data[4], (uint16_t)V.board_temp_cdeg);
    put16(&f.data[6], (uint16_t)V.climb_cms);
    (void)hal_can_send(&f); /* if all mailboxes are full the bus is in trouble: health reports it */

    if (++c.tick < 5)
        return;
    c.tick = 0;

    f.id = CAN_ID_HEARTBEAT; /* 1 Hz */
    f.len = 8;
    f.data[0] = (uint8_t)V.phase;
    f.data[1] = (uint8_t)V.power;
    put16(&f.data[2], V.faults);
    put16(&f.data[4], (uint16_t)(now / 1000u));
    f.data[6] = (uint8_t)V.reset_cause;
    f.data[7] = c.hb_counter++;
    (void)hal_can_send(&f);

    f.id = CAN_ID_POWER; /* 1 Hz */
    f.len = 8;
    put16(&f.data[0], V.vbat_mv);
    put16(&f.data[2], (uint16_t)V.ibat_ma);
    put16(&f.data[4], (uint16_t)(V.used_uah / 1000u));
    f.data[6] = V.heater_duty_pct;
    f.data[7] = (uint8_t)V.power;
    (void)hal_can_send(&f);
}
