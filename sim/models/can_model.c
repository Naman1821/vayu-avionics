/*
 * CAN model: our bxCAN controller's error counters and the payload node.
 *
 * With CAN-H shorted, every frame we try to send fails and is retried
 * automatically. Each failure adds 8 to the transmit error counter (TEC):
 * at 128 the node is error-passive, past 255 it goes bus-off and stops
 * driving the bus. With ABOM set it rejoins after 128 x 11 recessive bits
 * once the bus is healthy again.
 */
#include <string.h>

#include "config.h"
#include "models.h"
#include "sim.h"

#define FIFO_LEN 16

static struct {
    int      tec, rec;
    bool     bus_off;
    int      pending;       /* frames stuck in our 3 TX mailboxes */
    uint64_t recover_at;
    can_frame_t fifo[FIFO_LEN];
    int      head, count;
    uint64_t next_payload_hb;
    uint8_t  payload_counter;
} c;

void can_model_reset(void)
{
    memset(&c, 0, sizeof c);
    c.next_payload_hb = sim_ms + 1000;
}

bool can_model_send(const can_frame_t *f)
{
    (void)f;
    if (c.bus_off || c.pending >= 3)
        return false;
    if (F.can_fault) {
        c.pending++; /* will keep failing and retrying */
        return true;
    }
    if (c.tec > 0) c.tec--; /* successful transmission */
    return true;
}

bool can_model_receive(can_frame_t *f)
{
    if (c.count == 0)
        return false;
    *f = c.fifo[c.head];
    c.head = (c.head + 1) % FIFO_LEN;
    c.count--;
    return true;
}

void can_model_health(can_health_t *h)
{
    h->tx_errors = (uint8_t)(c.tec > 255 ? 255 : c.tec);
    h->rx_errors = (uint8_t)(c.rec > 255 ? 255 : c.rec);
    h->error_passive = c.tec >= 128 || c.rec >= 128;
    h->bus_off = c.bus_off;
}

void can_model_step(void)
{
    if (F.can_fault) {
        if (!c.bus_off && c.pending > 0) {
            c.tec += 16; /* two failed attempts per millisecond */
            if (c.tec > 255) {
                c.bus_off = true;
                c.pending = 0; /* mailboxes aborted */
                sim_event("CAN: transmit error counter passed 255, controller is bus-off");
            }
        }
        if (c.rec < 127) c.rec += 1;
        c.recover_at = sim_ms + 3; /* 128 x 11 recessive bits at 500 kbit/s = 2.8 ms */
    } else {
        if (c.bus_off && sim_ms >= c.recover_at) {
            c.bus_off = false;
            c.tec = 0;
            c.rec = 0;
            sim_event("CAN: bus healthy again, automatic bus-off recovery");
        }
        if (c.pending > 0) { c.pending = 0; c.tec = c.tec > 8 ? c.tec - 8 : 0; }
        if (c.rec > 0) c.rec--;
    }

    /* the payload node sends its heartbeat once a second */
    if (sim_ms >= c.next_payload_hb) {
        c.next_payload_hb += 1000;
        if (!F.can_fault && !c.bus_off && c.count < FIFO_LEN) {
            can_frame_t f = { CAN_ID_PAYLOAD_HB, 2, { 1, c.payload_counter++ } };
            c.fifo[(c.head + c.count) % FIFO_LEN] = f;
            c.count++;
        }
    }
}
