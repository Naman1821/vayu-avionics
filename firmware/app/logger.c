#include "logger.h"

#include <string.h>

#include "app.h"
#include "config.h"
#include "crc.h"
#include "w25q.h"

static struct {
    bool         ok, full, erasing;
    uint32_t     next_slot; /* index of the next free 64-byte slot */
    uint32_t     seq;
    log_record_t queue[LOG_QUEUE_LEN];
    uint8_t      q_head, q_count;
    uint32_t     dropped;
    uint8_t      tick;
} lg;

static bool slot_empty(uint32_t slot)
{
    uint8_t buf[LOG_RECORD_SIZE];
    if (w25q_read(slot * LOG_RECORD_SIZE, buf, sizeof buf) != HAL_OK)
        return false;
    for (unsigned i = 0; i < sizeof buf; i++)
        if (buf[i] != 0xFF)
            return false;
    return true;
}

static bool record_valid(const log_record_t *r)
{
    return r->magic == LOG_MAGIC && r->crc == crc16_ccitt((const uint8_t *)r + 4, LOG_RECORD_SIZE - 4);
}

void logger_init(log_resume_t *resume)
{
    memset(&lg, 0, sizeof lg);
    memset(resume, 0, sizeof *resume);

    uint8_t id[3] = { 0 };
    if (w25q_read_id(id) != HAL_OK || id[0] != 0xEF || id[1] != 0x40 || id[2] != 0x18)
        return; /* no flash: fly without a black box, and say so (FAULT_FLASH) */
    lg.ok = true;

    /* Records are written front to back, so used slots are contiguous.
     * Binary search for the first empty one: ~18 reads for 262,144 slots. */
    uint32_t lo = 0, hi = LOG_SLOTS;
    while (lo < hi) {
        uint32_t mid = lo + (hi - lo) / 2;
        if (slot_empty(mid))
            hi = mid;
        else
            lo = mid + 1;
    }
    lg.next_slot = lo;
    lg.full = (lo >= LOG_SLOTS);

    /* Newest good record. Power can die mid-write, so the very last one may be
     * torn (bad CRC): look a few back. */
    for (uint32_t back = 1; back <= 4 && back <= lo; back++) {
        log_record_t r;
        if (w25q_read((lo - back) * LOG_RECORD_SIZE, (uint8_t *)&r, sizeof r) == HAL_OK && record_valid(&r)) {
            lg.seq = r.seq + 1;
            resume->valid = true;
            resume->phase = r.phase;
            resume->ground_alt_cm = r.ground_alt_cm;
            resume->seq = r.seq;
            break;
        }
    }
}

static void fill(log_record_t *r, uint8_t type)
{
    memset(r, 0, sizeof *r);
    r->magic = LOG_MAGIC;
    r->t_ms = hal_millis();
    r->type = type;
    r->phase = (uint8_t)V.phase;
    r->faults = V.faults;
    r->alt_cm = V.alt_cm;
    r->gps_alt_cm = V.gps_alt_cm;
    r->lat_e7 = V.lat_e7;
    r->lon_e7 = V.lon_e7;
    r->pressure_pa = V.pressure_pa;
    r->ground_alt_cm = V.ground_alt_cm;
    r->board_temp_cdeg = V.board_temp_cdeg;
    r->vbat_mv = V.vbat_mv;
    r->ibat_ma = V.ibat_ma;
    memcpy(r->acc_mg, V.acc_mg, sizeof r->acc_mg);
    memcpy(r->gyro_ddps, V.gyro_ddps, sizeof r->gyro_ddps);
    r->climb_cms = V.climb_cms;
    r->heater_duty = V.heater_duty_pct;
    r->gps_num_sv = V.gps_num_sv;
    r->gps_fix = V.gps_fix_type;
    r->info = (uint8_t)((V.alt_src & 0x3) | ((V.reset_cause & 0x7) << 2) |
                        ((V.heater_on ? 1 : 0) << 5) | ((V.power & 0x3) << 6));
}

static void enqueue(uint8_t type)
{
    if (!lg.ok || lg.full)
        return;
    if (lg.q_count == LOG_QUEUE_LEN) {
        lg.dropped++;
        return;
    }
    log_record_t *r = &lg.queue[(lg.q_head + lg.q_count) % LOG_QUEUE_LEN];
    fill(r, type);
    r->seq = lg.seq++;
    r->crc = crc16_ccitt((const uint8_t *)r + 4, LOG_RECORD_SIZE - 4);
    lg.q_count++;
}

void logger_event(uint8_t type)
{
    enqueue(type);
}

void logger_task(uint32_t now)
{
    (void)now;
    bool flying = V.phase == PHASE_ASCENT || V.phase == PHASE_CRUISE || V.phase == PHASE_DESCENT;
    if (flying || ++lg.tick >= 10) { /* 10 Hz in the air, 1 Hz on the ground */
        lg.tick = 0;
        enqueue(REC_PERIODIC);
    }
}

void logger_service(uint32_t now)
{
    (void)now;
    if (!lg.ok || lg.q_count == 0 || w25q_busy())
        return;
    if (lg.erasing) /* the erase just finished */
        lg.erasing = false;
    if (lg.next_slot >= LOG_SLOTS) {
        lg.full = true;
        return;
    }
    /* 64-byte records never straddle a 256-byte page */
    if (w25q_program(lg.next_slot * LOG_RECORD_SIZE, (const uint8_t *)&lg.queue[lg.q_head],
                     LOG_RECORD_SIZE) != HAL_OK) {
        lg.ok = false; /* reported as FAULT_FLASH */
        return;
    }
    lg.next_slot++;
    lg.q_head = (uint8_t)((lg.q_head + 1) % LOG_QUEUE_LEN);
    lg.q_count--;
}

bool logger_erase(void)
{
    if (!lg.ok || w25q_erase_chip() != HAL_OK)
        return false;
    lg.erasing = true;
    lg.next_slot = 0;
    lg.full = false;
    return true;
}

bool     logger_ok(void)        { return lg.ok; }
bool     logger_full(void)      { return lg.full; }
uint32_t logger_next_slot(void) { return lg.next_slot; }
