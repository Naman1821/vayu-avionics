#include "flight_phase.h"

#include <stdlib.h>
#include <string.h>

#include "app.h"
#include "config.h"
#include "telemetry.h"

#define HIST_LEN (CLIMB_HISTORY_S * 1000 / PHASE_PERIOD_MS + 1) /* 21 samples = 2 s */
#define END_N    5                                               /* 0.5 s at each end */

enum { C_CLIMB, C_SINK, C_LEVEL, C_STILL, C_COUNT };

static struct {
    int32_t      hist[HIST_LEN];
    uint8_t      count, head;
    alt_source_t last_src;
    bool         active[C_COUNT];
    uint32_t     since[C_COUNT];
} f;

const char *phase_name(int phase)
{
    static const char *names[] = { "BOOT", "PREFLIGHT", "ASCENT", "CRUISE", "DESCENT", "LANDED" };
    return (phase >= 0 && phase <= PHASE_LANDED) ? names[phase] : "?";
}

static void set_phase(phase_t p)
{
    if (p == V.phase)
        return;
    V.phase = p;
    telemetry_event(EVT_PHASE, p);
}

/* true once `cond` has been continuously true for `need_ms` */
static bool held(int c, bool cond, uint32_t now, uint32_t need_ms)
{
    if (!cond) {
        f.active[c] = false;
        return false;
    }
    if (!f.active[c]) {
        f.active[c] = true;
        f.since[c] = now;
    }
    return now - f.since[c] >= need_ms;
}

void phase_init(const log_resume_t *r)
{
    memset(&f, 0, sizeof f);
    V.phase = PHASE_BOOT;
    V.warm_restart = false;
    if (r->valid && (r->phase == PHASE_ASCENT || r->phase == PHASE_CRUISE || r->phase == PHASE_DESCENT)) {
        /* We rebooted in the air (brown-out, watchdog). Carry on where the log
         * left off instead of treating the current altitude as "the ground". */
        V.phase = (phase_t)r->phase;
        V.ground_alt_cm = r->ground_alt_cm;
        V.warm_restart = true;
    }
}

void phase_task(uint32_t now)
{
    /* 1. pick the altitude source: barometer first, GPS if the barometer is out */
    alt_source_t src = V.baro_ok ? ALT_BARO : (V.gps_fix ? ALT_GPS : ALT_NONE);
    if (src == ALT_BARO)
        V.alt_cm = V.baro_alt_cm;
    else if (src == ALT_GPS)
        V.alt_cm = V.gps_alt_cm;
    if (src != f.last_src)
        f.count = 0; /* sources differ by metres: don't read a switch as a climb */
    f.last_src = src;
    V.alt_src = src;
    if (src == ALT_NONE)
        return; /* blind: hold the phase */

    /* 2. climb rate over the last 2 s: average of the newest 0.5 s minus the
     *    average of the oldest 0.5 s, divided by the 1.6 s between them.
     *    Averaging the ends makes it ~2x less noisy than "last minus first". */
    f.hist[f.head] = V.alt_cm;
    f.head = (uint8_t)((f.head + 1) % HIST_LEN);
    if (f.count < HIST_LEN)
        f.count++;
    if (f.count == HIST_LEN) {
        int32_t newest = 0, oldest = 0;
        for (int k = 0; k < END_N; k++) {
            newest += f.hist[(f.head + HIST_LEN - 1 - k) % HIST_LEN];
            oldest += f.hist[(f.head + k) % HIST_LEN];
        }
        V.climb_cms = (int16_t)((newest - oldest) * (1000 / PHASE_PERIOD_MS) / (END_N * (HIST_LEN - END_N)));
    } else {
        V.climb_cms = 0;
    }

    /* 3. conditions, each with its own timer */
    int32_t climb = V.climb_cms;
    bool climbing = held(C_CLIMB, climb > LAUNCH_CLIMB_CMS, now, LAUNCH_CONFIRM_MS);
    bool sinking  = held(C_SINK, climb < DESCENT_SINK_CMS, now, DESCENT_CONFIRM_MS);
    bool level    = held(C_LEVEL, labs(climb) < LEVEL_BAND_CMS, now, LEVEL_CONFIRM_MS);
    bool still    = held(C_STILL, labs(climb) < STILL_BAND_CMS, now, STILL_CONFIRM_MS);
    int32_t agl   = V.alt_cm - V.ground_alt_cm;

    /* 4. transitions */
    switch (V.phase) {
    case PHASE_BOOT:
        V.ground_alt_cm = V.alt_cm;
        set_phase(PHASE_PREFLIGHT);
        break;
    case PHASE_PREFLIGHT:
        if (labs(climb) < STILL_BAND_CMS) /* follow slow weather drift, only when still */
            V.ground_alt_cm += (V.alt_cm - V.ground_alt_cm) / 50;
        if (climbing || agl > LAUNCH_HEIGHT_CM)
            set_phase(PHASE_ASCENT);
        break;
    case PHASE_ASCENT:
        if (sinking) set_phase(PHASE_DESCENT);
        else if (level) set_phase(PHASE_CRUISE);
        break;
    case PHASE_CRUISE:
        if (sinking) set_phase(PHASE_DESCENT);
        else if (climbing) set_phase(PHASE_ASCENT);
        break;
    case PHASE_DESCENT:
        if (still && labs(agl) < NEAR_GROUND_CM) set_phase(PHASE_LANDED);
        else if (level && agl > NEAR_GROUND_CM) set_phase(PHASE_CRUISE);
        else if (climbing) set_phase(PHASE_ASCENT);
        break;
    case PHASE_LANDED:
        break;
    }
}
